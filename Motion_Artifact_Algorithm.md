# 脑电采集系统：运动伪迹 (Motion Artifact) 计算逻辑与代码处理机制分析

## 1. 概述
在脑电（EEG）采集过程中，受试者的头部运动或身体移动会引起电极与皮肤接触阻抗的变化，从而在微弱的脑电信号中引入极大的低频噪声，这被称为**运动伪迹 (Motion Artifact)**。

为了实时评估当前数据是否受到运动干扰，固件中集成了一套基于 **QMI8658 (六轴 IMU：三轴加速度计 + 三轴陀螺仪)** 的运动评分（Motion Score）计算机制。该机制通过分析加速度和角速度的剧烈变化程度，给出一个 `0~100` 的分数，随环境辅助数据包 (TYPE_AUX) 实时发送给上位机。

---

## 2. 数据采集流程 (Data Acquisition)
在固件中，IMU 数据的采集和处理位于主数据流循环中（见 `commands_impl.c`）。

1. **采集频率**：
   系统并非按照 EEG 的高采样率（如 250Hz）逐个处理 IMU，而是设置了一个专门的定时器，默认每隔 **20 毫秒 (50Hz)** 读取一次 QMI8658 的 XYZ 数据。
2. **读取接口**：
   调用 `QMI8658_Read_XYZ(float acc[3], float gyro[3])`，获取当前时刻的：
   * 三轴加速度 `acc` (单位: $g$, 重力加速度)
   * 三轴角速度 `gyro` (单位: $\degree/s$, 度每秒)

---

## 3. 运动特征提取 (Feature Extraction)
每采集到一组新的 IMU 数据，代码会立即计算两个核心特征值：**加速度差值 (Jerk/Delta Acc)** 和 **陀螺仪向量模长 (Gyro Magnitude)**。

### 3.1 加速度剧烈度 (Acceleration Difference)
代码计算当前加速度向量与**上一次采样**加速度向量之间的欧几里得距离差：
$$ \Delta Acc = \sqrt{(acc_x - last\_acc_x)^2 + (acc_y - last\_acc_y)^2 + (acc_z - last\_acc_z)^2} $$

* **限幅处理 (Clamping)**：为了防止极端冲击导致分数瞬间爆炸，代码对单次差值进行了限幅：
  `if (acc_diff > 0.5f) acc_diff = 0.5f;` （即限制最大变化率为 $0.5g$ / 20ms）。
* **更新缓存**：计算完成后，将当前的 `acc` 保存为下一次的 `last_acc`。

### 3.2 角速度剧烈度 (Gyroscope Magnitude)
代码计算当前角速度的向量模长（即头部旋转的绝对速率）：
$$ Gyro_{mag} = \sqrt{gyro_x^2 + gyro_y^2 + gyro_z^2} $$

* **限幅处理 (Clamping)**：同样限制了极端的旋转速率：
  `if (gyro_mag > 500.0f) gyro_mag = 500.0f;` （即限制最大角速度为 $500 \degree/s$）。

### 3.3 数据累加 (Accumulation)
特征提取后，将这两个值累加到全局的求和变量中，并增加采样计数器：
```c
acc_diff_sum += acc_diff;
gyro_mag_sum += gyro_mag;
motion_sample_count++;
```

---

## 4. 运动分数计算 (Score Calculation)
分数的计算和输出频率为 **1Hz (每秒1次)**。
当定时器检测到距离上次输出已过去 1000 毫秒时，开始结算这一秒内的运动分数。

### 4.1 计算平均特征值
首先，求出过去 1 秒内（约 50 个样本）特征的平均值：
```c
avg_acc_diff = acc_diff_sum / motion_sample_count;
avg_gyro_mag = gyro_mag_sum / motion_sample_count;
```

### 4.2 计算运动能量 (Motion Energy)
系统采用了一个加权融合公式，将加速度变化率和角速度绝对值结合成一个综合的“运动能量”指标。
```c
// Motion Energy Calculation (v2)
float motion_energy = (50.0f * avg_acc_diff) + (0.02f * avg_gyro_mag);
```
* **权重分析**：
  * 加速度权重 `50.0f`：因为日常运动中 $\Delta Acc$ 通常在 $0.01g \sim 0.1g$ 之间，乘以 50 后数值范围约在 $0.5 \sim 5.0$。
  * 陀螺仪权重 `0.02f`：日常头部转动角速度通常在 $10 \degree/s \sim 100 \degree/s$，乘以 0.02 后数值范围约在 $0.2 \sim 2.0$。
  * 两者在最终能量值中的贡献处于同一数量级，保证了无论是平移抖动还是转头，都能被敏锐捕捉。

### 4.3 分数归一化 (Normalization)
将运动能量映射到 `0~100` 的标准分数区间：
```c
float motion_score = motion_energy * 5.0f;
if (motion_score > 100.0f) motion_score = 100.0f;
if (motion_score < 0.0f) motion_score = 0.0f;
```
* 乘以 `5.0f` 是经验放大系数，使得轻微的运动（如轻微摇头）能产生明显的分数变化（例如 $2 \sim 10$ 分），而剧烈运动（如走路、剧烈晃头）能迅速达到 $100$ 分满分。

---

## 5. 数据打包与上位机显示
计算出 `motion_score` 后，固件会将其转换为 8 位无符号整数 (`uint8_t`)，并打包进 **TYPE_AUX (0x05)** 数据包中发给上位机。

**二进制协议结构 (16 Bytes Payload)**：
* `Acc_XYZ` (3 * int16, 乘以1000保留三位小数)
* `Gyro_XYZ` (3 * int16, 乘以10保留一位小数)
* `Bat_Vol` (1 * uint16, 电池电压乘以100)
* **`Score` (1 * uint8, 范围 0-100)**
* `Reserved` (1 * uint8)

**上位机 (Python GUI) 处理**：
在 `host_gui.py` 中解析到 `Score` 后：
```python
score_color = "red" if score > 20 else "green"
aux_str = f"AUX: Acc(...) Gyro(...) Bat:... Score:{score}"
self.lbl_dev_state.config(text=aux_str, foreground=score_color)
```
* **状态指示**：当 `Score > 20` 时，判定为当前处于**高运动伪迹状态**，UI 会将状态文字变为**红色**警告；当 `Score <= 20` 时，判定为安静状态，显示为**绿色**。

## 6. 总结
该算法具有计算量小、无需复杂滤波的优点，非常适合在 STM32 这种资源有限的微控制器上实时运行。通过联合评估加速度的**高频突变（差值）**和陀螺仪的**绝对旋转（模长）**，它能非常有效地标识出这段 EEG 数据是否被患者的运动所污染，为后期的脑电数据清洗（如剔除坏段）提供了直接的参考标签。