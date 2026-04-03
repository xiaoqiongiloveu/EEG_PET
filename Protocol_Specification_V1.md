# 脑电采集系统通信协议规范 V1.0

## 1. 概述
本文档定义了脑电采集下位机（Device）与上位机（Host）之间的通信协议。该协议主要用于设备配置、状态查询、高速脑电数据流传输及 OTA 升级。

- **物理接口**: USB CDC (虚拟串口)
- **字节序**: 小端模式 (Little-Endian)，除非特殊说明。
- **传输模式**: 混合模式 (二进制主导 + 兼容部分 ASCII 调试文本)。

---

## 2. 数据帧通用结构
所有二进制交互（命令下发、数据上报）均遵循以下通用帧格式：

| 偏移量 | 字段名 | 字节数 | 类型 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| 0 | `SYNC` | 2 | uint16 | 帧头同步字，固定为 `0x55 0xAA` |
| 2 | `LEN` | 2 | uint16 | `PAYLOAD` 字段的长度（字节数） |
| 4 | `TYPE` | 1 | uint8 | 数据包类型（见包类型定义） |
| 5 | `SEQ` | 2 | uint16 | 序列号，自增，用于检测丢包 |
| 7 | `TIMESTAMP` | 4 | uint32 | 下位机系统时间戳 (毫秒) |
| 11 | `PAYLOAD` | N | uint8[] | 载荷数据，长度由 `LEN` 决定 |
| 11+N | `CRC16` | 2 | uint16 | 从 `TYPE` 到 `PAYLOAD` 结束的 CRC16-CCITT 校验和 |

### 2.1 包类型定义 (TYPE)
| TYPE | 宏定义 | 方向 | 说明 |
| :--- | :--- | :--- | :--- |
| `0x00` | `BP_TYPE_CMD` | Host -> Device | 上位机下发控制指令 |
| `0xFF` | `BP_TYPE_CMD_RESP`| Device -> Host | 下位机对控制指令的确认回复 |
| `0x01` | `BP_TYPE_EEG` | Device -> Host | 脑电波形及 IMU 实时数据 |
| `0x02` | `BP_TYPE_AUX` | Device -> Host | 辅助数据 (电池电量、设备状态等) |
| `0x03` | `BP_TYPE_IMP` | Device -> Host | 阻抗测试数据 |
| `0x04` | `BP_TYPE_STATUS`| Device -> Host | 设备初始化与自检状态 |

---

## 3. 下发指令集 (TYPE = 0x00)

上位机通过发送 `TYPE = 0x00` 的包来控制下位机，其 `PAYLOAD` 的第 0 个字节为具体的**子命令 ID (CMD_ID)**，后续字节为参数。

### 3.1 子命令概览
| CMD_ID | 名称 | 说明 | PAYLOAD 结构 |
| :--- | :--- | :--- | :--- |
| `0x01` | `START_STREAM` | 启动普通实时数据流 | `[0x01]` (无参数) |
| `0x02` | `STOP_STREAM` | 停止当前采集任务 | `[0x02]` (无参数) |
| `0x03` | `SET_CONF` | 配置采集参数 | `[0x03] + [11字节配置参数]` |
| `0x04` | `GET_STATE` | 获取设备硬件状态 | `[0x04]` (无参数) |
| `0x05` | `LIST_FILES` | 列出 SD 卡内的文件 | `[0x05]` (无参数) |
| `0x0B` | `SINGLE` | 启动单通道测试 | `[0x0B]` (无参数) |
| `0x0C` | `QUAD` | 启动四通道测试 | `[0x0C]` (无参数) |
| `0x0D` | `AUTO_STREAM` | 启动带 SD 卡存储的自动流 | `[0x0D]` (无参数) |
| `0x0E` | `CHECK_LOSS` | 启动通道阻抗及丢包测试 | `[0x0E]` (无参数) |
| `0x0F` | `AUDIO_ROUTE` | 设置音频输出路由 | `[0x0F] + [1字节路由模式]` (0:两者, 1:喇叭, 2:耳机) |

### 3.2 详细指令说明

#### 3.2.1 配置参数 (SET_CONF - 0x03)
该指令用于在启动采集前设置 ADS1299 参数及采集时长。
**PAYLOAD 长度**: 12 字节 (`1字节 CMD_ID` + `11字节参数`)

| 字节偏移 (含CMD_ID) | 字段 | 类型 | 说明 |
| :--- | :--- | :--- | :--- |
| 0 | `CMD_ID` | uint8 | 固定为 `0x03` |
| 1-2 | `SPS` | uint16 | 采样率 (有效值: 250, 500, 1000, 2000, 4000, 8000, 16000) |
| 3 | `Gain` | uint8 | 增益 (有效值: 1, 2, 4, 6, 8, 12, 24) |
| 4 | `ChMask` | uint8 | 通道开启掩码。Bit0~Bit7 对应 CH1~CH8。1 为开启，0 为关闭。 |
| 5 | `InputType` | uint8 | 输入类型。`0`: Normal, `1`: Shorted (噪声测试), `5`: Test Signal |
| 6 | `BiasEn` | uint8 | 右腿驱动 (RLD) 使能。`1`: 开启, `0`: 关闭 |
| 7 | `TestSig` | uint8 | 测试信号源。`0`: 外部, `1`: 内部 |
| 8-11 | `Duration` | uint32 | 自动结束时长（秒）。`0` 表示无限时长。 |

#### 3.2.2 获取 SD 卡文件 (GET_FILE - 0x06)
该功能采用**“二进制触发 + 纯数据流透传”**的混合协议，以最大化大文件的传输速率。

1. **上位机请求 (Host -> Device)**
   * **指令格式**: 标准二进制命令帧 (`TYPE = 0x00`)
   * **PAYLOAD**: `[0x06] + [文件名字符串 (无 \0)]`
   * **示例**: 请求 `EEG_DATA.TXT`，PAYLOAD 为 `0x06 45 45 47 5F 44 41 54 41 2E 54 58 54`

2. **下位机响应数据流 (Device -> Host)**
   下位机在找到文件后，**不使用** `0x55 0xAA` 帧格式封装，而是直接按顺序发送以下内容：
   * **文件头 (Header)**: 纯文本 `OK:<文件字节数>\r\n` (例: `OK:1024\r\n`)。
   * **文件裸数据 (Raw Data)**: 直接将文件内容以字节流形式连续发送，不包含任何额外协议头或 CRC 校验。
   * **文件尾 (Footer)**: 数据发送完毕后，附加纯文本 `EOF:\r\n` 作为结束标志。

*上位机开发工程师注意：在发送 `GET_FILE` 请求后，接收端需暂时切换为流式接收模式，解析文本头后，按指定大小读取裸字节，直到匹配到 `EOF:\r\n` 后再切回二进制包解析模式。*

#### 3.2.14 AUDIO_ROUTE (0x0F)
设置 WM8978 音频芯片的输出通道路由，不立即播放音频，仅修改后续播放的默认通道。
- **发送格式**: `[0x0F] + [Route]`
  - `Route`: `0x00` (Both 喇叭和耳机), `0x01` (Speaker 仅喇叭), `0x02` (Headphone 仅耳机)
- **回复格式**: `[0x11] [0x0F] [Status] [Message]`

---

## 4. 上报数据格式

### 4.1 脑电数据包 (TYPE = 0x01: BP_TYPE_EEG)
用于在 `STREAM` 或 `AUTO_STREAM` 模式下高频上报数据。
**PAYLOAD 长度**: 47 字节。

| PAYLOAD 偏移 | 字段 | 类型 | 长度 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| 0-2 | `Status` | uint8[] | 3 | ADS1299 状态字节 (包含导联脱落标识) |
| 3-6 | `CH1_Data` | int32 | 4 | 通道 1 原始 ADC 值 (24位带符号，扩展为32位大端或系统端序，需校验) |
| 7-10 | `CH2_Data` | int32 | 4 | 通道 2 数据 |
| ... | ... | ... | ... | ... |
| 31-34 | `CH8_Data` | int32 | 4 | 通道 8 数据 |
| 35-36 | `Acc_X` | int16 | 2 | X轴加速度 (mg = val) / 1000.0 |
| 37-38 | `Acc_Y` | int16 | 2 | Y轴加速度 |
| 39-40 | `Acc_Z` | int16 | 2 | Z轴加速度 |
| 41-42 | `Gyro_X` | int16 | 2 | X轴角速度 (dps = val / 10.0) |
| 43-44 | `Gyro_Y` | int16 | 2 | Y轴角速度 |
| 45-46 | `Gyro_Z` | int16 | 2 | Z轴角速度 |

**电压转换公式**:
`V_uV = (int32_val) * (4.5 / (2^23 - 1)) / Gain * 1,000,000`

### 4.2 辅助数据包 (TYPE = 0x02: BP_TYPE_AUX)
该数据包以 **1Hz** 的频率在上报脑电数据（STREAM/AUTO_STREAM）期间穿插发送。它包含了电池电量、低频 IMU 数据以及**运动伪迹计算结果**。上位机无需自己计算运动伪迹，直接读取该包中的 `Motion_Score` 即可。

**PAYLOAD 长度**: 16 字节。

| PAYLOAD 偏移 | 字段 | 类型 | 长度 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| 0-1 | `Acc_X` | int16 | 2 | X轴加速度，单位：mg (恢复公式：`val / 1000.0` g) |
| 2-3 | `Acc_Y` | int16 | 2 | Y轴加速度，单位：mg |
| 4-5 | `Acc_Z` | int16 | 2 | Z轴加速度，单位：mg |
| 6-7 | `Gyro_X` | int16 | 2 | X轴角速度，(恢复公式：`val / 10.0` dps) |
| 8-9 | `Gyro_Y` | int16 | 2 | Y轴角速度 |
| 10-11 | `Gyro_Z` | int16 | 2 | Z轴角速度 |
| 12-13 | `Battery` | uint16 | 2 | 电池电压，(恢复公式：`val / 100.0` V) |
| 14 | `Motion_Score`| uint8 | 1 | **运动伪迹得分 (0 ~ 100)**。下位机基于近 1 秒内的加速度突变和角速度幅度综合计算得出。值越大代表运动越剧烈。 |
| 15 | `Reserved` | uint8 | 1 | 保留字节，默认为 0 |

### 4.3 阻抗数据包 (TYPE = 0x03: BP_TYPE_IMP)
在执行 `CHECK_LOSS` 或特定阻抗测试时上报。
**PAYLOAD 长度**: 16 字节。

| PAYLOAD 偏移 | 字段 | 类型 | 长度 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| 0-1 | `CH1_Imp` | uint16 | 2 | 通道 1 阻抗值，单位: kOhm |
| 2-3 | `CH2_Imp` | uint16 | 2 | 通道 2 阻抗值，单位: kOhm |
| ... | ... | ... | ... | ... |
| 14-15 | `CH8_Imp` | uint16 | 2 | 通道 8 阻抗值，单位: kOhm |

*注：若阻抗大于 9999 kOhm，通常截断为 9999。*

### 4.4 状态数据包 (TYPE = 0x04: BP_TYPE_STATUS)
在收到 `GET_STATE` 指令后回复。
**PAYLOAD 长度**: 变长 (5 + 版本号长度 + 1 + SN号长度)。

| PAYLOAD 偏移 | 字段 | 类型 | 长度 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| 0 | `WM8978_OK` | uint8 | 1 | 音频芯片状态 (`1`=OK, `0`=Fail) |
| 1 | `QMI8658_OK` | uint8 | 1 | IMU 芯片状态 (`1`=OK, `0`=Fail) |
| 2 | `SD_OK` | uint8 | 1 | SD卡状态 (`1`=OK, `0`=Fail) |
| 3-4 | `Battery_mV` | uint16 | 2 | 电池电压，单位: 毫伏 (mV) |
| 5...K | `Firmware_Ver`| string | 变长 | 固件版本号字符串 (如 "T1.0.1") |
| K+1 | `Separator` | uint8 | 1 | 字符串分隔符，固定为 `0x00` (`\0`) |
| K+2...N | `Device_SN` | string | 变长 | 设备序列号字符串 (如 "AIT2601X04566")，无 \0 结尾 |

---

## 5. 校验算法参考 (CRC16-CCITT)

下位机与上位机均使用标准的 CRC16-CCITT (False) 算法对数据包进行完整性校验。

**算法参数**:
- 多项式 (Poly): `0x1021`
- 初始值 (Init): `0xFFFF`
- 结果异或值 (XorOut): `0x0000`
- 输入反转 (RefIn): `False`
- 输出反转 (RefOut): `False`

**Python 解析代码示例**:
```python
import struct

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc

# 帧解析示例
def parse_frame(buffer):
    if len(buffer) < 13: return None
    if buffer[0:2] != b'\x55\xAA': return None
    
    payload_len = struct.unpack('<H', buffer[2:4])[0]
    total_len = 4 + 7 + payload_len + 2 # 帧头(4) + 帧参数(7) + 载荷 + CRC(2)
    
    if len(buffer) < total_len: return None
    
    frame = buffer[:total_len]
    crc_recv = struct.unpack('<H', frame[-2:])[0]
    crc_calc = crc16_ccitt(frame[4:-2]) # 校验从 TYPE 开始到 PAYLOAD 结束
    
    if crc_recv == crc_calc:
        return frame # 校验成功
    return None
```