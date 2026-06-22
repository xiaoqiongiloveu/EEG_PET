/* commands_impl.c */
#include "commands_impl.h"
#include "binary_protocol.h" // New Protocol
#include "command_system.h"
#include "fatfs.h"
#include "ff.h"
#include <stdio.h>
#include "ADS1299.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "circular_buffer.h"
#include "bsp_driver_sd.h"
#include <string.h>
#include <math.h>
#include "wavplay.h"

// --- Motion Artifact State Machine Definitions ---
typedef enum {
    MOTION_STABLE   = 0,  // 安静/无伪迹
    MOTION_MICRO    = 1,  // 微动 (通常不影响EEG)
    MOTION_ACTIVE   = 2,  // 明显活动 (可能产生轻微基线漂移)
    MOTION_ARTIFACT = 3,  // 运动伪迹 (建议剔除该段EEG)
    MOTION_SEVERE   = 4   // 剧烈运动/磕碰 (数据不可用)
} motion_state_t;

typedef struct {
    motion_state_t current_state;
    uint8_t stable_count;
} motion_state_machine_t;

#define DEAD_BAND 5.0f

/**
 * @brief 更新并获取当前的运动状态等级
 * @param raw_score 原始的运动能量得分 (0-100)
 * @retval motion_state_t 计算出的最终运动状态枚举值
 * @note 内部实现了死区抑制、动态基线跟踪以及状态机的平滑切换
 */
static motion_state_t Motion_State_Update(float raw_score) 
{
    static motion_state_machine_t sm = {MOTION_STABLE, 0};
    static float baseline = 0.0f;
    float adjusted_score = 0.0f;

    // 1.1 死区抑制
    if (raw_score < DEAD_BAND) {
        raw_score = 0.0f;
    }

    // 1.2 动态基线跟踪
    baseline = 0.99f * baseline + 0.01f * raw_score;

    // 1.3 提取真实的动态突变
    adjusted_score = raw_score - baseline;
    if (adjusted_score < 0.0f) {
        adjusted_score = 0.0f;
    }

    // 2. 状态机层
    motion_state_t target_state = sm.current_state;

    // 2.1 阈值映射
    if (adjusted_score > 60.0f)      target_state = MOTION_SEVERE;
    else if (adjusted_score > 35.0f) target_state = MOTION_ARTIFACT;
    else if (adjusted_score > 15.0f) target_state = MOTION_ACTIVE;
    else if (adjusted_score > 5.0f)  target_state = MOTION_MICRO;
    else                             target_state = MOTION_STABLE;

    sm.current_state = target_state;

    return sm.current_state;
}
// ------------------------------------------------

extern FATFS SDFatFS;
extern FIL SDFile;
extern CircularBuffer ADS1299_circula_buffer;
extern uint8_t sd_mounted;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern ADC_HandleTypeDef hadc1; // Extern ADC handle

// 外部函数/变量来自 main.c
extern float Battery_Voltage;
extern void Update_Battery_Voltage(void);
extern uint8_t wm8978_initialized;

// 全局配置变量
uint8_t g_cfg_gain = 24;
uint8_t g_cfg_ch_mask = 0xFF;
int g_cfg_sps = 250;
uint8_t g_cfg_input_type = 0; // 0=正常, 1=短接, 5=测试信号
uint8_t g_cfg_bias_en = 1;
uint32_t g_cfg_duration_sec = 0; // 0 = 无限长
uint8_t g_cfg_test_sig = 0; // 0=外部, 1=内部

// OTA 全局变量
volatile uint8_t g_file_transfer_mode = 0;
static uint32_t ota_total_bytes = 0;
static uint32_t ota_received_bytes = 0;
static FIL ota_fil;
static uint32_t ota_start_tick = 0;
volatile uint8_t stream_bad_ch_history = 0; // 移至顶层作用域以提高可见性
volatile uint8_t g_ota_complete_flag = 0;   // 标志位，通知主循环完成 OTA

// 任务全局变量
volatile uint8_t g_task_state = 0; // 0=无, 1=等待中, 2=运行中, 3=已完成
volatile uint8_t g_write_sd_enabled = 0; // 0=在线模式(不写SD), 1=离线模式(写入SD)
uint32_t g_task_id = 0;
uint32_t g_task_start_tick = 0;
uint32_t g_task_delay_ms = 0;

// 二进制协议静态缓冲区 (防止栈溢出)
static uint8_t g_bp_tx_buf[256];
static uint8_t g_bp_payload[128];

// 定义二进制文件头和帧结构
#pragma pack(push, 1)
typedef struct {
    uint32_t magic_word;      // 0x45454721 ("EEG!")
    uint16_t version;         // 0x0100 (V1.0)
    uint16_t header_length;   // 256
    uint16_t sample_rate;     // e.g., 250
    uint8_t  num_channels;    // 8
    uint8_t  has_imu;         // 1
    uint8_t  has_motion;      // 1
    uint8_t  data_format;     // 0=INT32 (stored as int32_t to save space)
    uint16_t resolution_uv;   // 1000
    uint64_t start_timestamp; // 0
    char device_name[16];     // "EEG_Device"
    char fw_version[16];      // "V1.0"
    uint8_t reserved[200];    // Padding to 256
} FileHeader_t;

typedef struct {
    uint32_t frame_seq;
    int32_t  eeg_data[8];
    int16_t  acc[3];
    int16_t  gyro[3];
    uint8_t  motion_state;
    uint8_t  checksum;
} DataFrame_t;
#pragma pack(pop)

/**
 * @brief 简单的软件 CRC32 计算实现 (多项式 0x04C11DB7)
 * @param data 指向待计算数据的指针
 * @param len 数据的字节长度
 * @retval uint32_t 计算出的 CRC32 校验值
 */
uint32_t soft_crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint32_t)data[i] << 24);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/**
 * @brief 计算存储在 SD 卡中的文件的 CRC32 校验值
 * @param filename 待校验的文件名
 * @retval uint32_t 计算出的文件 CRC32 校验值，若文件打开失败则返回 0
 * @note 采用标准以太网 CRC32 算法，兼容 Python 的 zlib.crc32，不使用查表法以节省空间
 */
uint32_t calculate_file_crc32(const char* filename) {
    extern FIL SDFile;
    if (f_open(&SDFile, filename, FA_READ) != FR_OK) return 0;
    
    uint32_t crc = 0xFFFFFFFF;
    uint8_t buf[512];
    UINT br;
    
    // 使用标准的 CRC32 无表实现
    while (1) {
        f_read(&SDFile, buf, sizeof(buf), &br);
        if (br == 0) break;
        
        // 更新 CRC
        for (size_t i = 0; i < br; i++) {
            uint32_t byte = buf[i];
            crc = crc ^ byte;
            for (int j = 0; j < 8; j++) {
                uint32_t mask = -(int)(crc & 1);
                crc = (crc >> 1) ^ (0xEDB88320 & mask);
            }
        }
    }
    f_close(&SDFile);
    return ~crc;
}

// OTA 缓冲区系统 (将 USB 中断与 FatFS 文件系统解耦)
#define OTA_BUFFER_SIZE 4096 // 4KB 环形缓冲区

static uint8_t ota_rx_buffer[OTA_BUFFER_SIZE];
static volatile uint16_t ota_buf_head = 0;
static volatile uint16_t ota_buf_tail = 0;
static volatile uint8_t ota_buf_overflow = 0;

/**
 * @brief 处理文件传输接收 (由 USB 中断调用)
 * @param buf 指向接收到的数据缓冲区的指针
 * @param len 接收到的数据长度
 * @note 此函数在 USB 接收中断(CDC_Receive_FS)中被调用，将数据快速推入环形缓冲区
 */
void handle_file_transfer(uint8_t* buf, uint32_t len) {
    if (!g_file_transfer_mode) return;
    
    // 将数据复制到环形缓冲区
    for (uint32_t i = 0; i < len; i++) {
        uint16_t next_head = (ota_buf_head + 1) % OTA_BUFFER_SIZE;
        if (next_head != ota_buf_tail) {
            ota_rx_buffer[ota_buf_head] = buf[i];
            ota_buf_head = next_head;
        } else {
            ota_buf_overflow = 1; // 缓冲区已满！丢弃数据
        }
    }
}

/**
 * @brief 处理 OTA 接收到的数据并写入 SD 卡
 * @note 在主循环中被轮询调用，负责将环形缓冲区中的数据落盘，以防长时间阻塞 USB 中断
 */
void process_ota_data(void) {
    IWDG->KR = 0xAAAA; // 在写入 SD 卡期间喂狗
    if (!g_file_transfer_mode) return;
    
    // 检查缓冲区是否溢出
    if (ota_buf_overflow) {
        g_file_transfer_mode = 0;
        f_close(&ota_fil);
        USB_Print("\r\n[OTA] Error: Buffer Overflow. Main loop too slow.\r\n");
        return;
    }

    // 检查是否有足够的数据需要写入
    uint16_t head = ota_buf_head; // 获取快照
    uint16_t tail = ota_buf_tail;
    
    if (head == tail) return; // 缓冲区为空
    
    // 计算连续可用的数据长度
    uint16_t count;
    if (head >= tail) {
        count = head - tail;
    } else {
        count = OTA_BUFFER_SIZE - tail; // 先读到缓冲区末尾
    }
    
    UINT bw;
    FRESULT fr = f_write(&ota_fil, &ota_rx_buffer[tail], count, &bw);
    
    if (fr != FR_OK) {
        g_file_transfer_mode = 0;
        f_close(&ota_fil);
        USB_Print("\r\n[OTA] Error: Write failed (%d).\r\n", fr);
        return;
    }
    
    // 推进尾指针
    ota_buf_tail = (tail + count) % OTA_BUFFER_SIZE;
    
    ota_received_bytes += bw;
    
    // 检查传输是否完成
    if (ota_received_bytes >= ota_total_bytes) {
         g_file_transfer_mode = 0;
         g_ota_complete_flag = 1;
         
         // 注意：文件关闭和"传输完成"的消息会在 app_run_loop() 中处理，
         // 以防止对文件句柄进行二次关闭 (FatFS Error 9)。
    } else {
        // 每接收约 64KB 发送一次文本进度，避免向主机发送过多消息
        if (ota_received_bytes % 65536 == 0) {
            USB_Print("[PROG] %lu / %lu\r\n", ota_received_bytes, ota_total_bytes);
        }
    }
}

/**
 * @brief 启动 OTA 升级传输
 * @param args 包含 OTA 文件总大小的参数字符串 (字节数)
 * @retval cmd_response_t 指令执行结果
 * @note 准备接收固件文件，并在 SD 卡中创建 update.bin 准备落盘
 */
cmd_response_t cmd_ota_start_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }
    
    if (args == NULL || args[0] == '\0') {
        response.success = 0;
        strcpy(response.message, "ERR: Usage OTA_START:<size_in_bytes>");
        return response;
    }
    
    // 解析文件大小
    uint32_t size = strtoul(args, NULL, 0);
    if (size == 0) {
        response.success = 0;
        strcpy(response.message, "ERR: Invalid size");
        return response;
    }
    
    // 打开/创建更新文件
    FRESULT fr = f_open(&ota_fil, "update.bin", FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        response.success = 0;
        sprintf(response.message, "ERR: Failed to create update.bin (%d)", fr);
        return response;
    }
    
    ota_total_bytes = size;
    ota_received_bytes = 0;
    ota_start_tick = HAL_GetTick();
    g_file_transfer_mode = 1;
    
    // 重置环形缓冲区
    ota_buf_head = 0;
    ota_buf_tail = 0;
    ota_buf_overflow = 0;
    
    USB_Print("[OTA] Ready to receive %lu bytes. Send binary data now...\r\n", size);
    
    strcpy(response.message, "OK: OTA Transfer Started");
    return response;
}

/**
 * @brief 应用 OTA 升级固件
 * @param args 无需参数
 * @retval cmd_response_t 指令执行结果
 * @note 检查 update.bin 的有效性并重启系统，将控制权交由 Bootloader 进行固件替换
 */
cmd_response_t cmd_ota_apply_handler(char* args) {
    cmd_response_t response = {1, ""};
    (void)args;
    
    FILINFO fno_info;
    if (f_stat("update.bin", &fno_info) != FR_OK) {
        response.success = 0;
        strcpy(response.message, "ERR: update.bin not found");
        return response;
    }
    
    if (fno_info.fsize == 0) {
        response.success = 0;
        strcpy(response.message, "ERR: update.bin is empty");
        return response;
    }
    
    USB_Print("[OTA] update.bin found (%lu bytes). Shutting down for update...\r\n", fno_info.fsize);
    HAL_Delay(500); // 等待消息发送完毕
    
    // 播放关机音效以指示 OTA 准备就绪 (使用 DEVICE_SHUTDOWN ID: 14)
    Play_System_Audio(14);
    
    // 执行关机/复位序列 (兼容旧项目行为)
    // 拉低 POWER_CTRL (PA4) 以切断电源
    HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_RESET);
    
    // 等待断电或重启
    extern volatile uint8_t g_usb_host_connected;
    if (g_usb_host_connected == 1) {
         NVIC_SystemReset();
    } else {
         while(1);
    }
    
    // 正常情况下不会执行到这里
    strcpy(response.message, "OK: Shutting down...");
    return response;
}

// 局部文件系统对象
static DIR dir;
static FILINFO fno;

volatile AppState app_state = APP_IDLE;
// static FIL data_file; 
static uint32_t start_time = 0;
static uint32_t sample_count = 0;
static uint32_t target_samples = 0;
static uint32_t last_save_time = 0;
static uint32_t last_display_time = 0;
static int current_sps = 250;
static int current_channel = 1;
static uint32_t stream_seq_num = 0;

// 阻抗计算变量
static float imp_max_val = -1e9;
static float imp_min_val = 1e9;
static int imp_window_count = 0;
static const int IMP_WINDOW_SIZE = 50; // 0.2s @ 250Hz (约 1.5 个 7.8Hz 周期)

// 噪声测试变量
static double noise_sum[8];
static double noise_sum_sq[8];
static float noise_min[8];
static float noise_max[8];

// CMRR 测试变量
static double cmrr_sum_sq[8];
static const float CMRR_VIN_RMS = 0.7071f; // 假设 2Vpp 输入 = 0.7071 Vrms

// CMRR 完整测试变量
static double cmrr_step1_sum_sq[8]; // 用于差模增益
static double cmrr_step2_sum_sq[8]; // 用于共模增益
static double cmrr_diff_gain[8];    // 计算出的差模增益

// 阻抗测试统计
static double imp_sum = 0.0;
static uint32_t imp_count = 0;

static double rld_sum[8];
static double rld_sum_sq[8];

// 全通道阻抗测试变量
static float imp_all_max[8];
static float imp_all_min[8];

// --- STREAM 全局配置 ---
// static uint8_t g_cfg_ch_mask = 0xFF; // 已移至全局
// static uint8_t g_cfg_gain = 24;      // 已移至全局
// static int     g_cfg_sps = 250;      // 已移至全局
// static uint8_t g_cfg_input_type = 0; // 已移至全局
// static uint8_t g_cfg_bias_en = 1;    // 已移至全局
static uint8_t current_bias_sensp_mask = 0; // AUTO_STREAM 运行时 BIAS 掩码

/**
 * @brief 将增益倍数转换为 ADS1299 寄存器对应的配置位
 * @param gain 设定的增益倍数 (1, 2, 4, 6, 8, 12, 24)
 * @retval uint8_t 对应寄存器的位掩码 (默认返回 24 倍增益)
 */
static uint8_t get_gain_code(uint8_t gain) {
    switch(gain) {
        case 1:  return 0x00;
        case 2:  return 0x10;
        case 4:  return 0x20;
        case 6:  return 0x30;
        case 8:  return 0x40;
        case 12: return 0x50; 
        case 24: return 0x60; 
        default: return 0x60; // 默认使用 24 倍增益
    }
}

/**
 * @brief 检查文件名是否以 .txt 结尾
 * @param filename 待检查的文件名
 * @retval int 如果是 .txt 文件返回 1，否则返回 0
 */
static int is_txt_file(const char* filename) {
  if (filename == NULL) return 0;
  int len = strlen(filename);
  if (len < 4) return 0;
  const char* ext = filename + len - 4;
  if ((ext[0] == '.' || ext[0] == '.') &&
      (ext[1] == 't' || ext[1] == 'T') &&
      (ext[2] == 'x' || ext[2] == 'X') &&
      (ext[3] == 't' || ext[3] == 'T')) {
    return 1;
  }
  return 0;
}

/**
 * @brief 打印文件处理或传输的进度信息
 * @param current 当前已处理的字节数
 * @param total 文件的总字节数
 */
static void display_file_progress(uint32_t current, uint32_t total) {
  if (total > 0) {
    float percent = (current * 100.0f) / total;
    USB_Print("[PROG] %lu/%lu bytes (%.1f%%)\r\n",
           (unsigned long)current,
           (unsigned long)total,
           percent);
  }
}

/**
 * @brief 阻塞等待 USB 传输通道变为空闲状态
 * @param timeout_ms 最大超时时间 (毫秒)
 * @retval int 如果在超时前变为空闲返回 1，如果超时返回 0
 */
static int usb_wait_tx_idle(uint32_t timeout_ms) {
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc == NULL) return 0;
  uint32_t start = HAL_GetTick();
  while (hcdc->TxState != 0) {
    if (HAL_GetTick() - start > timeout_ms) return 0;
  }
  return 1;
}

/**
 * @brief 检查并打印 SD 卡文件系统的容量与使用情况
 * @note 内部调用 FatFs API 获取总容量与剩余空间并打印到串口
 */
void check_filesystem_impl(void) {
  DWORD free_clust, free_size, total_size;
  FATFS *fs_ptr;

  USB_SendString("[FS] Checking filesystem...\r\n");

  FRESULT fr = f_getfree("", &free_clust, &fs_ptr);
  if (fr != FR_OK) {
    USB_Print("[FS] Error: Failed to get free space (%d)\r\n", fr);
    return;
  }

  if (fs_ptr == NULL) {
    USB_SendString("[FS] Error: Filesystem pointer is NULL\r\n");
    return;
  }

  total_size = (fs_ptr->n_fatent - 2) * (fs_ptr->csize / 2);
  free_size = free_clust * (fs_ptr->csize / 2);

  USB_SendString("[FS] Filesystem information:\r\n");
  USB_Print("  Total clusters: %lu\r\n", (unsigned long)(fs_ptr->n_fatent - 2));
  USB_Print("  Free clusters: %lu\r\n", (unsigned long)free_clust);
  USB_Print("  Cluster size: %u\r\n", fs_ptr->csize);
  USB_Print("  Sector size: %u bytes\r\n", 512);
  USB_Print("  Cluster size: %u bytes\r\n", fs_ptr->csize * 512);
  USB_Print("  Total size: %lu KB (approx %.2f MB)\r\n",
         (unsigned long)total_size, total_size / 1024.0);
  USB_Print("  Free space: %lu KB (approx %.2f MB)\r\n",
         (unsigned long)free_size, free_size / 1024.0);

  if (total_size > 0) {
    float usage_percent = 100.0f - (free_size * 100.0f / total_size);
    USB_Print("  Usage: %.1f%%\r\n", usage_percent);
    
    // 更新全局变量缓存
    extern uint8_t g_sd_usage_percent;
    uint8_t int_usage = (uint8_t)usage_percent;
    g_sd_usage_percent = (int_usage == 0) ? 1 : int_usage; // 避免显示0%，最低显示1%
  }
}

/**
 * @brief 处理 LIST 指令，列出 SD 卡根目录下的所有文本文件
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 */
cmd_response_t cmd_list_handler(char* args) {
  cmd_response_t response = {1, ""};
  FRESULT fr;
  (void)args;

  fr = f_opendir(&dir, "/");
  if (fr != FR_OK) {
    response.success = 0;
    sprintf(response.message, "ERR:Failed to open directory (%d)", fr);
    return response;
  }

  int file_count = 0;
  USB_Print("OK:");

  while ((fr = f_readdir(&dir, &fno)) == FR_OK && fno.fname[0] != 0) {
    if (is_txt_file(fno.fname)) {
      if (file_count > 0) {
        USB_SendString(",");
      }
      USB_Print("%s", fno.fname);
      file_count++;
    }
  }
  f_closedir(&dir);
  USB_SendString("\r\n");
  USB_Print("[LIST] Found %d .txt files\r\n", file_count);
  sprintf(response.message, "[LIST] Listed %d files", file_count);
  return response;
}

/**
 * @brief 处理 GET 指令，从 SD 卡读取并传输指定文件到上位机
 * @param args 包含要读取的目标文件名的参数字符串
 * @retval cmd_response_t 包含执行结果的响应结构体
 */
cmd_response_t cmd_get_handler(char* args) {
  cmd_response_t response = {1, ""};
  
  if (app_state != APP_IDLE) {
      response.success = 0;
      strcpy(response.message, "ERR: Device not idle. Send STOP first.");
      return response;
  }
  
  FRESULT fr;
  UINT br;
  static uint8_t buffer[2048];
  uint32_t file_size, total_sent = 0;
  uint32_t start_ms = 0;
  uint32_t next_report = 1024UL * 1024UL;

  if (args == NULL || args[0] == '\0') {
    response.success = 0;
    strcpy(response.message, "ERR:Missing filename");
    return response;
  }

  while (*args == ' ') args++;
  USB_Print("[CMD] Sending file: %s\r\n", args);

  fr = f_open(&SDFile, args, FA_READ);
  if (fr != FR_OK) {
    response.success = 0;
    sprintf(response.message, "ERR:File not found '%s' (%d)", args, fr);
    return response;
  }

  file_size = f_size(&SDFile);
  char header[32];
  int len = sprintf(header, "OK:%lu\r\n", (unsigned long)file_size);
  (void)len;
  USB_SendString(header);

  USB_Print("[FILE] Starting transmission: %s, size: %lu bytes\r\n",
         args, (unsigned long)file_size);

  start_ms = HAL_GetTick();
  while (1) {
    if (!usb_wait_tx_idle(1000)) {
      f_close(&SDFile);
      response.success = 0;
      strcpy(response.message, "ERR:USB TX timeout");
      return response;
    }

    fr = f_read(&SDFile, buffer, sizeof(buffer), &br);
    if (fr != FR_OK) break;
    if (br == 0) break;

    if (CDC_Transmit_FS(buffer, (uint16_t)br) != USBD_OK) {
      uint32_t retry_start = HAL_GetTick();
      while (CDC_Transmit_FS(buffer, (uint16_t)br) == USBD_BUSY) {
        if (!usb_wait_tx_idle(1000) || (HAL_GetTick() - retry_start > 2000)) {
          f_close(&SDFile);
          response.success = 0;
          strcpy(response.message, "ERR:USB busy timeout");
          return response;
        }
      }
    }

    total_sent += br;
    if (total_sent >= next_report || total_sent == file_size) {
      uint32_t elapsed_ms = HAL_GetTick() - start_ms;
      float speed_kib_s = 0.0f;
      if (elapsed_ms > 0) {
        speed_kib_s = ((float)total_sent * 1000.0f) / ((float)elapsed_ms * 1024.0f);
      }
      USB_Print("[PROG] %lu/%lu bytes (%.1f%%) %.1f KiB/s\r\n",
                (unsigned long)total_sent,
                (unsigned long)file_size,
                file_size > 0 ? (total_sent * 100.0f) / file_size : 0.0f,
                speed_kib_s);
      while (next_report <= total_sent && next_report < file_size) next_report += (1024UL * 1024UL);
    }
  }

  usb_wait_tx_idle(2000);

  USB_SendString("EOF:\r\n");
  f_close(&SDFile);
  USB_Print("[FILE] Transmission completed: %s\r\n", args);
  if (fr != FR_OK) {
    response.success = 0;
    sprintf(response.message, "ERR: Read failed (%d)", fr);
    return response;
  }
  uint32_t total_ms = HAL_GetTick() - start_ms;
  float avg_kib_s = 0.0f;
  if (total_ms > 0) {
    avg_kib_s = ((float)total_sent * 1000.0f) / ((float)total_ms * 1024.0f);
  }
  sprintf(response.message, "[GET] Transmitted: %s (%lu bytes) avg %.1f KiB/s",
          args, (unsigned long)total_sent, avg_kib_s);
  return response;
}

/**
 * @brief 处理 DIAG 指令，进行 SD 卡及文件系统的深度诊断
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 打印物理连接状态、挂载状态、容量信息及所有文件详情
 */
cmd_response_t cmd_diag_handler(char* args) {
  cmd_response_t response = {1, ""};
  (void)args;
  USB_SendString("\r\n=== SD Card Diagnosis ===\r\n");

  // 1. 检查 SD 卡物理是否存在
  uint8_t is_detected = BSP_SD_IsDetected();
  USB_Print("SD Card Physical Status: %s\r\n", is_detected == SD_PRESENT ? "DETECTED" : "NOT DETECTED");

  // 2. 检查文件系统挂载标志
  USB_Print("File System Mount Flag: %s\r\n", sd_mounted ? "MOUNTED" : "NOT MOUNTED");

  if (is_detected != SD_PRESENT) {
      USB_SendString("Error: No SD card detected in slot.\r\n");
      strcpy(response.message, "[DIAG] Failed: No SD card");
      return response;
  }

  // 3. 如果未挂载，尝试重新挂载
  if (!sd_mounted) {
      USB_SendString("Warning: FS not mounted. Attempting to mount...\r\n");
      FRESULT res = f_mount(&SDFatFS, SDPath, 1);
      if(res == FR_OK) {
          sd_mounted = 1;
          USB_SendString("Mount Success!\r\n");
      } else {
          USB_Print("Mount Failed! Error: %d\r\n", res);
          sprintf(response.message, "[DIAG] Mount Failed: %d", res);
          return response;
      }
  }

  check_filesystem_impl();
  USB_SendString("\r\n[DIAG] Listing all files:\r\n");

  FRESULT fr = f_opendir(&dir, "/");
  if (fr == FR_OK) {
    int count = 0;
    while ((fr = f_readdir(&dir, &fno)) == FR_OK && fno.fname[0] != 0) {
      count++;
      USB_Print("  %3d. %-20s Size: %10lu Attr:",
             count, fno.fname, (unsigned long)fno.fsize);
      if (fno.fattrib & AM_DIR) USB_SendString(" DIR");
      if (fno.fattrib & AM_RDO) USB_SendString(" RDO");
      if (fno.fattrib & AM_HID) USB_SendString(" HID");
      if (fno.fattrib & AM_SYS) USB_SendString(" SYS");
      if (fno.fattrib & AM_ARC) USB_SendString(" ARC");
      USB_SendString("\r\n");
    }
    f_closedir(&dir);
    if (count == 0) {
      USB_SendString("  [INFO] Directory is empty\r\n");
    }
  }
  USB_SendString("=== Diagnosis Completed ===\r\n");
  strcpy(response.message, "[DIAG] Diagnosis completed");
  return response;
}

/**
 * @brief 处理 HELP 指令，打印所有已注册指令的帮助信息
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 */
cmd_response_t cmd_help_handler(char* args) {
  cmd_response_t response = {1, ""};
  print_all_commands();
  strcpy(response.message, "[HELP] Command information displayed");
  return response;
}

/**
 * @brief 处理 DELETE 指令，从 SD 卡中删除指定文件
 * @param args 包含要删除的目标文件名的参数字符串
 * @retval cmd_response_t 包含执行结果的响应结构体
 */
cmd_response_t cmd_delete_handler(char* args) {
  cmd_response_t response = {1, ""};
  
  if (app_state != APP_IDLE) {
      response.success = 0;
      strcpy(response.message, "ERR: Device not idle. Send STOP first.");
      return response;
  }
  
  if (args == NULL || args[0] == '\0') {
      response.success = 0;
      strcpy(response.message, "ERR: Missing filename");
      return response;
  }
  while (*args == ' ') args++;
  FRESULT fr = f_unlink(args);
  if (fr == FR_OK) {
      sprintf(response.message, "OK: Deleted '%s'", args);
  } else {
      response.success = 0;
      sprintf(response.message, "ERR: Delete failed (%d)", fr);
  }
  return response;
}

/**
 * @brief 处理 INFO 指令，获取并打印指定文件的详细信息 (大小、日期、时间)
 * @param args 包含要查询的目标文件名的参数字符串
 * @retval cmd_response_t 包含执行结果的响应结构体
 */
cmd_response_t cmd_info_handler(char* args) {
  cmd_response_t response = {1, ""};
  if (args == NULL || args[0] == '\0') {
      response.success = 0;
      strcpy(response.message, "ERR: Missing filename");
      return response;
  }
  while (*args == ' ') args++;
  FILINFO fno_info;
  FRESULT fr = f_stat(args, &fno_info);
  if (fr == FR_OK) {
      USB_Print("File: %s\r\nSize: %lu bytes\r\nDate: %u-%02u-%02u\r\nTime: %02u:%02u\r\n", 
          fno_info.fname, (unsigned long)fno_info.fsize, 
          (fno_info.fdate >> 9) + 1980, (fno_info.fdate >> 5) & 15, fno_info.fdate & 31,
          (fno_info.ftime >> 11), (fno_info.ftime >> 5) & 63);
      strcpy(response.message, "OK: Info displayed");
  } else {
      response.success = 0;
      sprintf(response.message, "ERR: Stat failed (%d)", fr);
  }
  return response;
}

/**
 * @brief 处理 CREATELARGE 指令，生成用于测试 SD 卡写入速度的 10MB 伪随机大文件
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 */
cmd_response_t cmd_createlarge_handler(char* args) {
  cmd_response_t response = {1, ""};
  (void)args;
  const UINT BUF_SIZE = 16384; // 16KB 缓冲区以加速写入
  static uint8_t buffer[16384] __attribute__((aligned(4)));
  FRESULT fr;
  UINT bw;
  const uint32_t total_size = 10UL * 1024UL * 1024UL;
  uint32_t written = 0;
  uint32_t next_report = total_size / 10;
  uint32_t report_idx = 1;
  uint32_t pat_state = 0;
  uint32_t start_ms = 0;

  // 预填充缓冲区，以节省在写入循环中的 CPU 时间
  for (UINT i = 0; i < BUF_SIZE; i++) {
    if (pat_state < 26) {
      buffer[i] = (uint8_t)('A' + pat_state);
      pat_state++;
    } else if (pat_state == 26) {
      buffer[i] = '\r';
      pat_state++;
    } else {
      buffer[i] = '\n';
      pat_state = 0;
    }
  }

  USB_SendString("[TEST] Creating 10MB file 'large.txt'...\r\n");
  fr = f_open(&SDFile, "large.txt", FA_CREATE_ALWAYS | FA_WRITE);
  if (fr != FR_OK) {
      response.success = 0;
      sprintf(response.message, "ERR: Open failed (%d)", fr);
      return response;
  }

  start_ms = HAL_GetTick();
  while (written < total_size) {
      uint32_t remaining = total_size - written;
      UINT to_write = (remaining >= BUF_SIZE) ? BUF_SIZE : (UINT)remaining;

      fr = f_write(&SDFile, buffer, to_write, &bw);
      if (fr != FR_OK || bw < to_write) {
          f_close(&SDFile);
          response.success = 0;
          sprintf(response.message, "ERR: Write failed at %lu (%d)", written, fr);
          return response;
      }
      written += bw;
      if (report_idx <= 10 && written >= next_report * report_idx) {
        USB_Print("[PROG] %lu%% (%lu/%lu)\r\n",
                  (unsigned long)(report_idx * 10),
                  (unsigned long)written,
                  (unsigned long)total_size);
        report_idx++;
      }
  }
  f_sync(&SDFile);
  f_close(&SDFile);
  uint32_t elapsed_ms = HAL_GetTick() - start_ms;
  USB_Print("[TEST] Done. Total time: %lu ms (%.2f s)\r\n",
            (unsigned long)elapsed_ms,
            (double)elapsed_ms / 1000.0);
  sprintf(response.message, "OK: Created large.txt (%lu bytes) in %lu ms",
          (unsigned long)written,
          (unsigned long)elapsed_ms);
  return response;
}

/* === ADS1299 相关指令处理实现 === */

/**
 * @brief 处理 STREAM 指令，以文本模式启动脑电数据采集
 * @param args 包含目标文件名或采集参数的字符串
 * @retval cmd_response_t 包含执行结果的响应结构体
 */
cmd_response_t cmd_stream_handler(char* args) {
    cmd_response_t response = {1, ""};
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    // 解析目标文件名
    char filename[64];
    if (args != NULL && args[0] != '\0') {
        // 过滤掉前导空格和冒号
        while(*args == ' ' || *args == ':') args++;
        
        // 使用用户指定的文件名，限制长度
        if (strlen(args) > 50) { 
             args[50] = '\0';
        }
        
        // 检查是否自带后缀名
        if (strstr(args, ".") == NULL) {
            snprintf(filename, sizeof(filename), "%s.BIN", args);
        } else {
            snprintf(filename, sizeof(filename), "%s", args);
        }
        g_write_sd_enabled = 1; // 离线任务模式：启用 SD 卡写入
    } else {
        strcpy(filename, "EEG_DATA.BIN");
        g_write_sd_enabled = 0; // 在线实时模式：禁用 SD 卡写入
    }

    // 使用全局配置参数
    current_sps = g_cfg_sps;
    
    // 计算目标采样总数
    if (g_cfg_duration_sec > 0) {
        target_samples = g_cfg_duration_sec * g_cfg_sps;
    } else {
        target_samples = 0xFFFFFFFF; // 无限采集模式 (250SPS 下约 200 天)
    }

    // 配置 ADS1299 硬件寄存器
    // 注意：STREAM 指令不重置全局配置，如果需要修改，应通过 CONF 指令
    ADS1299_Stop();
    
    // 1. 设置采样率 (SPS)
    uint8_t sps_code = 0x06; // 默认为 250 SPS
    if (g_cfg_sps == 250) sps_code = 0x06;
    else if (g_cfg_sps == 500) sps_code = 0x05;
    else if (g_cfg_sps == 1000) sps_code = 0x04;
    else if (g_cfg_sps == 2000) sps_code = 0x03;
    else if (g_cfg_sps == 4000) sps_code = 0x02;
    else if (g_cfg_sps == 8000) sps_code = 0x01;
    else if (g_cfg_sps == 16000) sps_code = 0x00;

    ADS1299_Set_Sample_Parameter(sps_code, 6, 0); 
    
    // 1.5. 如果需要，配置 CONFIG2 开启测试信号发生器
    if (g_cfg_test_sig == 1 || g_cfg_input_type == 5) {
        // 0xD0: 开启内部测试信号发生器
        ADS1299_Write_Reg(0x02, 0xD0); 
    } else {
        // 0xC0: 外部正常输入
        ADS1299_Write_Reg(0x02, 0xC0); 
    }

    // 2. 配置通道的增益和输入模式
    uint8_t gain_bits = get_gain_code(g_cfg_gain);
    
    for(int i=1; i<=8; i++) { 
       if (g_cfg_ch_mask & (1 << (i-1))) {
           ADS1299_Write_Reg(0x04 + i, gain_bits | (g_cfg_input_type & 0x07));
       } else {
           ADS1299_Write_Reg(0x04 + i, 0x80 | 0x01); // 通道关断并短接输入
       }
    } 

    // 3. 配置右腿驱动偏置网络 (RLD BIAS)
    // 根据需求改回全通道引入 BIAS
    // 切勿动态切换偏置通道，以防止模拟阶跃响应和振荡
    if (g_cfg_bias_en) {
        // 开启 BIAS 放大器 (0xEC = 1110 1100)
        ADS1299_Write_Reg(0x03, 0xEC); 
        ADS1299_Write_Reg(0x0D, 0xFF); // 启用全通道(CH1-CH8) 的 BIAS_SENSP
        current_bias_sensp_mask = 0xFF; // 初始化运行时掩码记录
    } else {
        // 关闭 BIAS 放大器 (0xE8 = 1110 1000)
        ADS1299_Write_Reg(0x03, 0xE8);
        ADS1299_Write_Reg(0x0D, 0x00);
        current_bias_sensp_mask = 0x00;
    } 
    
    // 4. 配置导联脱落检测 (仅启用正极 P-pole)
    // 开启比较器以便在数据流模式下实时获取脱落状态
    ADS1299_Write_Reg(0x17, 0x02); // 开启 PD_LOFF_COMP

    // LOFF (0x04): 直流脱落检测, 6nA (Bits: 000 0 00 00) => 0x00
    ADS1299_Write_Reg(0x04, 0x00); 

    // LOFF_SENSP (0x0F): 为活动的通道启用正极脱落检测
    ADS1299_Write_Reg(0x0F, g_cfg_ch_mask); 

    // LOFF_SENSN (0x10): 禁用负极脱落检测 (0x00)
    ADS1299_Write_Reg(0x10, 0x00);
    
    // LOFF_FLIP (0x11): 禁用电流方向翻转
    ADS1299_Write_Reg(0x11, 0x00);
   
    USB_Print("\r\n[BOOT] ADS1299 Registers before START:\r\n"); 
    uint8_t regs[24]; 
    ADS1299_Read_Regs(0x00, regs, 24); 
    for(int i=0; i<24; i++) { 
       USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]); 
       HAL_Delay(5); 
    } 
    
    // 在开始采集前播放 7.wav (COLLECT_START)
    // 由于现在处于主循环上下文中 (延迟执行)，
    // 调用 Play_System_Audio 是安全的。
    Play_System_Audio(7);
    
    if (g_write_sd_enabled) {
        // 打开用于记录的文件 (边流式传输边保存)
        FRESULT fr = f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
        if (fr != FR_OK) {
            response.success = 0;
            sprintf(response.message, "ERR: Failed to open %s (%d)", filename, fr);
            return response;
        }
        
        // 写入二进制文件头
        FileHeader_t header = {0};
        header.magic_word = 0x45454721; // "EEG!"
        header.version = 0x0100;
        header.header_length = sizeof(FileHeader_t); // Should be 256
        header.sample_rate = current_sps;
        header.num_channels = 8;
        header.has_imu = 1;
        header.has_motion = 1;
        header.data_format = 0; // INT32
        header.resolution_uv = 1000;
        strcpy(header.device_name, "EEG_Device");
        strcpy(header.fw_version, "V1.0");
        
        UINT bw;
        f_write(&SDFile, &header, sizeof(FileHeader_t), &bw);
    }

    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    // 开始 stream 运行，蓝色1s闪烁 (慢闪)
    RGB_LED_SetSingle(LED_COLOR_BLUE, LED_MODE_FLASH_SLOW);

    app_state = APP_STREAM_RUNNING;
    start_time = HAL_GetTick();
    stream_seq_num = 0;
    sample_count = 0;
    last_save_time = start_time;
    last_display_time = start_time;
    
    // 清除坏道历史记录
    // extern uint8_t stream_bad_ch_history; // 移除了重复声明
    stream_bad_ch_history = 0;

    sprintf(response.message, "OK: STREAM started. File:%s (Mask=0x%02X, Gain=%d, SPS=%d)", 
            filename, g_cfg_ch_mask, g_cfg_gain, g_cfg_sps);
    return response;
}

/**
 * @brief 处理 AUTO_STREAM 指令，启动带阻抗与脱落检测的自动脑电采集任务
 * @param args 包含目标文件名的参数字符串
 * @retval cmd_response_t 包含执行结果的响应结构体
 */
cmd_response_t cmd_auto_stream_handler(char* args) {
    cmd_response_t response = {1, ""};
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    // 解析目标文件名
    char filename[64];
    if (args != NULL && args[0] != '\0') {
        while(*args == ' ' || *args == ':') args++;
        if (strlen(args) > 50) args[50] = '\0';
        if (strstr(args, ".") == NULL) {
            snprintf(filename, sizeof(filename), "%s.TXT", args);
        } else {
            snprintf(filename, sizeof(filename), "%s", args);
        }
    } else {
        strcpy(filename, "EEG_DATA.TXT");
    }

    int sps = 250;
    current_sps = sps;
    target_samples = 60 * 60 * sps; // 最长允许连续采集 1 小时

    // 配置 ADS1299
    ADS1299_Stop();
    
    // 1. 设置采样率 (使用全局配置)
    uint8_t sps_code = 0x06; // 默认为 250 SPS
    if (g_cfg_sps == 250) sps_code = 0x06;
    else if (g_cfg_sps == 500) sps_code = 0x05;
    else if (g_cfg_sps == 1000) sps_code = 0x04;
    else if (g_cfg_sps == 2000) sps_code = 0x03;
    else if (g_cfg_sps == 4000) sps_code = 0x02;
    else if (g_cfg_sps == 8000) sps_code = 0x01;
    else if (g_cfg_sps == 16000) sps_code = 0x00;

    ADS1299_Set_Sample_Parameter(sps_code, 6, 0); 
    
    // 1.5. 配置 CONFIG2 (0x02)
    ADS1299_Write_Reg(0x02, 0xC0); // 恢复为外部正常输入，清除任何内部测试信号

    // 2. 配置通道的增益和输入模式
    uint8_t gain_bits = get_gain_code(g_cfg_gain);
    for(int i=1; i<=8; i++) { 
       if (g_cfg_ch_mask & (1 << (i-1))) {
           ADS1299_Write_Reg(0x04 + i, gain_bits | (g_cfg_input_type & 0x07)); 
       } else {
           ADS1299_Write_Reg(0x04 + i, 0x80 | 0x01); // 关断并短接未启用的通道
       }
    } 

    // 3. 配置右腿驱动偏置网络 (RLD BIAS) - 初始设置
    // 根据需求改回全通道引入 BIAS
    if (g_cfg_bias_en) {
        ADS1299_Write_Reg(0x03, 0xEC); 
        ADS1299_Write_Reg(0x0D, 0xFF); // 启用全通道(CH1-CH8) 的 BIAS_SENSP
        current_bias_sensp_mask = 0xFF; // 初始化运行时掩码记录
    } else {
        ADS1299_Write_Reg(0x03, 0xE8);
        ADS1299_Write_Reg(0x0D, 0x00);
        current_bias_sensp_mask = 0x00;
    } 

    // 4. 配置导联脱落检测 (仅启用正极 P-pole)
    // CONFIG4 (0x17): 开启导联脱落硬件比较器 (Bit 1)
    ADS1299_Write_Reg(0x17, 0x02); // PD_LOFF_COMP = 1

    // LOFF (0x04): 直流脱落检测, 注入电流 6nA (Bits: 000 0 00 00) => 0x00
    ADS1299_Write_Reg(0x04, 0x00); 

    // LOFF_SENSP (0x0F): 为当前活动的所有正极输入开启脱落检测
    ADS1299_Write_Reg(0x0F, g_cfg_ch_mask); 

    // LOFF_SENSN (0x10): 禁用负极的脱落检测 (0x00)
    ADS1299_Write_Reg(0x10, 0x00);
    
    // LOFF_FLIP (0x11): 禁用电流方向翻转
    ADS1299_Write_Reg(0x11, 0x00);

    // 播放提示音 7.wav (开始采集)
    Play_System_Audio(7);
    
    // 打开/创建目标文件
    FRESULT fr = f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        response.success = 0;
        sprintf(response.message, "ERR: Failed to open %s (%d)", filename, fr);
        return response;
    }
    
    g_write_sd_enabled = 1; // 启用 SD 卡写入

    // 写入二进制文件头信息
    FileHeader_t header = {0};
    header.magic_word = 0x45454721; // "EEG!"
    header.version = 0x0100;
    header.header_length = sizeof(FileHeader_t);
    header.sample_rate = g_cfg_sps;
    header.num_channels = 8;
    header.has_imu = 1;
    header.has_motion = 1;
    header.data_format = 0; // INT32
    header.resolution_uv = 1000;
    strcpy(header.device_name, "EEG_Device");
    strcpy(header.fw_version, "V1.0");
    
    UINT bw;
    f_write(&SDFile, &header, sizeof(FileHeader_t), &bw);

    // 重置历史脱落记录
    stream_bad_ch_history = 0;

    // 打印寄存器状态以供调试验证
    USB_Print("\r\n[BOOT] ADS1299 Registers before START (AUTO_STREAM):\r\n"); 
    uint8_t regs[24]; 
    ADS1299_Read_Regs(0x00, regs, 24); 
    for(int i=0; i<24; i++) { 
       USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]); 
       HAL_Delay(5); 
    } 

    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    // 开始 auto stream 运行，蓝色1s闪烁 (慢闪)
    RGB_LED_SetSingle(LED_COLOR_BLUE, LED_MODE_FLASH_SLOW);

    app_state = APP_AUTO_STREAM_RUNNING;
    start_time = HAL_GetTick();
    stream_seq_num = 0;
    sample_count = 0;
    last_save_time = start_time;
    last_display_time = start_time;

    sprintf(response.message, "OK: AUTO_STREAM started. File:%s", filename);
    return response;
}

/**
 * @brief 处理 CONF 指令，动态配置脑电采集的全局参数
 * @param args 包含配置参数的字符串 (格式: MASK,GAIN,SPS,INPUT_TYPE,BIAS_EN)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 仅在空闲状态下修改全局配置，实际生效需要等到下一次 START 指令
 */
cmd_response_t cmd_conf_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    // 如果解析失败，使用默认值
    unsigned int mask = 0xFF;
    int gain = 24;
    int sps = 250;
    int input_type = 0;
    int bias_en = 1;
    
    // 解析参数: CONF:MASK(16进制),GAIN,SPS,INPUT_TYPE,BIAS_EN
    // 示例: CONF:FF,24,250,0,1
    if (args == NULL || args[0] == '\0') {
         sprintf(response.message, "Current Config: Mask=0x%02X, Gain=%d, SPS=%d, Input=%d, Bias=%d", 
                 g_cfg_ch_mask, g_cfg_gain, g_cfg_sps, g_cfg_input_type, g_cfg_bias_en);
         return response;
    }

    int n = sscanf(args, "%x,%d,%d,%d,%d", &mask, &gain, &sps, &input_type, &bias_en);
    
    if (n >= 1) {
        g_cfg_ch_mask = (uint8_t)mask;
        if (n >= 2) g_cfg_gain = (uint8_t)gain;
        if (n >= 3) g_cfg_sps = sps;
        if (n >= 4) g_cfg_input_type = (uint8_t)input_type;
        if (n >= 5) g_cfg_bias_en = (uint8_t)bias_en;
        
        sprintf(response.message, "OK: Config Set (Mask=0x%02X, Gain=%d, SPS=%d, Input=%d, Bias=%d)",
                g_cfg_ch_mask, g_cfg_gain, g_cfg_sps, g_cfg_input_type, g_cfg_bias_en);
        
        // 方案配置下发成功，播放提示音 17
        Play_System_Audio(17);
    } else {
        response.success = 0;
        sprintf(response.message, "ERR: Usage CONF:MASK(hex),GAIN,SPS,INPUT_TYPE,BIAS_EN");
    }
    return response;
}

/**
 * @brief 处理 POWER_TEST 指令，启动系统极限功耗测试
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 同时开启脑电采集、SD 卡写入、循环音频播放和 LED 常亮，以评估电池极限续航
 */
cmd_response_t cmd_power_test_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }
    
    // 1. 打开 SD 卡文件用于连续写入测试数据
    FRESULT fr = f_open(&SDFile, "PWR_TEST.BIN", FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        response.success = 0;
        sprintf(response.message, "ERR: SD Open failed (%d)", fr);
        return response;
    }
    
    // 2. 启动 ADS1299 (最高功耗模式: 全通道开启，24倍增益，250 SPS)
    ADS1299_Stop();
    ADS1299_Set_Sample_Parameter(0x06, 6, 0); // 250SPS
    
    // 开启所有通道
    for(int i=1; i<=8; i++) {
        ADS1299_Write_Reg(0x04 + i, 0x60); 
    }
    // 开启右腿驱动偏置
    ADS1299_Write_Reg(0x03, 0xEC); 
    ADS1299_Write_Reg(0x0D, 0xFF); 
    
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);
    
    // 3. 启动音频循环播放 (在底层实现中死循环播放 3.wav)
    Wav_Play("AUDIO/3.wav"); 
    
    // 4. 点亮 LED
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    
    // 5. 切换到极限功耗测试状态
    app_state = APP_POWER_TEST_RUNNING;
    start_time = HAL_GetTick();
    sample_count = 0;
    
    strcpy(response.message, "OK: POWER TEST STARTED (ADS+SD+AUDIO+LED). Send STOP to end.");
    return response;
}

/**
 * @brief 处理 ADC_TEST 指令，启动电池电压连续记录测试
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 每隔 0.5 秒读取一次电池电压并以 CSV 格式追加到 SD 卡的 adc.txt 文件中
 */
cmd_response_t cmd_adc_test_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }
    
    // 以追加模式打开 adc.txt
    FRESULT fr = f_open(&SDFile, "adc.txt", FA_OPEN_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        response.success = 0;
        sprintf(response.message, "ERR: SD Open adc.txt failed (%d)", fr);
        return response;
    }
    
    // 移动指针到文件末尾
    f_lseek(&SDFile, f_size(&SDFile));
    
    // 写入 CSV 表头
    char header[] = "Timestamp_ms,Raw_ADC,Voltage_V,Percent\r\n";
    UINT bw;
    f_write(&SDFile, header, strlen(header), &bw);
    f_sync(&SDFile);
    
    app_state = APP_ADC_TEST_RUNNING;
    start_time = HAL_GetTick(); // 记录起始时间用于控制记录间隔
    
    strcpy(response.message, "OK: ADC Logging Started (0.5s interval). Send STOP to end.");
    return response;
}

/**
 * @brief 处理 STOP 指令，安全停止当前所有正在进行的任务
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 停止采集、关闭正在写入的文件、停止音频并返回空闲状态
 */
cmd_response_t cmd_stop_handler(char* args) {
    cmd_response_t response = {1, "OK: Stopping..."};
    
    // 如果本来就是空闲状态，直接返回，避免在空闲时关闭未打开的文件或错误触发停止音效导致死锁
    if (app_state == APP_IDLE) {
        strcpy(response.message, "OK: Already Idle");
        return response;
    }
    
    // 立即关闭记录文件，防止锁死引发后续操作错误 (如 GET 文件)
    if (app_state == APP_QUAD_RUNNING || app_state == APP_STREAM_RUNNING || app_state == APP_AUTO_STREAM_RUNNING || app_state == APP_POWER_TEST_RUNNING || app_state == APP_ADC_TEST_RUNNING || app_state == APP_IMP_TEST_ALL_RUNNING) {
        if (app_state != APP_ADC_TEST_RUNNING) ADS1299_Stop(); // 仅在有脑电任务时停止 ADS1299
        f_close(&SDFile);
    }
    
    // 触发停止请求状态，由主循环安全收尾
    app_state = APP_STOP_REQUESTED;
    
    return response;
}

/**
 * @brief 处理 PAUSE 指令，暂停当前的在线采集
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 仅支持在线模式下的暂停，并播放暂停音效
 */
cmd_response_t cmd_pause_handler(char* args) {
    cmd_response_t response = {1, "OK: Streaming Paused"};
    
    // 仅针对未写 SD 卡的在线实时流模式有效
    if (app_state == APP_STREAM_RUNNING && g_write_sd_enabled == 0) {
        ADS1299_Stop();
        app_state = APP_STREAM_PAUSED;
        Play_System_Audio(9); // 播放 COLLECT_PAUSE 音效
    } else {
        response.success = 0;
        strcpy(response.message, "ERR: Only online streaming can be paused");
    }
    
    return response;
}

/**
 * @brief 处理 RESUME 指令，恢复之前暂停的在线采集
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 从暂停点无缝继续，不清空环形缓冲区
 */
cmd_response_t cmd_resume_handler(char* args) {
    cmd_response_t response = {1, "OK: Streaming Resumed"};
    
    if (app_state == APP_STREAM_PAUSED && g_write_sd_enabled == 0) {
        Play_System_Audio(10); // 播放 COLLECT_RESUME 音效
        
        ADS1299_Start();
        // 注意：这里不要清空环形缓冲区，以免丢失暂停瞬间的数据
        // circular_buffer_clear(&ADS1299_circula_buffer);
        
        app_state = APP_STREAM_RUNNING;
    } else {
        response.success = 0;
        strcpy(response.message, "ERR: Stream is not paused");
    }
    
    return response;
}

/**
 * @brief 处理 STOP_IMP 指令，结束阻抗测试并进行语音播报反馈
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 评估 8 个通道的最终阻抗，若全部通过(或7个通过且平均值小于200k)播报成功，否则播报失败
 */
cmd_response_t cmd_stop_imp_handler(char* args) {
    cmd_response_t response = {1, "OK: Impedance Stopped"};
    
    if (app_state == APP_IMP_TEST_ALL_RUNNING) {
        int pass_count = 0;
        float sum = 0.0f;
        
        for(int i=0; i<8; i++) {
            if (g_latest_impedances[i] < 100000.0f) {
                pass_count++;
            }
            sum += g_latest_impedances[i];
        }
        float avg = sum / 8.0f;
        
        // 成功判定标准：8个通道全通，或者至少7个通且平均阻抗小于 200kOhm
        if (pass_count == 8 || (pass_count >= 7 && avg < 200000.0f)) {
            Play_System_Audio(5); // 播放 IMPEDANCE_END_OK 语音
        } else {
            Play_System_Audio(6); // 播放 IMPEDANCE_END_FAIL 语音
        }
        
        ADS1299_Stop();
        app_state = APP_IDLE;
        RGB_LED_SetSingle(LED_COLOR_BLUE, LED_MODE_SOLID_ON); // 恢复蓝灯常亮
    } else {
        response.success = 0;
        strcpy(response.message, "ERR: Not in impedance test");
    }
    
    return response;
}

// 用于在中断与主循环间共享 IMU 数据的静态缓冲
static float g_latest_acc[3] = {0};
static float g_latest_gyro[3] = {0};
static uint8_t g_latest_motion_state = 0;

float g_latest_impedances[8] = {0};

/**
 * @brief 系统核心超级状态机与数据处理主循环
 * @note 负责 OTA 文件落盘、停止指令的收尾、ADC 记录、IMU 运动评估以及 ADS1299 数据包的解析、打包与下发
 */
void app_run_loop(void) {
    // 检查来自 USB 中断的 OTA 传输完成标志
    if (g_ota_complete_flag) {
        g_ota_complete_flag = 0;
        
        // 此时已处于主循环上下文，可安全调用 FatFS 关闭文件和重启
        FRESULT fr_close = f_close(&ota_fil);
        uint32_t duration = HAL_GetTick() - ota_start_tick;
        
        if (fr_close == FR_OK) {
            USB_Print("\r\n[OTA] Transfer Complete. %lu bytes.\r\n", ota_received_bytes);
            USB_Print("[OTA] Time: %lu ms. Auto-resetting in 1s...\r\n", duration);
            
            // 等待最后的 USB 消息发送完毕
            HAL_Delay(1000);
            
            // 执行系统软复位，进入 Bootloader
            NVIC_SystemReset();
        } else {
            USB_Print("\r\n[OTA] Error: File Close Failed (%d)\r\n", fr_close);
        }
    }

    if (app_state == APP_IDLE) return;

    if (app_state == APP_STOP_REQUESTED) {
        ADS1299_Stop();
        
        // 修复：在停止时总是尝试关闭 SD 文件对象，确保没有遗留锁死的文件句柄
        // 在空闲或覆盖状态下关闭已关闭的文件是相对安全的
        f_close(&SDFile);
        USB_Print("File closed (if open).\r\n");

        app_state = APP_IDLE;
        
        // 只要回到空闲状态，无论从哪个模式，都恢复蓝灯常亮
        RGB_LED_SetSingle(LED_COLOR_BLUE, LED_MODE_SOLID_ON);
        
        if (g_task_state == 2) { // 任务正在运行
            g_task_state = 3; // 标记为已完成
            
            // 清除备份寄存器中的任务状态和任务 ID
            BackupReg_Write(BKP_REG_TASK_STATE, 0);
            BackupReg_Write(BKP_REG_TASK_ID, 0);
        }
        
        // 停止采集后播放 13.wav (COLLECT_END) 提示音
        Play_System_Audio(13);
        
        USB_Print("Acquisition Stopped. Saved %lu samples.\r\n", sample_count);
        
        // 报告采集过程中出现的坏道历史
        if (stream_bad_ch_history != 0) {
             USB_Print("Warning: Bad channels detected during session: 0x%02X\r\n", stream_bad_ch_history);
        }
        
        return;
    }

    if (app_state == APP_ADC_TEST_RUNNING) {
        if (HAL_GetTick() - start_time >= 500) {
            start_time = HAL_GetTick();
            
            // 启动 ADC 读取
            HAL_ADC_Start(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
                uint32_t raw = HAL_ADC_GetValue(&hadc1);
                // 基于实测的 4.2V (Raw 4031) 和 3.7V (Raw 3519) 进行电压换算
                float voltage = ((float)raw * 0.00097656f) + 0.2635f;
                float percentage = (voltage - 3.6f) / (4.1f - 3.6f) * 100.0f;
                if (percentage < 0) percentage = 0;
                if (percentage > 100) percentage = 100;
                
                // 记录到 SD 卡
                char line[64];
                int len = snprintf(line, sizeof(line), "%lu,%lu,%.3f,%.1f\r\n", 
                                   HAL_GetTick(), raw, voltage, percentage);
                
                UINT bw;
                f_write(&SDFile, line, len, &bw);
                f_sync(&SDFile);
                
                // 通过 USB 打印以供实时监测
                USB_Print("ADC: Raw=%lu, V=%.3f, P=%.1f%%\r\n", raw, voltage, percentage);
            }
        }
    }

    if (app_state == APP_SINGLE_RUNNING || app_state == APP_QUAD_RUNNING || app_state == APP_STREAM_RUNNING || app_state == APP_AUTO_STREAM_RUNNING || app_state == APP_IMP_TEST_RUNNING || app_state == APP_NOISE_TEST_RUNNING || app_state == APP_CMRR_TEST_RUNNING || app_state == APP_CMRR_STEP1_RUNNING || app_state == APP_CMRR_STEP2_RUNNING || app_state == APP_IMP_TEST_ALL_RUNNING || app_state == APP_RLD_TEST_RUNNING || app_state == APP_POWER_TEST_RUNNING) 
			{
        
        // --- 辅助数据处理 (50Hz 采样, 1Hz 输出) 适用于 STREAM 模式 ---
        if (app_state == APP_STREAM_RUNNING) {
            
            static uint32_t last_aux_sample_time = 0;
            static uint32_t last_aux_output_time = 0;
            static float avg_motion = 0.0f;
            static float peak_energy = 0.0f;
            static float last_acc[3] = {0};
            static uint8_t first_sample = 1;

            // 50Hz 采样间隔 = 20ms
            if (HAL_GetTick() - last_aux_sample_time >= 20) {
                float acc[3], gyro[3];
                QMI8658_Read_XYZ(acc, gyro); // 读取 IMU 数据
                
                // 更新全局缓存供二进制流打包使用
                g_latest_acc[0] = acc[0]; g_latest_acc[1] = acc[1]; g_latest_acc[2] = acc[2];
                g_latest_gyro[0] = gyro[0]; g_latest_gyro[1] = gyro[1]; g_latest_gyro[2] = gyro[2];

                if (first_sample) {
                    last_acc[0] = acc[0];
                    last_acc[1] = acc[1];
                    last_acc[2] = acc[2];
                    first_sample = 0;
                }

                // 1. 计算加速度突变 (Jerk) - 曼哈顿距离差
                // |Ax_t - Ax_t-1| + |Ay_t - Ay_t-1| + |Az_t - Az_t-1|
                float acc_diff = fabsf(acc[0] - last_acc[0]) + 
                                 fabsf(acc[1] - last_acc[1]) + 
                                 fabsf(acc[2] - last_acc[2]);
                
                // 钳制极端尖峰 (acc_diff > 0.5g)
                if (acc_diff > 0.5f) acc_diff = 0.5f;

                // Update last acc
                last_acc[0] = acc[0];
                last_acc[1] = acc[1];
                last_acc[2] = acc[2];

                // 2. Calculate Gyro Magnitude
                // sqrt(Gx^2 + Gy^2 + Gz^2)
                float gyro_mag = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
                
                // Clamp extreme values (gyro_mag > 500 deg/s)
                if (gyro_mag > 500.0f) gyro_mag = 500.0f;

                float energy_now = (50.0f * acc_diff) + (0.02f * gyro_mag);

                avg_motion = 0.95f * avg_motion + 0.05f * energy_now;

                if (energy_now > peak_energy) {
                    peak_energy = energy_now;
                } else {
                    peak_energy *= 0.95f;
                }
                
                last_aux_sample_time = HAL_GetTick();
            }

            // 1Hz Output
            if (HAL_GetTick() - last_aux_output_time >= 1000) {
                float acc[3], gyro[3];
                // Read one latest sample for display
                QMI8658_Read_XYZ(acc, gyro);
                
                float final_energy = (avg_motion > peak_energy) ? avg_motion : peak_energy;
                
                // Motion Score Normalization (0-100)
                float motion_score = final_energy * 5.0f;
                if (motion_score > 100.0f) motion_score = 100.0f;
                if (motion_score < 0.0f) motion_score = 0.0f;

                // Call the new Anti-False-Positive and State Machine Layer
                motion_state_t final_state = Motion_State_Update(motion_score);
                g_latest_motion_state = (uint8_t)final_state;
                
                // Trigger ARTIFACT_HIGH Audio (连续2秒最高等级 MOTION_SEVERE)
                static uint8_t severe_consecutive_count = 0;
                if (final_state == MOTION_SEVERE) {
                    severe_consecutive_count++;
                } else {
                    severe_consecutive_count = 0;
                }

                if (severe_consecutive_count >= 2) {
                    static uint32_t last_motion_warn = 0;
                    if (HAL_GetTick() - last_motion_warn > 10000) { // 10秒防抖
                        Play_System_Audio(11); // ARTIFACT_HIGH
                        last_motion_warn = HAL_GetTick();
                    }
                }

                if (g_use_binary_protocol) {
                     // Binary AUX Packet (TYPE=0x05)
                     // Structure: Acc[3]*1000, Gyro[3]*10, Bat*100, Score/State(1), Rsv(1)
                     uint8_t aux_payload[16];
                     
                     // Acc (int16_t, *1000)
                     int16_t a_x = (int16_t)(acc[0] * 1000.0f);
                     int16_t a_y = (int16_t)(acc[1] * 1000.0f);
                     int16_t a_z = (int16_t)(acc[2] * 1000.0f);
                     memcpy(&aux_payload[0], &a_x, 2);
                     memcpy(&aux_payload[2], &a_y, 2);
                     memcpy(&aux_payload[4], &a_z, 2);
                     
                     // Gyro (int16_t, *10)
                     int16_t g_x = (int16_t)(gyro[0] * 10.0f);
                     int16_t g_y = (int16_t)(gyro[1] * 10.0f);
                     int16_t g_z = (int16_t)(gyro[2] * 10.0f);
                     memcpy(&aux_payload[6], &g_x, 2);
                     memcpy(&aux_payload[8], &g_y, 2);
                     memcpy(&aux_payload[10], &g_z, 2);
                     
                     // Battery (uint16_t, *100)
                     extern float batt_raw_vol;
                     uint16_t bat_val = (uint16_t)(batt_raw_vol * 100.0f);
                     memcpy(&aux_payload[12], &bat_val, 2);
                     
                     // Output the 0-4 State instead of raw score
                     aux_payload[14] = (uint8_t)final_state;
                     
                     // Reserved (uint8_t)
                     aux_payload[15] = 0;
                     
                     static uint16_t aux_seq = 0;
                    uint16_t tx_len = BP_Pack(g_bp_tx_buf, BP_TYPE_AUX, aux_seq++, HAL_GetTick(), aux_payload, 16);
                    CDC_Transmit_FS(g_bp_tx_buf, tx_len);
                     
                } else {
                    char aux_line[128];
                    int aux_len = snprintf(aux_line, sizeof(aux_line), 
                        "AUX:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\r\n", 
                        acc[0], acc[1], acc[2], 
                        gyro[0], gyro[1], gyro[2], 
                        batt_raw_vol, final_energy, final_state); 
                    CDC_Transmit_FS((uint8_t*)aux_line, aux_len);
                }
                
                last_aux_output_time = HAL_GetTick();
            }
        }
        // -----------------------------------------------

        // 检查超时限制
        if (sample_count >= target_samples) {
            app_state = APP_STOP_REQUESTED;
            return;
        }

        uint8_t packet[27];
        // ADS1299 数据包: 3 字节状态字 + 8 * 3 字节数据 = 27 字节 + 4 字节时间戳 = 31 字节
        while (circular_buffer_get_data_count(&ADS1299_circula_buffer) >= 31) {
            // 帧同步检查: 探查第一个字节以验证其是否为 ADS1299 状态字头 (0xC0)
            uint8_t header_byte;
            circular_buffer_pop(&ADS1299_circula_buffer, &header_byte);
            
            if ((header_byte & 0xF0) != 0xC0) {
                // 检测到帧未对齐！丢弃此字节并继续搜索 0xC0 帧头。
                continue;
            }
            
            // 找到有效的帧头。将其放入 packet[0] 并读取剩余的 26 字节。
            packet[0] = header_byte;
            circular_buffer_read(&ADS1299_circula_buffer, &packet[1], 26);
            
            // 提取硬件时间戳
            uint8_t ts_buf[4];
            circular_buffer_read(&ADS1299_circula_buffer, ts_buf, 4);
            uint32_t packet_ts = ((uint32_t)ts_buf[0] << 24) | ((uint32_t)ts_buf[1] << 16) | ((uint32_t)ts_buf[2] << 8) | ts_buf[3];
            
            static char line[256];
            int len = 0;
            
            // 增益 24, Vref=4.5V
            // 4.5V / (2^23 - 1) = 0.53644 uV (在增益为 1 时的 LSB)
            // 换算单位: uV / LSB
            float gain_val = (float)g_cfg_gain;
            if (gain_val < 1.0f) gain_val = 1.0f; // 安全检查
            float scale = 0.5364f / gain_val; 

            if (app_state == APP_CMRR_STEP1_RUNNING) {
                // 步骤 1: 测量差模增益
                for(int i=0; i<8; i++) {
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    float voltage = (float)val * scale * 1e-6f; 
                    cmrr_step1_sum_sq[i] += (double)(voltage * voltage);
                }
                if (sample_count >= target_samples - 1) {
                    USB_Print("\r\n[STEP 1] Differential Gain Measurement Done.\r\n");
                    USB_Print("CH | Vout_diff (uVrms) | Measured Gain\r\n");
                    USB_Print("---|-------------------|--------------\r\n");
                    const float Vin_diff_rms = 50.0e-6f * 0.7071f; // 50uVpp = 17.677 uVrms
                    
                    for(int i=0; i<8; i++) {
                        double vout_rms = sqrt(cmrr_step1_sum_sq[i] / target_samples);
                        cmrr_diff_gain[i] = vout_rms / Vin_diff_rms;
                        USB_Print(" %d | %17.3f | %12.4f\r\n", i+1, vout_rms*1e6, cmrr_diff_gain[i]);
                    }
                    USB_Print("Now run TEST_CMRR_FULL:STEP2 with 2V Common Mode Signal.\r\n");
                }
            }
            else if (app_state == APP_CMRR_STEP2_RUNNING) {
                // 步骤 2: 共模增益及最终结果计算
                for(int i=0; i<8; i++) {
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    float voltage = (float)val * scale * 1e-6f; 
                    cmrr_step2_sum_sq[i] += (double)(voltage * voltage);
                }
                if (sample_count >= target_samples - 1) {
                    USB_Print("\r\n[STEP 2] Common Mode Gain Measurement Done.\r\n");
                    USB_Print("=== Final CMRR Results ===\r\n");
                    USB_Print("CH | Diff Gain | Comm Gain | CMRR (dB)\r\n");
                    USB_Print("---|-----------|-----------|----------\r\n");
                    const float Vin_comm_rms = 2.0f * 0.7071f; // 假设输入 2Vpp，即 Vrms = 2 / (2*sqrt(2)) = 0.7071
                    
                    for(int i=0; i<8; i++) {
                        double vout_comm_rms = sqrt(cmrr_step2_sum_sq[i] / target_samples);
                        double comm_gain = vout_comm_rms / 0.7071; // Vin_comm = 0.7071
                        
                        double cmrr_ratio = cmrr_diff_gain[i] / comm_gain;
                        double cmrr_db = 20.0 * log10(cmrr_ratio);
                        
                        USB_Print(" %d | %9.2f | %9.2e | %9.2f\r\n", 
                                  i+1, cmrr_diff_gain[i], comm_gain, cmrr_db);
                    }
                }
            }
            else if (app_state == APP_CMRR_TEST_RUNNING) {
                for(int i=0; i<8; i++) {
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    float voltage = (float)val * scale * 1e-6f; // 将 uV 转换为 Volts 以便计算
                    
                    // 我们关注交流信号的幅度 (RMS)
                    // 简单的 RMS 计算，假设均值接近于 0 或已被高通滤波器移除
                    cmrr_sum_sq[i] += (double)(voltage * voltage);
                }

                if (sample_count >= target_samples - 1) {
                    USB_SendString("\r\n[TEST_CMRR] Analysis Result (Assuming Vin=0.707 Vrms):\r\n");
                    USB_SendString("CH | Vout (uVrms) | CMRR (dB)\r\n");
                    USB_SendString("---|--------------|----------\r\n");
                    
                    for(int i=0; i<8; i++) {
                        double vout_rms = sqrt(cmrr_sum_sq[i] / target_samples);
                        // CMRR = 20 * log10( (Gain * Vin) / Vout )
                        // 增益 = 24
                        double ratio = (24.0 * CMRR_VIN_RMS) / vout_rms;
                        double cmrr_db = 20.0 * log10(ratio);
                        
                        USB_Print(" %d | %12.3f | %8.2f\r\n", 
                                  i+1, vout_rms * 1e6, cmrr_db);
                    }
                }
            }
            else if (app_state == APP_RLD_TEST_RUNNING) {
                // RLD 测试: 计算交流分量的 RMS (标准差)
                for(int i=0; i<8; i++) {
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    float voltage = (float)val * scale; // uV (注意: scale 单位是 uV/LSB)
                    
                    rld_sum[i] += voltage;
                    rld_sum_sq[i] += (double)(voltage * voltage);
                }

                if (sample_count >= target_samples - 1) {
                    USB_Print("\r\n[RLD_TEST] 10s Acquisition Done.\r\n");
                    USB_Print("CH | Mean (uV) | RMS_AC (uVrms)\r\n");
                    USB_Print("---|-----------|---------------\r\n");
                    
                    for(int i=0; i<8; i++) {
                        double mean = rld_sum[i] / target_samples;
                        double variance = (rld_sum_sq[i] / target_samples) - (mean * mean);
                        double rms_ac = (variance > 0) ? sqrt(variance) : 0.0;
                        
                        USB_Print(" %d | %9.2f | %13.3f\r\n", i+1, mean, rms_ac);
                    }
                }
            }
            else if (app_state == APP_NOISE_TEST_RUNNING) {
                for(int i=0; i<8; i++) {
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    float voltage = (float)val * scale;
                    
                    noise_sum[i] += voltage;
                    noise_sum_sq[i] += (voltage * voltage);
                    if (voltage < noise_min[i]) noise_min[i] = voltage;
                    if (voltage > noise_max[i]) noise_max[i] = voltage;
                }
                
                // 检查是否完成 (最后一个样本)
                if (sample_count >= target_samples - 1) { 
                     USB_SendString("\r\n[TEST_NOISE] Analysis Result (uV):\r\n");
                     USB_SendString("CH | Mean (Offset) | Noise (RMS) | P-P (Peak-to-Peak)\r\n");
                     USB_SendString("---|---------------|-------------|-------------------\r\n");
                     
                     for(int i=0; i<8; i++) {
                         double mean = noise_sum[i] / target_samples;
                         double variance = (noise_sum_sq[i] / target_samples) - (mean * mean);
                         if (variance < 0) variance = 0; 
                         double rms_noise = sqrt(variance);
                         float pp = noise_max[i] - noise_min[i];
                         
                         USB_Print(" %d | %13.3f | %11.3f | %17.3f\r\n", 
                                   i+1, mean, rms_noise, pp);
                     }
                }
            }
            else if (app_state == APP_POWER_TEST_RUNNING) {
                // 仅写入文件，由文件写入循环处理
                // 在这个简化版本中，我们只是让数据流动
                
                // 将数据写入 SD 卡
                UINT bw;
                f_write(&SDFile, packet, 27, &bw);
            }
            else if (app_state == APP_IMP_TEST_ALL_RUNNING) {
                // 提取硬件导联脱落状态字 (LOFF_STATP)
                uint8_t loff_statp = 0;
                if (packet[0] & 0x08) loff_statp |= 0x80; // Ch8
                if (packet[0] & 0x04) loff_statp |= 0x40; // Ch7
                if (packet[0] & 0x02) loff_statp |= 0x20; // Ch6
                if (packet[0] & 0x01) loff_statp |= 0x10; // Ch5
                if (packet[1] & 0x80) loff_statp |= 0x08; // Ch4
                if (packet[1] & 0x40) loff_statp |= 0x04; // Ch3
                if (packet[1] & 0x20) loff_statp |= 0x02; // Ch2
                if (packet[1] & 0x10) loff_statp |= 0x01; // Ch1

                static uint8_t imp_lead_off_flag[8] = {0};

                // 8 通道阻抗逻辑
                for(int i=0; i<8; i++) {
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    float voltage = (float)val * scale; 

                    if (voltage > imp_all_max[i]) imp_all_max[i] = voltage;
                    if (voltage < imp_all_min[i]) imp_all_min[i] = voltage;

                    // 只要在 0.5s 窗口内出现过一次脱落标志，就判定为脱落
                    if (loff_statp & (1 << i)) {
                        imp_lead_off_flag[i] = 1;
                    }
                }
                
                imp_window_count++;

                // 每 0.5 秒 (125 个样本) 计算一次阻抗
                if (imp_window_count >= 125) {
                    float impedances[8];
                    int pass_count = 0;
                    
                    for(int i=0; i<8; i++) {
                        float vpp = imp_all_max[i] - imp_all_min[i]; 
                        // Z = Vpp(uV) * 1e-6 / (2 * 24nA) = Vpp / 0.048
                        // 电流增加到 24nA 以提高信噪比
                        impedances[i] = (vpp * 1e-6f) / 48.0e-9f;

                        // 【新增】：硬件脱落否决逻辑
                        if (imp_lead_off_flag[i]) {
                            impedances[i] = 9999000.0f; // 强行拉满到 9999 kOhm
                        }

                        g_latest_impedances[i] = impedances[i]; // 全局存储以供 STOP_IMP 评估
                        
                        // 检查是否 < 50kOhm (成功阈值)
                        if (impedances[i] < 50000.0f) pass_count++;
                        
                        // 重置
                        imp_all_max[i] = -1e9;
                        imp_all_min[i] = 1e9;
                        imp_lead_off_flag[i] = 0; // 重置脱落标志
                    }
                    
                    // 格式: IMP:F5:v,F3:v,FZ:v,F4:v,F6:v,AF8:v,FPZ:v,AF7:v
                    // 值单位为 kOhm，最大限制为 9999
                    
                    if (g_use_binary_protocol) {
                        uint8_t imp_payload[16];
                        for(int i=0; i<8; i++) {
                             // 单位: 1 kOhm
                             // impedances[i] 单位为 Ohm，除以 1000 得到 kOhm
                             float imp_kohm = impedances[i] / 1000.0f;
                             
                             // 新增逻辑：如果计算出的阻抗小于 100kOhm，则再除以 10
                             if (imp_kohm < 100.0f) {
                                 imp_kohm = imp_kohm / 10.0f;
                             }
                             
                             if (imp_kohm > 65535.0f) imp_kohm = 65535.0f;
                             uint16_t imp_val = (uint16_t)imp_kohm;
                             memcpy(&imp_payload[i*2], &imp_val, 2);
                        }
                        static uint16_t imp_seq = 0;
                        uint16_t tx_len = BP_Pack(g_bp_tx_buf, BP_TYPE_IMP, imp_seq++, HAL_GetTick(), imp_payload, 16);
                        CDC_Transmit_FS(g_bp_tx_buf, tx_len);
                    } else {
                        char imp_buf[256];
                        int ascii_imp[8];
                        
                        for (int i=0; i<8; i++) {
                            float imp_kohm = impedances[i] / 1000.0f;
                            if (imp_kohm < 100.0f) {
                                imp_kohm = imp_kohm / 10.0f;
                            }
                            ascii_imp[i] = (int)(imp_kohm > 9999.0f ? 9999 : imp_kohm);
                        }
                        
                        snprintf(imp_buf, sizeof(imp_buf), 
                            "IMP:F5:%d,F3:%d,FZ:%d,F4:%d,F6:%d,AF8:%d,FPZ:%d,AF7:%d\r\n",
                            ascii_imp[0], ascii_imp[1], ascii_imp[2], ascii_imp[3], 
                            ascii_imp[4], ascii_imp[5], ascii_imp[6], ascii_imp[7]
                        );
                        USB_SendString(imp_buf);
                    }
                    
                    imp_window_count = 0;
                }
            }
            else if (app_state == APP_IMP_TEST_RUNNING) {
                // 阻抗测试逻辑 (目前仅支持单通道 CH1)
                int ch_offset = 3 + (current_channel - 1) * 3;
                int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                if (val & 0x800000) val |= 0xFF000000;
                float voltage = (float)val * scale; // 电压单位: uV

                // 调试: 定期打印原始十六进制和电压
                // 每 10 个样本打印一次以避免输出过多，但足以观察
                if (imp_window_count % 10 == 0) {
                     len = snprintf(line, sizeof(line), "DBG: Hex=0x%06X, Val=%ld, Volts=%.2f uV\r\n", 
                                    (unsigned int)(val & 0xFFFFFF), (long)val, voltage);
                     CDC_Transmit_FS((uint8_t*)line, len);
                }

                // 更新最大/最小值
                if (voltage > imp_max_val) imp_max_val = voltage;
                if (voltage < imp_min_val) imp_min_val = voltage;
                imp_window_count++;

                // 定期计算阻抗
                if (imp_window_count >= IMP_WINDOW_SIZE) {
                    float vpp = imp_max_val - imp_min_val; // 峰峰值电压 (单位: uV)
                    
                    // Z = Vpp(uV) * 1e-6 / (2 * I_inj)
                    // I_inj = 6nA = 6e-9 A (通过 LOFF 寄存器 0x01 配置 -> ILEAD_OFF=00 -> 6nA)
                    // Z = Vpp(uV) * 1e-6 / 12e-9 = Vpp(uV) / 0.012
                    float raw_impedance = (vpp * 1e-6f) / (2.0f * 6.0e-9f);
                    
                    // 无校准，使用原始阻抗
                    float impedance = raw_impedance;
                    
                    len = snprintf(line, sizeof(line), "Z: %.2f kOhm (Vpp: %.2f uV)\r\n", 
                                   impedance / 1000.0f, vpp);
                    CDC_Transmit_FS((uint8_t*)line, len);

                    // 累加计算平均值
                    imp_sum += impedance;
                    imp_count++;

                    // 为下一个窗口重置
                    imp_max_val = -1e9;
                    imp_min_val = 1e9;
                    imp_window_count = 0;
                }
                
                if (sample_count >= target_samples - 1 && imp_count > 0) {
                    double avg_imp = imp_sum / imp_count;
                    len = snprintf(line, sizeof(line), "\r\n[TEST_IMP] Average Impedance: %.2f kOhm\r\n", avg_imp / 1000.0);
                    CDC_Transmit_FS((uint8_t*)line, len);
                }
            }
            else if (app_state == APP_SINGLE_RUNNING) {
                int ch_offset = 3 + (current_channel - 1) * 3;
                int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                if (val & 0x800000) val |= 0xFF000000;
                float voltage = (float)val * scale;
                
                len = snprintf(line, sizeof(line), "%.2f\n", voltage);
                // 注意: SINGLE 模式目前不写入 SD 卡，仅根据需求为 QUAD 模式写入
                CDC_Transmit_FS((uint8_t*)line, len);
            } else if (app_state == APP_AUTO_STREAM_RUNNING) {
                // 检查导联脱落 (正极)
                // 状态字 0: 1100 + IN8P..IN5P
                // 状态字 1: IN4P..IN1P + IN8N..IN5N
                
                uint8_t loff_statp = 0;
                // IN8P..IN5P (字节 0 的位 3-0) -> 映射到位 7-4
                if (packet[0] & 0x08) loff_statp |= 0x80; // Ch8
                if (packet[0] & 0x04) loff_statp |= 0x40; // Ch7
                if (packet[0] & 0x02) loff_statp |= 0x20; // Ch6
                if (packet[0] & 0x01) loff_statp |= 0x10; // Ch5
                // IN4P..IN1P (字节 1 的位 7-4) -> 映射到位 3-0
                if (packet[1] & 0x80) loff_statp |= 0x08; // Ch4
                if (packet[1] & 0x40) loff_statp |= 0x04; // Ch3
                if (packet[1] & 0x20) loff_statp |= 0x02; // Ch2
                if (packet[1] & 0x10) loff_statp |= 0x01; // Ch1
                
                uint8_t bad_channels = loff_statp & current_bias_sensp_mask;
                
                // 记录坏道历史
                stream_bad_ch_history |= bad_channels;
                
                if (bad_channels != 0) {
                     // [FIX] 不再在导联脱落时关闭通道或修改 BIAS。
                     // 仅记录坏道历史用于状态报告。硬件继续正常运行所有通道。
                     
                     // 防抖：电极脱落报警，3秒内最多报一次
                     static uint32_t last_loff_warn_auto = 0;
                     if (HAL_GetTick() - last_loff_warn_auto > 3000) {
                         Play_System_Audio(12); // ELECTRODE_LOST
                         last_loff_warn_auto = HAL_GetTick();
                     }
                }
                
                // --- 辅助数据处理 (50Hz 采样, 1Hz 输出) 适用于 AUTO_STREAM 模式 ---
                static uint32_t last_aux_sample_time = 0;
                static uint32_t last_aux_output_time = 0;
                static float avg_motion = 0.0f;
                static float peak_energy = 0.0f;
                static float last_acc[3] = {0};
                static uint8_t first_sample = 1;

                // 50Hz 采样间隔 = 20ms
                if (HAL_GetTick() - last_aux_sample_time >= 20) {
                    float acc[3], gyro[3];
                    QMI8658_Read_XYZ(acc, gyro); // 读取 IMU 数据

                    g_latest_acc[0] = acc[0]; g_latest_acc[1] = acc[1]; g_latest_acc[2] = acc[2];
                    g_latest_gyro[0] = gyro[0]; g_latest_gyro[1] = gyro[1]; g_latest_gyro[2] = gyro[2];

                    if (first_sample) {
                        last_acc[0] = acc[0];
                        last_acc[1] = acc[1];
                        last_acc[2] = acc[2];
                        first_sample = 0;
                    }

                    // 1. 计算加速度突变 (曼哈顿距离差)
                    float acc_diff = fabsf(acc[0] - last_acc[0]) + 
                                     fabsf(acc[1] - last_acc[1]) + 
                                     fabsf(acc[2] - last_acc[2]);
                    
                    // 钳制极端尖峰 (acc_diff > 0.5g)
                    if (acc_diff > 0.5f) acc_diff = 0.5f;

                    // 更新最后一次的加速度
                    last_acc[0] = acc[0];
                    last_acc[1] = acc[1];
                    last_acc[2] = acc[2];

                    // 2. 计算陀螺仪幅值
                    float gyro_mag = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
                    
                    // 钳制极端值 (gyro_mag > 500 deg/s)
                    if (gyro_mag > 500.0f) gyro_mag = 500.0f;

                    float energy_now = (50.0f * acc_diff) + (0.02f * gyro_mag);

                    avg_motion = 0.95f * avg_motion + 0.05f * energy_now;

                    if (energy_now > peak_energy) {
                        peak_energy = energy_now;
                    } else {
                        peak_energy *= 0.95f;
                    }
                    
                    last_aux_sample_time = HAL_GetTick();
                }

                // 1Hz 输出
                if (HAL_GetTick() - last_aux_output_time >= 1000) {
                    float acc[3], gyro[3];
                    QMI8658_Read_XYZ(acc, gyro); // 读取最新的一个样本用于显示
                    
                    float final_energy = (avg_motion > peak_energy) ? avg_motion : peak_energy;
                    
                    // 运动分数归一化 (0-100)
                    float motion_score = final_energy * 5.0f;
                    if (motion_score > 100.0f) motion_score = 100.0f;
                    if (motion_score < 0.0f) motion_score = 0.0f;

                    motion_state_t final_state = Motion_State_Update(motion_score);
                    g_latest_motion_state = (uint8_t)final_state;
                    
                    // 触发 ARTIFACT_HIGH 报警音
                    if (final_state == MOTION_SEVERE || final_state == MOTION_ARTIFACT) {
                        static uint32_t last_motion_warn_auto = 0;
                        if (HAL_GetTick() - last_motion_warn_auto > 10000) { // 10秒防抖
                            Play_System_Audio(11); // ARTIFACT_HIGH
                            last_motion_warn_auto = HAL_GetTick();
                        }
                    }

                    if (g_use_binary_protocol) {
                        uint8_t aux_payload[16];
                        int16_t a_x = (int16_t)(acc[0] * 1000.0f);
                        int16_t a_y = (int16_t)(acc[1] * 1000.0f);
                        int16_t a_z = (int16_t)(acc[2] * 1000.0f);
                        memcpy(&aux_payload[0], &a_x, 2);
                        memcpy(&aux_payload[2], &a_y, 2);
                        memcpy(&aux_payload[4], &a_z, 2);

                        int16_t g_x = (int16_t)(gyro[0] * 10.0f);
                        int16_t g_y = (int16_t)(gyro[1] * 10.0f);
                        int16_t g_z = (int16_t)(gyro[2] * 10.0f);
                        memcpy(&aux_payload[6], &g_x, 2);
                        memcpy(&aux_payload[8], &g_y, 2);
                        memcpy(&aux_payload[10], &g_z, 2);

                        extern float batt_raw_vol;
                        uint16_t bat_val = (uint16_t)(batt_raw_vol * 100.0f);
                        memcpy(&aux_payload[12], &bat_val, 2);

                        aux_payload[14] = (uint8_t)final_state;
                        aux_payload[15] = 0;

                        static uint16_t aux_seq_auto = 0;
                        uint16_t tx_len = BP_Pack(g_bp_tx_buf, BP_TYPE_AUX, aux_seq_auto++, HAL_GetTick(), aux_payload, 16);
                        CDC_Transmit_FS(g_bp_tx_buf, tx_len);
                    } else {
                        extern float batt_raw_vol;
                        char aux_line[128];
                        int aux_len = snprintf(aux_line, sizeof(aux_line), 
                            "AUX:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\r\n", 
                            acc[0], acc[1], acc[2], 
                            gyro[0], gyro[1], gyro[2], 
                            batt_raw_vol, final_energy, final_state);
                        CDC_Transmit_FS((uint8_t*)aux_line, aux_len);
                    }
                    
                    last_aux_output_time = HAL_GetTick();
                }
                
                // 常规流传输逻辑 (与 APP_STREAM_RUNNING 相同)
                float voltages[8];
                int32_t raw_vals[8];
                
                for(int i=0; i<8; i++) {
                    // [FIX] 移除了对脱落通道的数据克隆。
                    // 现在无论其脱落状态如何，我们都解析实际通道数据。
                    // 如果电极脱落，上位机将看到真实的噪声/满偏信号，
                    // 这是在医学上正确的行为。
                    
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    
                    raw_vals[i] = val;
                    voltages[i] = (float)val * scale;
                }
                
                if (g_use_binary_protocol) {
                    // 状态字 (3字节)
                    g_bp_payload[0] = packet[0];
                    g_bp_payload[1] = packet[1];
                    g_bp_payload[2] = packet[2];
                    
                    // 通道数据 (32字节)
                    for(int i=0; i<8; i++) {
                        int32_t val = raw_vals[i];
                        
                        g_bp_payload[3 + i*4] = val & 0xFF;
                        g_bp_payload[3 + i*4 + 1] = (val >> 8) & 0xFF;
                        g_bp_payload[3 + i*4 + 2] = (val >> 16) & 0xFF;
                        g_bp_payload[3 + i*4 + 3] = (val >> 24) & 0xFF;
                    }
                    
                    // IMU 数据 (12字节)
                    for(int k=0; k<3; k++) {
                        int16_t acc_val = (int16_t)(g_latest_acc[k] * 1000.0f);
                        int16_t gyro_val = (int16_t)(g_latest_gyro[k] * 10.0f);
                        
                        int acc_idx = 35 + k*2;
                        g_bp_payload[acc_idx] = acc_val & 0xFF;
                        g_bp_payload[acc_idx+1] = (acc_val >> 8) & 0xFF;
                        
                        int gyro_idx = 41 + k*2;
                        g_bp_payload[gyro_idx] = gyro_val & 0xFF;
                        g_bp_payload[gyro_idx+1] = (gyro_val >> 8) & 0xFF;
                    }
                    
                    static uint16_t bin_seq = 0;
                    uint16_t tx_len = BP_Pack(g_bp_tx_buf, BP_TYPE_EEG, bin_seq++, packet_ts, g_bp_payload, 47);
                    
                    // 直接发送 (无减速)
                    uint8_t usb_res = CDC_Transmit_FS(g_bp_tx_buf, tx_len);
                    if (usb_res == USBD_BUSY) {
                        for(volatile int w=0; w<1000; w++); 
                        usb_res = CDC_Transmit_FS(g_bp_tx_buf, tx_len);
                    }
                } else {
                    // EEG 数据传输 (替换了 LOFF 状态字符串)
                    len = snprintf(line, sizeof(line), "EEG:C4:%.2f,C2:%.2f,CZ:%.2f,C1:%.2f,C3:%.2f,AF7:%.2f,FPZ:%.2f,AF8:%.2f\r\n",        
                                   voltages[4], voltages[3], voltages[2], voltages[0],
                                   voltages[1], voltages[7], voltages[6], voltages[5]);
                    
                    uint8_t usb_res = CDC_Transmit_FS((uint8_t*)line, len);
                    if (usb_res == USBD_BUSY) {
                        uint32_t retry_start = HAL_GetTick();
                        while (CDC_Transmit_FS((uint8_t*)line, len) == USBD_BUSY) {
                            if (HAL_GetTick() - retry_start > 2) break; 
                        }
                    }
                }
                
                // 写入 SD 卡 (二进制数据帧)
                DataFrame_t frame = {0};
                frame.frame_seq = sample_count;
                for(int i=0; i<8; i++) {
                    frame.eeg_data[i] = raw_vals[i];
                }
                for(int k=0; k<3; k++) {
                    frame.acc[k] = (int16_t)(g_latest_acc[k] * 1000.0f);
                    frame.gyro[k] = (int16_t)(g_latest_gyro[k] * 10.0f);
                }
                frame.motion_state = g_latest_motion_state;
                
                // 计算简单的校验和
                uint8_t *p_frame = (uint8_t*)&frame;
                uint8_t chk = 0;
                for(size_t i=0; i<sizeof(DataFrame_t)-1; i++) {
                    chk ^= p_frame[i];
                }
                frame.checksum = chk;
                
                // 仅在启用时写入 SD 卡 (二进制数据帧)
                if (g_write_sd_enabled) {
                    UINT bw;
                    FRESULT fr = f_write(&SDFile, &frame, sizeof(DataFrame_t), &bw);
                    if (fr == FR_OK && bw == sizeof(DataFrame_t)) {
                        if (sample_count % 50 == 0) {
                            f_sync(&SDFile);
                        }
                    }
                }
                
            } else if (app_state == APP_STREAM_RUNNING) {
                // 检查导联脱落 (正极)
                // 状态字 0: 1100 + IN8P..IN5P (位 3-0)
                // 状态字 1: IN4P..IN1P (位 7-4) + IN8N..IN5N (位 3-0)
                
                uint8_t loff_statp = 0;
                // IN8P..IN5P (字节 0 的位 3-0) -> 映射到位 7-4
                if (packet[0] & 0x08) loff_statp |= 0x80; // Ch8
                if (packet[0] & 0x04) loff_statp |= 0x40; // Ch7
                if (packet[0] & 0x02) loff_statp |= 0x20; // Ch6
                if (packet[0] & 0x01) loff_statp |= 0x10; // Ch5
                // IN4P..IN1P (字节 1 的位 7-4) -> 映射到位 3-0
                if (packet[1] & 0x80) loff_statp |= 0x08; // Ch4
                if (packet[1] & 0x40) loff_statp |= 0x04; // Ch3
                if (packet[1] & 0x20) loff_statp |= 0x02; // Ch2
                if (packet[1] & 0x10) loff_statp |= 0x01; // Ch1
                
                // 识别脱落且在偏置网络中启用的通道
                uint8_t new_bad_channels = loff_statp & current_bias_sensp_mask;
                
                if (new_bad_channels != 0) {
                     // [FIX] 不再在导联脱落时关闭通道或修改 BIAS。
                     // 仅记录坏道历史用于状态报告。硬件继续正常运行所有通道。
                     
                     // 累加历史记录
                     stream_bad_ch_history |= new_bad_channels;
                     
                     // 防抖：电极脱落报警，3秒内最多报一次
                     static uint32_t last_loff_warn = 0;
                     if (HAL_GetTick() - last_loff_warn > 3000) {
                         Play_System_Audio(12); // ELECTRODE_LOST
                         last_loff_warn = HAL_GetTick();
                     }
                }
                
                float voltages[8];
                int32_t raw_vals[8];
                
                // 解析通道数据 (包含了脱落通道不克隆数据的逻辑)
                for(int i=0; i<8; i++) {
                    // [FIX] 移除了对脱落通道的数据克隆。
                    // 现在无论其脱落状态如何，我们都解析实际通道数据。
                    // 如果电极脱落，上位机将看到真实的噪声/满偏信号，
                    // 这是在医学上正确的行为。
                    
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    
                    raw_vals[i] = val;
                    voltages[i] = (float)val * scale;
                }
                
                // 格式: EEG:SEQ:x,v1,v2,v3,v4,v5,v6,v7,v8\n
                if (g_use_binary_protocol) {
                    // 使用历史脱落状态更新状态字节，以使 UI 保持红色警示
                    // packet[0] 位 3-0: CH8, CH7, CH6, CH5
                    // packet[1] 位 7-4: CH4, CH3, CH2, CH1
                    uint8_t pkt0 = packet[0];
                    uint8_t pkt1 = packet[1];
                    
                    if (stream_bad_ch_history & (1 << 0)) pkt1 |= 0x10; // CH1
                    if (stream_bad_ch_history & (1 << 1)) pkt1 |= 0x20; // CH2
                    if (stream_bad_ch_history & (1 << 2)) pkt1 |= 0x40; // CH3
                    if (stream_bad_ch_history & (1 << 3)) pkt1 |= 0x80; // CH4
                    
                    if (stream_bad_ch_history & (1 << 4)) pkt0 |= 0x01; // CH5
                    if (stream_bad_ch_history & (1 << 5)) pkt0 |= 0x02; // CH6
                    if (stream_bad_ch_history & (1 << 6)) pkt0 |= 0x04; // CH7
                    if (stream_bad_ch_history & (1 << 7)) pkt0 |= 0x08; // CH8
                    
                    g_bp_payload[0] = pkt0; g_bp_payload[1] = pkt1; g_bp_payload[2] = packet[2];
                    
                    for(int i=0; i<8; i++) {
                        int32_t val = raw_vals[i];
                        g_bp_payload[3 + i*4] = val & 0xFF;
                        g_bp_payload[3 + i*4 + 1] = (val >> 8) & 0xFF;
                        g_bp_payload[3 + i*4 + 2] = (val >> 16) & 0xFF;
                        g_bp_payload[3 + i*4 + 3] = (val >> 24) & 0xFF;
                    }
                    
                    for(int k=0; k<3; k++) {
                        int16_t acc_val = (int16_t)(g_latest_acc[k] * 1000.0f);
                        int16_t gyro_val = (int16_t)(g_latest_gyro[k] * 10.0f);
                        int acc_idx = 35 + k*2;
                        g_bp_payload[acc_idx] = acc_val & 0xFF; g_bp_payload[acc_idx+1] = (acc_val >> 8) & 0xFF;
                        int gyro_idx = 41 + k*2;
                        g_bp_payload[gyro_idx] = gyro_val & 0xFF; g_bp_payload[gyro_idx+1] = (gyro_val >> 8) & 0xFF;
                    }
                    
                    static uint16_t bin_seq = 0;
                    uint16_t tx_len = BP_Pack(g_bp_tx_buf, BP_TYPE_EEG, bin_seq++, packet_ts, g_bp_payload, 47);
                    
                    // 直接发送 (无减速)
                    uint8_t usb_res = CDC_Transmit_FS(g_bp_tx_buf, tx_len);
                    if (usb_res == USBD_BUSY) {
                         // 尝试非常短暂的忙等待
                         for(volatile int w=0; w<1000; w++); 
                         usb_res = CDC_Transmit_FS(g_bp_tx_buf, tx_len);
                    }
                } else {
								    len = snprintf(line, sizeof(line), "EEG:C4:%.2f,C2:%.2f,CZ:%.2f,C1:%.2f,C3:%.2f,AF7:%.2f,FPZ:%.2f,AF8:%.2f\r\n",        
                                   voltages[4], voltages[3], voltages[2], voltages[0],
                                   voltages[1], voltages[7], voltages[6], voltages[5]);
                    uint8_t usb_res = CDC_Transmit_FS((uint8_t*)line, len);
                    if (usb_res == USBD_BUSY) {
                        uint32_t retry_start = HAL_GetTick();
                        while (CDC_Transmit_FS((uint8_t*)line, len) == USBD_BUSY) {
                            if (HAL_GetTick() - retry_start > 2) break; // 2ms 超时 - 不阻塞
                        }
                    }
                }
                
                // 注意: 如果 USB 断开连接，CDC_Transmit_FS 可能会返回 USBD_FAIL 或直接丢弃数据。
                // 关键在于上面的超时防止了无限等待。
                
                // 写入 SD 卡 (边流式传输边保存二进制)
                DataFrame_t frame = {0};
                frame.frame_seq = sample_count;
                for(int i=0; i<8; i++) {
                    frame.eeg_data[i] = raw_vals[i];
                }
                for(int k=0; k<3; k++) {
                    frame.acc[k] = (int16_t)(g_latest_acc[k] * 1000.0f);
                    frame.gyro[k] = (int16_t)(g_latest_gyro[k] * 10.0f);
                }
                frame.motion_state = g_latest_motion_state;
                
                // 计算简单的校验和
                uint8_t *p_frame = (uint8_t*)&frame;
                uint8_t chk = 0;
                for(size_t i=0; i<sizeof(DataFrame_t)-1; i++) {
                    chk ^= p_frame[i];
                }
                frame.checksum = chk;
                
                // 仅在启用时写入 SD 卡 (边流式传输边保存二进制)
                if (g_write_sd_enabled) {
                    UINT bw;
                    FRESULT fr = f_write(&SDFile, &frame, sizeof(DataFrame_t), &bw);
                    if (fr != FR_OK || bw < sizeof(DataFrame_t)) {
                        // SD 写入错误 - 暂且继续
                    } else {
                        // 定期同步 (每 50 个样本 ~ 0.2s)
                        if (sample_count % 50 == 0) {
                            f_sync(&SDFile);
                        }
                    }
                }
                
            } else {
                // QUAD (8 通道) 模式
                DataFrame_t frame = {0};
                frame.frame_seq = sample_count;
                for(int i=0; i<8; i++) {
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    frame.eeg_data[i] = val;
                }
                for(int k=0; k<3; k++) {
                    frame.acc[k] = (int16_t)(g_latest_acc[k] * 1000.0f);
                    frame.gyro[k] = (int16_t)(g_latest_gyro[k] * 10.0f);
                }
                frame.motion_state = g_latest_motion_state;
                
                // 计算简单的校验和
                uint8_t *p_frame = (uint8_t*)&frame;
                uint8_t chk = 0;
                for(size_t i=0; i<sizeof(DataFrame_t)-1; i++) {
                    chk ^= p_frame[i];
                }
                frame.checksum = chk;
                
                // 仅在启用时写入 SD 卡
                if (g_write_sd_enabled) {
                    UINT bw;
                    FRESULT fr = f_write(&SDFile, &frame, sizeof(DataFrame_t), &bw);
                    if (fr != FR_OK || bw < sizeof(DataFrame_t)) {
                        if (g_use_binary_protocol == 0) USB_Print("SD Write Err: %d\r\n", fr);
                        app_state = APP_STOP_REQUESTED; // 写入错误时停止
                    } else {
                        if (sample_count % 50 == 0) {
                            f_sync(&SDFile);
                        }
                    }
                }
            }

            sample_count++;
            
            if (HAL_GetTick() - last_display_time > 1000) {
                 // USB_Print("Sampling... %lu\r\n", sample_count);
                 last_display_time = HAL_GetTick();
            }
        }
    }
}

/**
 * @brief 处理 REG 指令，读取并打印 ADS1299 所有寄存器的当前配置值
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 必须在空闲状态(无采集运行)下执行，以便发送 SDATAC 指令停止连续数据模式并读取寄存器
 */
cmd_response_t cmd_reg_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition running, stop first");
        return response;
    }

    uint8_t regs[24];
    memset(regs, 0, 24);

    // 确保设备处于停止连续读取模式 (SDATAC) 才能读取寄存器
    ADS1299_Send_Cmd(ADS1299_SDATAC);
    HAL_Delay(10); 

    // 读取所有寄存器 (0x00 到 0x17)
    ADS1299_Read_Regs(0x00, regs, 24);

    USB_Print("\r\n=== ADS1299 Registers ===\r\n");
		HAL_Delay(10);
    USB_Print("ID:      0x%02X  CONFIG1: 0x%02X  CONFIG2: 0x%02X\r\n", regs[0x00], regs[0x01], regs[0x02]);
		HAL_Delay(10);
    USB_Print("CONFIG3: 0x%02X  LOFF:    0x%02X  CH1SET:  0x%02X\r\n", regs[0x03], regs[0x04], regs[0x05]);
		HAL_Delay(10);
    USB_Print("CH2SET:  0x%02X  CH3SET:  0x%02X  CH4SET:  0x%02X\r\n", regs[0x06], regs[0x07], regs[0x08]);
		HAL_Delay(10);
    USB_Print("CH5SET:  0x%02X  CH6SET:  0x%02X  CH7SET:  0x%02X\r\n", regs[0x09], regs[0x0A], regs[0x0B]);
		HAL_Delay(10);
    USB_Print("CH8SET:  0x%02X  BIAS_P:  0x%02X  BIAS_N:  0x%02X\r\n", regs[0x0C], regs[0x0D], regs[0x0E]);
		HAL_Delay(10);
    USB_Print("LOFF_P:  0x%02X  LOFF_N:  0x%02X  LOFF_F:  0x%02X\r\n", regs[0x0F], regs[0x10], regs[0x11]);
		HAL_Delay(10);
    USB_Print("LOFF_SP: 0x%02X  LOFF_SN: 0x%02X  GPIO:    0x%02X\r\n", regs[0x12], regs[0x13], regs[0x14]);
		HAL_Delay(10);
    USB_Print("MISC1:   0x%02X  MISC2:   0x%02X  CONFIG4: 0x%02X\r\n", regs[0x15], regs[0x16], regs[0x17]);
		HAL_Delay(10);
    
    strcpy(response.message, "OK: Registers read");
    return response;
}

/**
 * @brief 处理 TEST_IMP 指令，启动单通道 (CH1) 阻抗测量测试
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 硬件配置: 注入 24nA 交流信号(fDR/4)，开启比较器，并在循环中打印交流阻抗
 */
cmd_response_t cmd_test_imp_handler(char* args) {
    cmd_response_t response = {1, ""};

    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    ADS1299_Stop();
    HAL_Delay(10);

    // 手动配置阻抗测试相关的硬件寄存器
    ADS1299_Write_Reg(0x00, 0x3E); // ID 寄存器 (只读)
    ADS1299_Write_Reg(0x01, 0x96); // CONFIG1: 菊花链禁用，振荡器时钟输出禁用，250 SPS
    ADS1299_Write_Reg(0x02, 0xC0); // CONFIG2: 内部测试信号生成
    ADS1299_Write_Reg(0x03, 0xE6); // CONFIG3: 开启内部参考缓冲，VREF = 4.5V
    ADS1299_Write_Reg(0x04, 0x01); // LOFF: 24nA 电流，交流阻抗脱落检测 (fDR/4)

    // CHnSET 配置: 0x60 = 24 倍增益正常输入; 0x80 = 通道关断
    ADS1299_Write_Reg(0x05, 0x60); // CH1: 启用, 增益 24, 正常输入
    ADS1299_Write_Reg(0x06, 0x80); // CH2: 关断
    ADS1299_Write_Reg(0x07, 0x80); // CH3: 关断
    ADS1299_Write_Reg(0x08, 0x80); // CH4: 关断
    ADS1299_Write_Reg(0x09, 0x80); // CH5: 关断
    ADS1299_Write_Reg(0x0A, 0x80); // CH6: 关断
    ADS1299_Write_Reg(0x0B, 0x80); // CH7: 关断
    ADS1299_Write_Reg(0x0C, 0x80); // CH8: 关断

    ADS1299_Write_Reg(0x0D, 0x00); // BIAS_SENSP: 禁用
    ADS1299_Write_Reg(0x0E, 0x00); // BIAS_SENSN: 禁用
    ADS1299_Write_Reg(0x0F, 0x01); // LOFF_SENSP: 仅启用 CH1 正极脱落检测
    ADS1299_Write_Reg(0x10, 0x00); // LOFF_SENSN: 禁用
    ADS1299_Write_Reg(0x11, 0x00); // LOFF_FLIP: 禁用
    ADS1299_Write_Reg(0x12, 0x00); // LOFF_STATP (只读)
    ADS1299_Write_Reg(0x13, 0x00); // LOFF_STATN (只读)
    ADS1299_Write_Reg(0x14, 0x00); // GPIO
    ADS1299_Write_Reg(0x15, 0x20); // MISC1: 将 SRB1 连接到所有的负极输入 (短接参考)
    ADS1299_Write_Reg(0x16, 0x00); // MISC2
    ADS1299_Write_Reg(0x17, 0x02); // CONFIG4: 开启 PD_LOFF_COMP 导联脱落硬件比较器

    USB_Print("[TEST_IMP] Registers Configured.\r\n");
		
    USB_Print("\r\n[BOOT] ADS1299 Registers before START:\r\n");
		uint8_t regs[24];
		ADS1299_Read_Regs(0x00, regs, 24);
		for(int i=0; i<24; i++) {
				USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]);
				HAL_Delay(5);
		}

    // 播放提示音 3.wav (开始阻抗检测)
    Play_System_Audio(3);

    // 启动采集
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    // 开始阻抗检测运行，蓝色0.5s闪烁 (中速闪烁)
    RGB_LED_SetSingle(LED_COLOR_BLUE, LED_MODE_FLASH_MID);

    app_state = APP_IMP_TEST_RUNNING; 
    start_time = HAL_GetTick();
    sample_count = 0;
    target_samples = 60 * 250; // 测试时长限制为 1 分钟
    last_display_time = start_time;
    current_channel = 1; // 仅聚焦于通道 1

    // 重置阻抗统计参数
    imp_max_val = -1e9;
    imp_min_val = 1e9;
    imp_window_count = 0;
    imp_sum = 0.0;
    imp_count = 0;

    strcpy(response.message, "OK: TEST_IMP Started (CH1)");
    return response;
}

/**
 * @brief 处理 TEST_IMP_ALL 指令，启动全通道 (8CH) 阻抗测量测试
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 与 TEST_IMP 类似，但配置所有通道开启并接入脱落检测网络，并在主循环中上报二进制或文本格式阻抗值
 */
cmd_response_t cmd_test_imp_all_handler(char* args) {
    cmd_response_t response = {1, ""};

    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    ADS1299_Stop();
    HAL_Delay(10);
    
    // 切换为二进制协议模式以发送 TYPE_IMP 数据包
    extern volatile uint8_t g_use_binary_protocol;
    g_use_binary_protocol = 1;

    // 手动配置全通道阻抗测试相关的硬件寄存器
    ADS1299_Write_Reg(0x01, 0x96); // CONFIG1: 250 SPS
    ADS1299_Write_Reg(0x02, 0xC0); // CONFIG2: 内部测试信号生成
    ADS1299_Write_Reg(0x03, 0xE6); // CONFIG3: 开启内部参考缓冲，VREF = 4.5V
    ADS1299_Write_Reg(0x04, 0x05); // LOFF: 24nA 电流，交流阻抗脱落检测 (fDR/4)

    // 启用所有 8 个通道 (增益 24，正常输入)
    for(int i=1; i<=8; i++) {
        ADS1299_Write_Reg(0x04 + i, 0x60); 
    }

    ADS1299_Write_Reg(0x0D, 0x00); // BIAS_SENSP
    ADS1299_Write_Reg(0x0E, 0x00); // BIAS_SENSN
    ADS1299_Write_Reg(0x0F, 0xFF); // LOFF_SENSP: 为所有正极输入启用脱落检测
    ADS1299_Write_Reg(0x10, 0x00); // LOFF_SENSN
    ADS1299_Write_Reg(0x11, 0x00); // LOFF_FLIP
    ADS1299_Write_Reg(0x15, 0x20); // MISC1: 将 SRB1 连接到所有的负极输入
    ADS1299_Write_Reg(0x17, 0x02); // CONFIG4: 开启 PD_LOFF_COMP 导联脱落硬件比较器

    USB_Print("[TEST_IMP_ALL] Registers Configured (8CH).\r\n");
    
    // 读回并打印寄存器配置
    USB_Print("\r\n[BOOT] ADS1299 Registers before START:\r\n");
    uint8_t regs[24];
    ADS1299_Read_Regs(0x00, regs, 24);
    
    // 在二进制模式下，避免打印 24 行文本，这可能导致 USB 缓冲区溢出或上位机解析错误
    extern volatile uint8_t g_use_binary_protocol;
    if (!g_use_binary_protocol) {
        for(int i=0; i<24; i++) {
            USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]);
            HAL_Delay(5);
        }
    } else {
        // 仅打印摘要或保持静默以防止阻塞主循环
        USB_Print("Registers dumped (omitted for binary mode).\r\n");
    }
    
    // 播放提示音 3.wav (开始阻抗检测)
    Play_System_Audio(3);
    
    // 启动采集
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    // 开始阻抗检测运行，蓝色0.5s闪烁 (中速闪烁)
    RGB_LED_SetSingle(LED_COLOR_BLUE, LED_MODE_FLASH_MID);

    app_state = APP_IMP_TEST_ALL_RUNNING; 
    start_time = HAL_GetTick();
    sample_count = 0;
    target_samples = 0xFFFFFFFF; // 无限期运行 (直到接收到 STOP 指令)
    last_display_time = start_time;

    // 重置阻抗统计参数
    for(int i=0; i<8; i++) {
        imp_all_max[i] = -1e9;
        imp_all_min[i] = 1e9;
    }
    imp_window_count = 0;

    strcpy(response.message, "OK: TEST_IMP_ALL Started (8CH). Send STOP to end.");
    return response;
}

/**
 * @brief 处理 TEST_NOISE 指令，启动系统内部本底噪声测试
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 所有通道输入短接，采集 1 秒钟的数据以计算 RMS 噪声和峰峰值
 */
cmd_response_t cmd_test_noise_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }

    ADS1299_Stop();
    HAL_Delay(10);

    // 1. 配置寄存器
    // 0x00 为只读
    ADS1299_Write_Reg(0x01, 0x96); // CONFIG1: 250 SPS
    ADS1299_Write_Reg(0x02, 0xC0); // CONFIG2: 开启内部测试信号
    ADS1299_Write_Reg(0x03, 0xE6); // CONFIG3: 开启参考缓冲，VRef 4.5V
    ADS1299_Write_Reg(0x04, 0x00); // LOFF: 禁用

    // CH1-CH8: 增益 24，输入短接以测量本底噪声 (0x61)
    for(int i=1; i<=8; i++) {
        ADS1299_Write_Reg(0x04 + i, 0x61); 
    }
    
    ADS1299_Write_Reg(0x0D, 0xFF); // BIAS_SENSP: 全部启用
    ADS1299_Write_Reg(0x0E, 0x00); // BIAS_SENSN
    ADS1299_Write_Reg(0x0F, 0x00); // LOFF_SENSP
    ADS1299_Write_Reg(0x10, 0x00); // LOFF_SENSN
    ADS1299_Write_Reg(0x11, 0x00); // LOFF_FLIP
    ADS1299_Write_Reg(0x12, 0x00); // LOFF_STATP (RO)
    ADS1299_Write_Reg(0x13, 0x00); // LOFF_STATN (RO)
    ADS1299_Write_Reg(0x14, 0x00); // GPIO
    ADS1299_Write_Reg(0x15, 0x20); // MISC1: SRB1
    ADS1299_Write_Reg(0x16, 0x00); // MISC2
    ADS1299_Write_Reg(0x17, 0x00); // CONFIG4

    USB_Print("[TEST_NOISE] Registers Configured.\r\n");

    // 2. 读回并打印寄存器
    USB_Print("\r\n[BOOT] ADS1299 Registers after Config:\r\n");
    uint8_t regs[24];
    ADS1299_Read_Regs(0x00, regs, 24);
    for(int i=0; i<24; i++) {
        USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]);
        HAL_Delay(5);
    }

    // 3. 初始化变量
    for(int i=0; i<8; i++) {
        noise_sum[i] = 0.0;
        noise_sum_sq[i] = 0.0;
        noise_min[i] = 1e9;
        noise_max[i] = -1e9;
    }

    // 4. 启动采集
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    app_state = APP_NOISE_TEST_RUNNING;
    start_time = HAL_GetTick();
    sample_count = 0;
    target_samples = 250; // 1 秒 @ 250SPS
    last_display_time = start_time;

    strcpy(response.message, "OK: TEST_NOISE started (1 sec)");
    return response;
}

/**
 * @brief 处理 TEST_CMRR 指令，启动单步共模抑制比测试
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 提示用户输入共模信号，运行 2 秒后计算并打印 CMRR 结果
 */
cmd_response_t cmd_test_cmrr_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }

    ADS1299_Stop();
    HAL_Delay(10);

    USB_Print("[TEST_CMRR] Please ensure 2Vpp Common Mode signal is applied to all inputs.\r\n");

    // 1. 配置寄存器
    // 0x00 为只读 (ID)
    ADS1299_Write_Reg(0x01, 0x96); // CONFIG1: 250 SPS
    ADS1299_Write_Reg(0x02, 0xC0); // CONFIG2: 开启内部测试信号
    ADS1299_Write_Reg(0x03, 0xF6); // CONFIG3: VREF=4.5V, RLDREF_INT=1, PD_RLD=1, RLD_MEAS=1
    ADS1299_Write_Reg(0x04, 0x00); // LOFF: 禁用

    // CH1-CH8: 增益 24，正常输入 (0x60)
    for(int i=1; i<=8; i++) {
        ADS1299_Write_Reg(0x04 + i, 0x60); 
    }
    
    ADS1299_Write_Reg(0x0D, 0xFF); // BIAS_SENSP: 全部启用
    ADS1299_Write_Reg(0x0E, 0x00); // BIAS_SENSN
    ADS1299_Write_Reg(0x0F, 0x00); // LOFF_SENSP
    ADS1299_Write_Reg(0x10, 0x00); // LOFF_SENSN
    ADS1299_Write_Reg(0x11, 0x00); // LOFF_FLIP
    ADS1299_Write_Reg(0x12, 0x00); // LOFF_STATP (RO)
    ADS1299_Write_Reg(0x13, 0x00); // LOFF_STATN (RO)
    ADS1299_Write_Reg(0x14, 0x00); // GPIO
    ADS1299_Write_Reg(0x15, 0x20); // MISC1: SRB1
    ADS1299_Write_Reg(0x16, 0x00); // MISC2
    ADS1299_Write_Reg(0x17, 0x00); // CONFIG4

    USB_Print("[TEST_CMRR] Registers Configured.\r\n");

    // 2. 读回并打印
    USB_Print("\r\n[BOOT] ADS1299 Registers after Config:\r\n");
    uint8_t regs[24];
    ADS1299_Read_Regs(0x00, regs, 24);
    for(int i=0; i<24; i++) {
        USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]);
        HAL_Delay(5);
    }

    // 3. 初始化变量
    for(int i=0; i<8; i++) {
        cmrr_sum_sq[i] = 0.0;
    }

    // 4. 启动采集
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    app_state = APP_CMRR_TEST_RUNNING;
    start_time = HAL_GetTick();
    sample_count = 0;
    target_samples = 500; // 2 秒 @ 250SPS
    last_display_time = start_time;
    
    strcpy(response.message, "OK: TEST_CMRR started (2 sec)");
    return response;
}

/**
 * @brief 处理 TEST_CMRR_FULL 指令，启动两步法精确共模抑制比测试
 * @param args 包含当前测试步骤的参数 (STEP1 或 STEP2)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note STEP1 测量差模增益，STEP2 测量共模增益并综合计算最终的 CMRR(dB)
 */
cmd_response_t cmd_test_cmrr_full_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }

    if (args != NULL && strcmp(args, "STEP1") == 0) {
        // STEP 1: 差模增益测试
        // 要求: 输入 4mV 差模信号
        USB_Print("[TEST_CMRR_FULL] STEP 1: Measuring Differential Gain.\r\n");
        USB_Print("Please Apply 4mV (1.414 mVrms) Differential Signal to all channels.\r\n");

        ADS1299_Stop();
        HAL_Delay(10);
        
        // 配置为正常输入 (增益 24)
        ADS1299_Write_Reg(0x01, 0x96); 
        ADS1299_Write_Reg(0x02, 0xC0); 
        ADS1299_Write_Reg(0x03, 0xF6); 
        ADS1299_Write_Reg(0x04, 0x00); 

        for(int i=1; i<=8; i++) ADS1299_Write_Reg(0x04 + i, 0x60); 
        
        ADS1299_Write_Reg(0x0D, 0xFF); 
        ADS1299_Write_Reg(0x0E, 0x00); 
        ADS1299_Write_Reg(0x15, 0x20); // MISC1: SRB1 连接到所有负极输入
        
        // 初始化变量
        for(int i=0; i<8; i++) cmrr_step1_sum_sq[i] = 0.0;
        
        ADS1299_Start();
        circular_buffer_clear(&ADS1299_circula_buffer);
        
        app_state = APP_CMRR_STEP1_RUNNING;
        start_time = HAL_GetTick();
        sample_count = 0;
        target_samples = 500; // 2 秒
        
        strcpy(response.message, "OK: STEP1 Started");
    } 
    else if (args != NULL && strcmp(args, "STEP2") == 0) {
        // STEP 2: 共模增益测试
        // 要求: 输入 2V 共模信号
        USB_Print("[TEST_CMRR_FULL] STEP 2: Measuring Common Mode Gain.\r\n");
        USB_Print("Please Apply 2V (0.7071 Vrms) Common Mode Signal to all channels.\r\n");
        
        ADS1299_Stop();
        HAL_Delay(10);
        
        // 采用相同配置
        ADS1299_Write_Reg(0x01, 0x96); 
        ADS1299_Write_Reg(0x02, 0xC0); 
        ADS1299_Write_Reg(0x03, 0xF6); 
        ADS1299_Write_Reg(0x04, 0x00); 
        for(int i=1; i<=8; i++) ADS1299_Write_Reg(0x04 + i, 0x60); 
        ADS1299_Write_Reg(0x0D, 0xFF); 
        ADS1299_Write_Reg(0x0E, 0x00); 
        ADS1299_Write_Reg(0x15, 0x20);

        // 初始化变量
        for(int i=0; i<8; i++) cmrr_step2_sum_sq[i] = 0.0;
        
        ADS1299_Start();
        circular_buffer_clear(&ADS1299_circula_buffer);
        
        app_state = APP_CMRR_STEP2_RUNNING;
        start_time = HAL_GetTick();
        sample_count = 0;
        target_samples = 500; // 2 秒
        
        strcpy(response.message, "OK: STEP2 Started");
    }
    else {
        response.success = 0;
        strcpy(response.message, "ERR: Usage: TEST_CMRR_FULL:STEP1 or STEP2");
    }

    return response;
}

/**
 * @brief 处理 STATE 指令，获取并打印设备当前的综合状态
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 上报 SN 码、版本号、电池电量、USB/SD 状态以及 ADS1299/WM8978 初始化状态
 */
cmd_response_t cmd_get_state_handler(char* args) {
    cmd_response_t response = {1, ""};
    (void)args;

    // 在上报前更新最新的电池电压读取
    Update_Battery_Voltage();

    // 检查 ADS1299 ID
    uint8_t ads_id = ADS1299_ReadRegister(0x00);
    int ads_ok = ((ads_id & 0x1F) == 0x1E) ? 1 : 0; // ADS1299 ID 通常为 0x3E (rev+ID)

    // 检查 USB 状态
    int usb_ok = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1 : 0;

    extern char g_device_sn[];
    extern const char* g_device_version;

    USB_Print("\r\n=== DEVICE STATE REPORT ===\r\n");
    USB_Print("SN: %s\r\n", g_device_sn);
    USB_Print("VERSION:%s\r\n", g_device_version);
    USB_Print("Battery: %.1f%%\r\n", Battery_Voltage);
    USB_Print("USB:     %s\r\n", usb_ok ? "CONNECTED" : "DISCONNECTED");
    USB_Print("Charging:  %s\r\n", usb_ok ? "CHARGING" : "DISCHARGING");
    USB_Print("SD Card: %s\r\n", sd_mounted ? "MOUNTED" : "NOT MOUNTED");
    USB_Print("ADS1299: %s (ID: 0x%02X)\r\n", ads_ok ? "OK" : "ERROR", ads_id);
    USB_Print("WM8978:  %s\r\n", wm8978_initialized ? "OK" : "ERROR");
    
    USB_Print("=== END REPORT ===\r\n");

    strcpy(response.message, "OK: State report sent");
    return response;
}

/**
 * @brief 处理 PLAYMUSIC 指令，播放指定的音频文件
 * @param args 包含目标音频文件名的字符串 (可省略 .wav 后缀)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 仅在系统空闲时允许手动触发播放
 */
cmd_response_t cmd_playmusic_handler(char* args) {
    cmd_response_t response = {1, ""};
    if (args == NULL || args[0] == '\0') {
        response.success = 0;
        strcpy(response.message, "ERR: Missing filename");
        return response;
    }
    
    // 检查是否处于空闲状态
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }

    char path[128];
    // 消除前导空格
    while (*args == ' ') args++;
    
    USB_Print("[CMD] PLAYMUSIC args: '%s'\r\n", args);

    // 构建文件路径 (自动补全 .wav)
    if (strstr(args, ".wav") || strstr(args, ".WAV")) {
        sprintf(path, "AUDIO/%s", args);
    } else {
        sprintf(path, "AUDIO/%s.wav", args);
    }
    
    USB_Print("[CMD] Playing path: '%s'\r\n", path);

    wav_loop_mode = 0; // 仅播放一次
    
    uint8_t res = Wav_Play(path);
    if (res == 0) {
        sprintf(response.message, "OK: Playing %s", path);
    } else {
        response.success = 0;
        sprintf(response.message, "ERR: Play failed (%d)", res);
    }
    
    return response;
}

/**
 * @brief 处理 TEST_RX 指令，测试 USB 接收计数功能
 * @param args 包含指令参数 (START 或 STOP)
 * @retval cmd_response_t 包含执行结果的响应结构体
 */
cmd_response_t cmd_rx_test_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    // 消除前导空格
    while (*args == ' ') args++;
    
    if (strcmp(args, "START") == 0 || strcmp(args, "start") == 0) {
        g_rx_test_count = 0;
        g_rx_test_last_byte = 0;
        g_rx_test_mode = 1;
        sprintf(response.message, "RX Test Mode STARTED. Send 0x0303 packets now.");
    } 
    else if (strcmp(args, "STOP") == 0 || strcmp(args, "stop") == 0) {
        g_rx_test_mode = 0;
        sprintf(response.message, "RX Test Mode STOPPED. Count: %lu", g_rx_test_count);
    }
    else {
        // 仅报告状态
        sprintf(response.message, "RX Test Mode: %s, Count: %lu", 
                g_rx_test_mode ? "ON" : "OFF", g_rx_test_count);
    }
    
    return response;
}

/**
 * @brief 处理 RLD_TEST 指令，测试右腿驱动 (RLD) 抑制效果
 * @param args 指令参数 (未使用)
 * @retval cmd_response_t 包含执行结果的响应结构体
 * @note 将所有通道接入 RLD 偏置网络并运行 10 秒钟以测量本底交流 RMS 值
 */
cmd_response_t cmd_rld_test_handler(char* args) {
    cmd_response_t response = {1, ""};

    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    ADS1299_Stop();
    HAL_Delay(10);

    // 手动配置寄存器用于 RLD 性能测试
    ADS1299_Write_Reg(0x01, 0x96); // CONFIG1: 250 SPS
    ADS1299_Write_Reg(0x02, 0xC0); // CONFIG2: 内部测试信号开启
    ADS1299_Write_Reg(0x03, 0xEC); // CONFIG3: PD_REFBUF=1, VREF=4V, RLD_MEAS=1, RLDREF_INT=1, PD_RLD=1
    ADS1299_Write_Reg(0x04, 0x00); // LOFF: 禁用

    // CHnSET: 0x60 (增益 24，正常输入)
    for(int i=0; i<8; i++) {
        ADS1299_Write_Reg(0x05 + i, 0x60); 
    }

    ADS1299_Write_Reg(0x0D, 0xFF); // BIAS_SENSP: 所有通道均连接到 RLD
    ADS1299_Write_Reg(0x0E, 0xFF); // BIAS_SENSN: 所有通道均连接到 RLD
    ADS1299_Write_Reg(0x0F, 0x00); // LOFF_SENSP
    ADS1299_Write_Reg(0x10, 0x00); // LOFF_SENSN
    ADS1299_Write_Reg(0x11, 0x00); // LOFF_FLIP
    ADS1299_Write_Reg(0x12, 0x00); // LOFF_STATP (RO)
    ADS1299_Write_Reg(0x13, 0x00); // LOFF_STATN (RO)
    ADS1299_Write_Reg(0x14, 0x00); // GPIO
    ADS1299_Write_Reg(0x15, 0x20); // MISC1: SRB1
    ADS1299_Write_Reg(0x16, 0x00); // MISC2
    ADS1299_Write_Reg(0x17, 0x00); // CONFIG4

    USB_Print("[RLD_TEST] Registers Configured for RLD Test (10s).\r\n");
    
    // 读回以供验证
    USB_Print("[BOOT] ADS1299 Registers:\r\n");
    uint8_t regs[24];
    ADS1299_Read_Regs(0x00, regs, 24);
    for(int i=0; i<24; i++) {
        USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]);
        HAL_Delay(2);
    }

    // 初始化统计变量
    for(int i=0; i<8; i++) {
        rld_sum[i] = 0.0;
        rld_sum_sq[i] = 0.0;
    }

    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    app_state = APP_RLD_TEST_RUNNING;
    start_time = HAL_GetTick();
    sample_count = 0;
    target_samples = 10 * 250; // 10 秒 @ 250 SPS
    last_display_time = start_time;

    strcpy(response.message, "OK: RLD_TEST Started. Please wait 10s...");
    return response;
}

/**
 * @brief 注册所有的串口与 USB 指令
 * @note 初始化命令系统并将所有的指令字符串绑定到对应的处理函数
 */
void register_all_commands(void) {
  command_system_init();

  register_command("LIST", cmd_list_handler, "LIST", "List all .txt files");
  register_command("GET", cmd_get_handler, "GET:<filename>", "Send file");
  register_command("STATE", cmd_get_state_handler, "STATE", "Report device initialization state");
  register_command("DIAG", cmd_diag_handler, "DIAG", "SD card diagnosis");
  register_command("HELP", cmd_help_handler, "HELP", "Display command help");
  register_command("CREATELARGE", cmd_createlarge_handler, "CREATELARGE", "Create large test file");
  register_command("DELETE", cmd_delete_handler, "DELETE:<filename>", "Delete file");
  register_command("INFO", cmd_info_handler, "INFO:<filename>", "Display file information");
  register_command("STREAM", cmd_stream_handler, "STREAM", "8 channel stream (1 hour)");
  register_command("AUTO_STREAM", cmd_auto_stream_handler, "AUTO_STREAM:<filename>", "8 channel stream with P-pole lead-off monitoring");
  register_command("STOP", cmd_stop_handler, "STOP", "Stop acquisition");
  register_command("REG", cmd_reg_handler, "REG", "Read ADS1299 registers");
  register_command("TEST_IMP", cmd_test_imp_handler, "TEST_IMP", "Test Impedance Config");
  register_command("TEST_IMP_ALL", cmd_test_imp_all_handler, "TEST_IMP_ALL", "Test Impedance (8CH, 1s log)");
  register_command("TEST_NOISE", cmd_test_noise_handler, "TEST_NOISE", "Internal noise test");
  register_command("TEST_CMRR", cmd_test_cmrr_handler, "TEST_CMRR", "CMRR test");
  register_command("TEST_CMRR_FULL", cmd_test_cmrr_full_handler, "TEST_CMRR_FULL:<STEP1|STEP2>", "Full CMRR Test (Two Pass)");
  register_command("TEST_RX", cmd_rx_test_handler, "TEST_RX:<START|STOP>", "Test USB RX counting 0x0303");
	register_command("RLD_TEST", cmd_rld_test_handler, "RLD_TEST", "Test RLD suppression (10s)");
	register_command("PLAYMUSIC", cmd_playmusic_handler, "PLAYMUSIC:<filename>", "Play .wav file in AUDIO folder (e.g. PLAYMUSIC:1)");
	register_command("CONF", cmd_conf_handler, "CONF:<MASK_HEX>,<GAIN>,<SPS>", "Config STREAM (e.g. FF,24,250)");
	register_command("OTA_START", cmd_ota_start_handler, "OTA_START:<size>", "Start OTA transfer");
	register_command("OTA_APPLY", cmd_ota_apply_handler, "OTA_APPLY", "Apply update and reset");

    // USB_Print("[CMD] Command system: Registered %d commands\r\n", 15);
}
