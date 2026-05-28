/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "ADS1299.h"
#include "circular_buffer.h"
#include "soft_i2c.h"
#include "qmi8658_adapter.h"
#include "delay.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "command_system.h"
#include "commands_impl.h"
#include "binary_protocol.h" // Added for BP_CMD definitions
#include "wm8978.h"
#include "wavplay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_sdmmc1;
DMA_HandleTypeDef hdma_sai1_a;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SAI_HandleTypeDef hsai_BlockA1;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN PV */
CircularBuffer ADS1299_circula_buffer;
uint8_t sd_write_buf[512]; // Buffer for SD card writes
UINT bw;                   // Bytes written
FRESULT res;               // FatFs function common result code
uint8_t sd_mounted = 0;    // Flag for SD mount status
uint8_t file_opened = 0;   // Flag for file open status

float Battery_Voltage = 0.0f;
float batt_raw_vol = 0.0f; // Stores raw voltage in Volts for low battery check
uint8_t low_batt_cnt = 0;  // Counter for low battery debounce
float acc[3], gyro[3];
uint32_t last_qmi_print = 0;
uint8_t wm8978_initialized = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SAI1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */
void Send_EEG_Packet(uint8_t *raw_27_bytes);
void USB_Print(const char* format, ...);
void USB_SendString(const char* str);
void Power_Control_Init(void);
void Check_Power_Off(void);
void Update_Battery_Voltage(void);
void Low_Battery_Check(void);
void Send_AUX_Packet(float *acc, float *gyro, float vbat);
static void MX_IWDG_Init(void);
void BackupReg_Write(uint32_t index, uint32_t value);
uint32_t BackupReg_Read(uint32_t index);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char g_device_sn[24] = {0};
const char* g_device_version = "T1.0.3";

/**
  * @brief  生成动态设备序列号(SN)
  * @note   通过读取STM32芯片出厂自带的96位UID，进行哈希和取模运算，
  *         生成一个固定的5位数字尾号，并格式化到全局变量 g_device_sn 中。
  *         确保每台设备的SN唯一且每次开机保持不变。
  */
void Generate_Device_SN(void)
{
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();
    // 对96位UID进行简单的异或哈希计算
    uint32_t uid_hash = uid0 ^ uid1 ^ uid2;
    // 取模 100000 得到一个5位数字
    uint32_t sn_digits = uid_hash % 100000;
    
    snprintf(g_device_sn, sizeof(g_device_sn), "AIT2601X%05lu", sn_digits);
}

/**
  * @brief  EXTI 外部中断回调函数
  * @param  GPIO_Pin: 触发中断的GPIO引脚号
  * @note   主要用于捕获 ADS1299 的 DRDY (Data Ready) 数据就绪信号下降沿。
  *         当引脚匹配时，将调用底层数据处理函数读取脑电数据。
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == ADS1299_DRDY_Pin)
  {
    ADS1299_DRDY_Handler();
  }
}

/**
  * @brief  SD卡初始化与挂载处理流程
  * @note   该函数负责初始化FatFs文件系统、挂载SD卡，并执行一次基础的文件读写测试。
  *         同时，它还会检查并清理遗留的OTA升级文件(update.bin)，防止设备陷入无限升级循环。
  *         如果挂载失败，将通过音频系统播报设备错误提示音。
  */
void SD_Init_Process(void)
{
    // 1. 挂载文件系统
    USB_Print("[SD] Mounting FS... (ClockDiv: %d)\r\n", hsd1.Init.ClockDiv);
    // 尝试挂载最多3次
    for(int i=0; i<3; i++) {
        res = f_mount(&SDFatFS, SDPath, 1);
        if(res == FR_OK) break;
        USB_Print("[SD] Mount attempt %d failed: %d. Retrying...\r\n", i+1, res);
        HAL_Delay(100);
    }
    
    if(res != FR_OK)
    {
        USB_Print("[SD] Mount Failed! Error: %d\r\n", res);
        Play_System_Audio(16); // DEVICE_ERROR
        return;
    }
    USB_Print("[SD] Mount Success!\r\n");
    sd_mounted = 1;

    // 2. 测试写操作
    USB_Print("[SD] Testing Write to '2025.txt'...\r\n");
    res = f_open(&SDFile, "2025.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if(res != FR_OK) {
        USB_Print("[SD] Open for Write Failed! Error: %d\r\n", res);
        return;
    }
    
    char test_str[] = "20250113\r\n";
    UINT bytes_written;
    res = f_write(&SDFile, test_str, strlen(test_str), &bytes_written);
    if(res != FR_OK) {
        USB_Print("[SD] Write Failed! Error: %d\r\n", res);
        f_close(&SDFile);
        return;
    }
    USB_Print("[SD] Written %d bytes.\r\n", bytes_written);
    
    // 强制同步文件缓冲区到物理磁盘
    res = f_sync(&SDFile);
    if(res != FR_OK) {
        USB_Print("[SD] Sync Failed! Error: %d\r\n", res);
    }

    f_close(&SDFile);

    // 3. 测试读操作
    USB_Print("[SD] Testing Read from '2025.txt'...\r\n");
    res = f_open(&SDFile, "2025.txt", FA_READ);
    if(res != FR_OK) {
        USB_Print("[SD] Open for Read Failed! Error: %d\r\n", res);
        return;
    }
    
    char read_buf[64] = {0};
    UINT bytes_read;
    res = f_read(&SDFile, read_buf, sizeof(read_buf)-1, &bytes_read);
    if(res != FR_OK) {
        USB_Print("[SD] Read Failed! Error: %d\r\n", res);
        f_close(&SDFile);
        return;
    }
    USB_Print("[SD] Read Content: %s\r\n", read_buf);
    f_close(&SDFile);
    
    USB_Print("[SD] Test Complete.\r\n");
    
    // 4. 清理旧的OTA固件文件，防止开机陷入无限刷机循环
    FILINFO fno_ota;
    if (f_stat("update.bin", &fno_ota) == FR_OK) {
        USB_Print("[BOOT] Found old update.bin, deleting...\r\n");
        f_unlink("update.bin");
        USB_Print("[BOOT] update.bin deleted.\r\n");
    }
}

/**
  * @brief  板级支持包(BSP)初始化
  * @note   负责初始化系统基础电源控制引脚，并配置软件I2C（用于传感器通信）。
  *         该函数必须在其他外设初始化之前调用。
  */
void User_BSP_Init(void)
{
    // 1. 电源控制初始化 (必须最先执行，防止断电)
    Power_Control_Init();
    
    // 2. 软件I2C初始化 (目前暂无挂载的传感器，保留接口)
    Soft_I2C_Init();
}

/**
  * @brief  核心业务外设与传感器初始化
  * @note   按顺序初始化系统的核心硬件外设，包括：
  *         1. 环形缓冲区 (用于EEG数据中转)
  *         2. ADS1299 高精度脑电ADC芯片
  *         3. QMI8658 六轴姿态传感器
  *         4. SD卡与FatFs文件系统
  *         5. WM8978 音频编解码芯片 (配置为DAC耳机输出模式)
  *         若关键芯片初始化失败，将触发错误报警音。
  */
void User_Device_Init(void)
{
    // 1. 初始化环形缓冲区 (接收ADS1299数据必备)
    circular_buffer_init(&ADS1299_circula_buffer, CIRCULAR_BUFFER_MAX_SIZE);

    // 2. 初始化 ADS1299 脑电模拟前端
    USB_Print("[BOOT] Initializing ADS1299...\r\n");
    ADS1299_Init();
    uint8_t ads_id = ADS1299_ReadRegister(0x00);
    if ((ads_id & 0x1F) != 0x1E) {
        Play_System_Audio(16); // DEVICE_ERROR
    }
    
    // 3. 初始化 QMI8658 六轴姿态传感器
    USB_Print("[BOOT] Initializing QMI8658...\r\n");
    if (QMI8658_Init() == 0) {
        USB_Print("[BOOT] QMI8658 Init Success (ID=0x05)\r\n");
    } else {
        USB_Print("[BOOT] QMI8658 Init Failed!\r\n");
        Play_System_Audio(16); // DEVICE_ERROR
    }
    
    // 4. 初始化 SD卡 与 FatFs
    USB_Print("[BOOT] Initializing SD Card...\r\n");
    SD_Init_Process(); 

    // 5. 初始化 WM8978 音频编解码器
    USB_Print("[BOOT] Initializing WM8978...\r\n");
    if (WM8978_Init() == 0) {
        wm8978_initialized = 1;
        USB_Print("[BOOT] WM8978 Init Success!\r\n");
        
        // 为耳机输出配置基础DAC参数
        WM8978_ADDA_Cfg(1, 0);      // 开启 DAC, 关闭 ADC
        WM8978_Write_Reg(11, 0xFF); // 设置数字音量最大
        
        WM8978_Input_Cfg(0, 0, 0);  // 关闭所有输入通道
        WM8978_Output_Cfg(1, 0);    // 开启 DAC 输出
        WM8978_HPvol_Set(63, 63);   // 耳机音量设置为最大 (63)
        WM8978_SPKvol_Set(63);      // 喇叭音量设置为最大 (63)
    } else {
        wm8978_initialized = 0;
        USB_Print("[BOOT] WM8978 Init Failed!\r\n");
    }
}

/**
  * @brief  应用层逻辑初始化
  * @note   注册系统所有支持的命令行(CLI)与二进制指令，绑定对应的回调处理函数。
  */
void User_App_Init(void)
{
    // Register CLI Commands
    register_all_commands();
}
/* USER CODE END 0 */

/**
  * @brief  主程序入口 (Main Loop)
  * @retval int 程序的退出状态 (嵌入式系统中通常不会返回)
  * @note   系统启动的总控流程，包含底层硬件配置、看门狗初始化、热启动任务恢复逻辑，
  *         以及死循环中的设备状态轮询、USB指令解析、音频状态机刷新等。
  */
int main(void)
{

  /* USER CODE BEGIN 4 */
/* USER CODE END 4 */

  /* 芯片配置 --------------------------------------------------------*/

  /* 重置所有外设，初始化Flash接口和Systick。 */
  // 1. 关闭所有中断 (出于安全考虑，防止初始化过程中被打断)
  __disable_irq();

  // 3. 复位RCC (将时钟重置为默认状态)
  HAL_RCC_DeInit();
  
  // 3.1 强制设置中断向量表偏移 (这对于Bootloader跳转启动至关重要)
  SCB->VTOR = FLASH_BASE | 0x8000;

  // 4. 初始化HAL库 (配置SysTick滴答定时器等)
  HAL_Init();
  
  // CRITICAL: 立即喂狗 (防止看门狗超时复位)
  IWDG->KR = 0xAAAA;
  
  // CRITICAL: 立即锁定电源保持开启 (软件电源自锁机制)
  // 首先使能GPIOA时钟 (以防其未开启)
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = POWER_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_CTRL_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_SET);
  
  // 5. 重新开启全局中断
  __enable_irq();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* 配置系统时钟 */
  SystemClock_Config();

  /* 配置外设的公共时钟 */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  Delay_Init();
/* USER CODE END SysInit */

  /* 初始化所有已配置的外设 */
  MX_GPIO_Init();
  
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  // MX_I2C2_Init();
  MX_SAI1_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // 0. RGB LED 硬件引脚初始化
  RGB_LED_Init();

  // 1. BSP 板级基础初始化 (电源控制、软件I2C)
  User_BSP_Init();

  // 2. 核心业务外设与传感器初始化 (ADS1299, QMI8658, SD, Audio)
  User_Device_Init();
  
  // --- 开机仅执行一次：基于MCU UID生成唯一的动态SN码 ---
  Generate_Device_SN();
  // -------------------------------------------------

  // 3. 应用层初始化 (注册CLI命令行与协议缓存)
  User_App_Init();

  // 4. 热启动任务恢复逻辑 (应对死机复位后的无缝恢复)
  if (BackupReg_Read(BKP_REG_MAGIC) == BKP_MAGIC_NUMBER) {
      uint32_t last_state = BackupReg_Read(BKP_REG_TASK_STATE);
      uint32_t last_id = BackupReg_Read(BKP_REG_TASK_ID);
      
      if (last_state == 2) { // 2 = 任务正在运行 (RUNNING)
          extern volatile uint8_t g_task_state;
          extern uint32_t g_task_id;
          g_task_id = last_id;
          g_task_state = 2; // 立即将状态机切回运行态
          
          char filename[32];
          snprintf(filename, sizeof(filename), "T_%lu.BIN", g_task_id);
          
          extern volatile uint8_t g_use_binary_protocol;
          g_use_binary_protocol = 1;
          
          extern cmd_response_t cmd_stream_handler(char*);
          cmd_stream_handler(filename); // 恢复底层记录流
          USB_Print("[BOOT] Hot Start: Resuming Task %lu\r\n", g_task_id);
      }
  }

  // 5. 启动独立看门狗 (4.0秒超时限制)
  MX_IWDG_Init();
  USB_Print("[BOOT] Watchdog Enabled. System Ready.\r\n");
  
  // 检查是否通过SD卡标识文件禁用了设备
  {
      extern FIL SDFile;
      // 以防万一，先强制关闭可能已打开的文件句柄
      f_close(&SDFile); 
      if (f_open(&SDFile, "disabled.bin", FA_READ) == FR_OK) {
          f_close(&SDFile);
          extern volatile uint8_t g_task_state;
          g_task_state = 99; // 启动时立即将设备置为禁用状态 (99)
      }
  }
  
  USB_Print("\r\n=================================\r\n");
  USB_Print("   APP FIRMWARE VERSION: %s   \r\n", g_device_version);
  USB_Print("   DEVICE SN: %s              \r\n", g_device_sn);
  USB_Print("=================================\r\n");
  
  // 初始化完成后，播放开机欢迎语音 (DEVICE_BOOT, ID: 1)
  Play_System_Audio(1);
  
  /* USER CODE END 2 */

  /* 无限循环 */
  /* USER CODE BEGIN WHILE */
  
  // 初始化默认状态：开机默认蓝灯中速闪烁
  RGB_LED_SetSingle(LED_COLOR_BLUE, LED_MODE_SOLID_ON);
  
  while (1)
  {
    IWDG->KR = 0xAAAA; // 喂狗操作
    
    // --- 处理 RGB 三色灯的非阻塞状态机 ---
    RGB_LED_Process();
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 1. 处理通过USB接收到的串口命令
    if (usb_cmd_ready) {
        usb_cmd_ready = 0;
        process_command_ex(usb_cmd_buffer);
    }

    // 2. 处理OTA升级缓冲区 (将固件数据写入SD卡)
    extern void process_ota_data(void);
    process_ota_data();

    // 3. 处理挂起的二进制协议指令 (从USB中断中延期处理，避免在中断中耗时)
     extern volatile uint8_t g_pending_binary_cmd;
     extern char g_pending_filename[]; // 定义在 command_system.c
     extern uint32_t g_pending_ota_size;
     extern uint32_t g_pending_ota_crc; // 增加CRC校验

     if (g_pending_binary_cmd != 0) {
         uint8_t cmd = g_pending_binary_cmd;
         g_pending_binary_cmd = 0; // 清除标志位
        
        if (cmd == BP_CMD_START_STREAM) {
             extern volatile uint8_t g_use_binary_protocol;
             g_use_binary_protocol = 1;
             extern cmd_response_t cmd_stream_handler(char*);
             cmd_stream_handler(NULL);
        }
        else if (cmd == BP_CMD_GET_STATE) {
             // 构造状态包: DeviceType(1) QMI(1) SD(1) BAT(2) DeviceState(1) TaskID(4) Ver(N) + '\0' + SN(N)
             uint8_t status_payload[64];
             status_payload[0] = 1; // 设备类型: 1 = 小神探 (XiaoShenTan), 0 = 健脑师 (JianNaoShi)
             status_payload[1] = 1; // QMI 状态正常
             status_payload[2] = 1; // SD卡 状态正常
             uint16_t bat_mv = 415; // 4.15V (模拟值，应当替换为真实的ADC读取值)
             status_payload[3] = bat_mv & 0xFF;
             status_payload[4] = (bat_mv >> 8) & 0xFF;
             
             extern volatile uint8_t g_task_state;
             extern uint32_t g_task_id;
             extern volatile uint8_t g_usb_host_connected;
             static uint8_t last_usb_state = 0;
             
             uint8_t dev_state = 0;
             
             // 状态逻辑判定
             if (g_usb_host_connected == 0) {
                 dev_state = 4; // 未连接 (闲置的小神探端口)
                 last_usb_state = 0;
             } 
             else if (g_task_state == 1 || g_task_state == 2) {
                 dev_state = 3; // 进行中 (包括任务倒计时和进行中)
                 
                 // 如果在任务进行中刚刚建立连接，播放连接成功提示音
                 if (!last_usb_state) {
                     last_usb_state = 1;
                     Play_System_Audio(2);
                 }
             } 
             else if (g_task_state == 99) { // 假设99是一个特殊的“已停用”标志位
                 dev_state = 5; // 已停用
                 
                 // 如果刚刚建立连接，播放连接成功提示音
                 if (!last_usb_state) {
                     last_usb_state = 1;
                     Play_System_Audio(2);
                 }
             } 
             else {
                 // 已连接且处于空闲状态
                 if (bat_mv >= 360) { // 假设大于3.6V电量充足 (>60%，根据需要调整阈值)
                     dev_state = 1; // 已就绪 (已连接，电量>60%)
                 } else {
                     dev_state = 2; // 电量低 (已连接，电量<60%)
                 }
                 
                 // 如果刚刚建立连接，播放连接成功提示音
                 if (!last_usb_state) {
                     last_usb_state = 1;
                     Play_System_Audio(2);
                 }
             }
             
             status_payload[5] = dev_state;
             memcpy(&status_payload[6], &g_task_id, 4);
             
             const char* ver = g_device_version;
             const char* sn = g_device_sn;
             
             int offset = 10;
             strcpy((char*)&status_payload[offset], ver);
             offset += strlen(ver) + 1; // 包含 null 终止符作为分隔
             
             strcpy((char*)&status_payload[offset], sn);
             offset += strlen(sn); // 最后一个字符串不需要加 null 终止符
             
             uint8_t tx_buf[128];
             uint16_t tx_len = BP_Pack(tx_buf, BP_TYPE_STATUS, 0, status_payload, offset); // 注意：此处是异步发送，Seq固定为0
             CDC_Transmit_FS(tx_buf, tx_len);
             
             // 基于PB3电平跳变的音频输出切换逻辑
             uint8_t current_pb3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
             static uint8_t last_pb3_state = 0xFF; // 静态变量，跨函数调用保持状态
              if (current_pb3 != last_pb3_state) {
                  // 重新运行完整的初始化序列以确保信号路径完全恢复
                  if (WM8978_Init() == 0) {
                      WM8978_ADDA_Cfg(1, 0);      // 开启 DAC, 关闭 ADC
                      WM8978_Write_Reg(11, 0xFF); // 数字音量最大
                      WM8978_Input_Cfg(0, 0, 0);  // 关闭所有输入
                      WM8978_Output_Cfg(1, 0);    // 开启 DAC 输出
                      
                      if (current_pb3 == 0) { // 低电平 = 喇叭 (未插入耳机 或 硬件反相逻辑)
                           WM8978_HPvol_Set(0, 0);    // 静音耳机
                           WM8978_SPKvol_Set(63);     // 喇叭音量最大 (0-63 范围)
                       } else { // 高电平 = 耳机 (已插入耳机 或 硬件反相逻辑)
                           WM8978_HPvol_Set(63, 63); // 耳机音量最大 (0-63 范围)
                           WM8978_SPKvol_Set(0);      // 静音喇叭
                       }
                  }
                  last_pb3_state = current_pb3;
              }
        }
        else if (cmd == BP_CMD_STOP_STREAM) {
             extern cmd_response_t cmd_stop_handler(char*);
             cmd_stop_handler(NULL);
        }
        else if (cmd == BP_CMD_TEST_IMP) {
            // 阻抗测试指令 (0x0E)
            extern cmd_response_t cmd_test_imp_all_handler(char* args);
            cmd_response_t resp = cmd_test_imp_all_handler(NULL);
            BP_SendResponse(BP_CMD_TEST_IMP, resp.success ? BP_RESP_OK : BP_RESP_ERR, resp.message);
        }
        else if (cmd == BP_CMD_DISABLE_DEVICE) {
            // 在SD卡中写入一个 "disabled.bin" 停用标志文件
            extern FIL SDFile;
            // 强制关闭任何可能已经打开的文件，防止出现 FR_DENIED (6) 错误
            f_close(&SDFile);
            
            FRESULT fr = f_open(&SDFile, "disabled.bin", FA_CREATE_ALWAYS | FA_WRITE);
            if (fr == FR_OK) {
                uint8_t dummy = 1;
                UINT bw;
                f_write(&SDFile, &dummy, 1, &bw);
                // 确保数据落盘
                f_sync(&SDFile);
                f_close(&SDFile);
                extern volatile uint8_t g_task_state;
                g_task_state = 99; // 立即将设备置为已停用状态
                BP_SendResponse(BP_CMD_DISABLE_DEVICE, BP_RESP_OK, "Device Disabled");
            } else {
                // 将错误码格式化到字符串中以帮助调试
                char err_msg[32];
                snprintf(err_msg, sizeof(err_msg), "SD Write Failed (%d)", fr);
                BP_SendResponse(BP_CMD_DISABLE_DEVICE, BP_RESP_ERR, err_msg);
            }
        }
        else if (cmd == BP_CMD_ENABLE_DEVICE) {
            // 从SD卡中删除 "disabled.bin" 停用标志文件
            extern FIL SDFile;
            f_close(&SDFile); // 强制关闭句柄
            
            if (f_unlink("disabled.bin") == FR_OK) {
                extern volatile uint8_t g_task_state;
                if (g_task_state == 99) g_task_state = 0; // 清除停用状态
                BP_SendResponse(BP_CMD_ENABLE_DEVICE, BP_RESP_OK, "Device Enabled");
            } else {
                BP_SendResponse(BP_CMD_ENABLE_DEVICE, BP_RESP_ERR, "SD Unlink Failed or File Not Found");
            }
        }
        else if (cmd == BP_CMD_AUTO_STREAM) {
             extern volatile uint8_t g_use_binary_protocol;
             g_use_binary_protocol = 1;
             extern cmd_response_t cmd_auto_stream_handler(char*);
             // 如果没有参数，默认使用 "EEG_DATA.TXT"
             cmd_auto_stream_handler("EEG_DATA.TXT");
        }
        else if (cmd == BP_CMD_AUDIO_ROUTE) {
             extern volatile uint8_t g_audio_route;
             if (g_audio_route == 0) { // 喇叭和耳机同时输出
                 WM8978_SPKvol_Set(63);
                 WM8978_HPvol_Set(63, 63);
             } else if (g_audio_route == 1) { // 仅喇叭输出
                 WM8978_HPvol_Set(0, 0); // 静音耳机
                 WM8978_SPKvol_Set(63);  // 取消喇叭静音
             } else if (g_audio_route == 2) { // 仅耳机输出
                 WM8978_SPKvol_Set(0);   // 静音喇叭
                 WM8978_HPvol_Set(63, 63); // 取消耳机静音
             }
        }
        else if (cmd == BP_CMD_LIST_FILES) {
             // 列出文件
             DIR dir;
             FILINFO fno;
             FRESULT fr = f_opendir(&dir, "/");
             if (fr == FR_OK) {
                 char file_list[512] = {0}; // 最大缓冲区
                 int offset = 0;
                 for (;;) {
                     fr = f_readdir(&dir, &fno);
                     if (fr != FR_OK || fno.fname[0] == 0) break;
                     if (fno.fattrib & AM_DIR) continue; // 跳过目录
                     
                     // 格式化为: 文件名|大小;
                     int len = snprintf(file_list + offset, sizeof(file_list) - offset, "%s|%lu;", fno.fname, fno.fsize);
                     if (len > 0) offset += len;
                     if (offset >= sizeof(file_list) - 32) break; // 防止缓冲区溢出
                 }
                 f_closedir(&dir);
                 
                 // 发送包含文件列表的响应
                 uint8_t tx_buf[544];
                 uint8_t payload[512];
                 payload[0] = BP_CMD_LIST_FILES;
                 payload[1] = BP_RESP_OK;
                 if (offset > 508) offset = 508; // 安全上限
                 memcpy(&payload[2], file_list, offset);
                 
                 uint16_t tx_len = BP_Pack(tx_buf, BP_TYPE_CMD_RESP, 0, payload, 2 + offset);
                 CDC_Transmit_FS(tx_buf, tx_len);
             } else {
                 BP_SendResponse(BP_CMD_LIST_FILES, BP_RESP_ERR, "Open Dir Failed");
             }
        }
        else if (cmd == BP_CMD_DELETE_FILE) {
             FRESULT fr = f_unlink(g_pending_filename);
             if (fr == FR_OK) {
                 char msg[64];
                 snprintf(msg, sizeof(msg), "Deleted %s", g_pending_filename);
                 BP_SendResponse(BP_CMD_DELETE_FILE, BP_RESP_OK, msg);
             } else {
                 char msg[64];
                 snprintf(msg, sizeof(msg), "Del Fail %d", fr);
                 BP_SendResponse(BP_CMD_DELETE_FILE, BP_RESP_ERR, msg);
             }
        }
        else if (cmd == BP_CMD_CLEAR_DATA) {
             DIR dir;
             FILINFO fno;
             FRESULT fr;
             int deleted_count = 0;
             int deleted_in_pass;
             do {
                 deleted_in_pass = 0;
                 fr = f_opendir(&dir, "/");
                 if (fr == FR_OK) {
                     for (;;) {
                         fr = f_readdir(&dir, &fno);
                         if (fr != FR_OK || fno.fname[0] == 0) break;
                         if (fno.fattrib & AM_DIR) continue; // 跳过目录
                         
                         // 检查是否为目标文件 (.BIN 或 .TXT)
                         if (strstr(fno.fname, ".BIN") != NULL || strstr(fno.fname, ".TXT") != NULL ||
                             strstr(fno.fname, ".bin") != NULL || strstr(fno.fname, ".txt") != NULL) {
                             
                             // 排除受保护的文件
                              if (strcmp(fno.fname, "update.bin") != 0 && strcmp(fno.fname, "UPDATE.BIN") != 0 &&
                                  strcmp(fno.fname, "disabled.bin") != 0 && strcmp(fno.fname, "DISABLED.BIN") != 0) {
                                 
                                 // 为了FatFs的安全性，每次遍历目录只删除一个文件
                                 if (f_unlink(fno.fname) == FR_OK) {
                                     deleted_count++;
                                     deleted_in_pass++;
                                     break; // 打破 readdir 循环，重新执行 opendir
                                 }
                             }
                         }
                     }
                     f_closedir(&dir);
                 } else {
                     break; // opendir 失败
                 }
             } while (deleted_in_pass > 0);
             
             if (fr == FR_OK || deleted_count > 0) {
                 char msg[64];
                 snprintf(msg, sizeof(msg), "Clear Done: %d files", deleted_count);
                 BP_SendResponse(BP_CMD_CLEAR_DATA, BP_RESP_OK, msg);
             } else {
                 BP_SendResponse(BP_CMD_CLEAR_DATA, BP_RESP_ERR, "Clear Failed");
             }
        }
        else if (cmd == BP_CMD_PAUSE) {
             extern cmd_response_t cmd_pause_handler(char*);
             cmd_response_t resp = cmd_pause_handler(NULL);
             BP_SendResponse(BP_CMD_PAUSE, resp.success ? BP_RESP_OK : BP_RESP_ERR, resp.message);
        }
        else if (cmd == BP_CMD_RESUME) {
             extern cmd_response_t cmd_resume_handler(char*);
             cmd_response_t resp = cmd_resume_handler(NULL);
             BP_SendResponse(BP_CMD_RESUME, resp.success ? BP_RESP_OK : BP_RESP_ERR, resp.message);
        }
        else if (cmd == BP_CMD_STOP_IMP) {
             extern cmd_response_t cmd_stop_imp_handler(char*);
             cmd_response_t resp = cmd_stop_imp_handler(NULL);
             BP_SendResponse(BP_CMD_STOP_IMP, resp.success ? BP_RESP_OK : BP_RESP_ERR, resp.message);
        }
        else if (cmd == BP_CMD_GET_FILE) {
             extern FIL SDFile;
             FRESULT fr = f_open(&SDFile, g_pending_filename, FA_READ);
             if (fr == FR_OK) {
                 uint32_t total_size = f_size(&SDFile);
                 uint32_t sent_bytes = 0;
                 uint8_t read_buf[256]; // Chunk size
                 UINT br;
                 
                 // 发送起始包: 包含文件大小的 CMD_RESP
                 char start_msg[64];
                 snprintf(start_msg, sizeof(start_msg), "SIZE:%lu", total_size);
                 BP_SendResponse(BP_CMD_GET_FILE, BP_RESP_OK, start_msg);
                 
                 // 流式传输数据
                 uint16_t seq = 0;
                 while (sent_bytes < total_size) {
                     IWDG->KR = 0xAAAA; // 喂狗，防止大文件传输时看门狗复位
                     RGB_LED_Process(); // 保持传输期间LED状态机的响应
                     
                     fr = f_read(&SDFile, read_buf, sizeof(read_buf), &br);
                     if (fr != FR_OK || br == 0) break;
                     
                     // 发送数据包
                     // 暂时使用 BP_TYPE_OTA (0x20) 作为通用文件数据传输类型
                     uint8_t tx_buf[300];
                      uint16_t tx_len = BP_Pack(tx_buf, 0x20, seq++, read_buf, br); // 0x20 = OTA/文件类型
                      
                      uint8_t res = CDC_Transmit_FS(tx_buf, tx_len);
                      if (res == USBD_BUSY) {
                          HAL_Delay(2); // 稍作等待
                          res = CDC_Transmit_FS(tx_buf, tx_len);
                      }
                      
                      sent_bytes += br;
                 }
                 f_close(&SDFile);
                 
                 // 发送结束包
                 BP_SendResponse(BP_CMD_GET_FILE, BP_RESP_OK, "EOF");
                 
             } else {
                  BP_SendResponse(BP_CMD_GET_FILE, BP_RESP_ERR, "Open Fail");
              }
         }
     }
    
    // --- 恢复原有 Main 函数中的逻辑 ---
    // 处理音频播放状态机
    extern void Wav_Process(void);
    Wav_Process(); 
    
    // 检查关机按键 (硬件关机的重要逻辑!)
    extern void Check_Power_Off(void);
    Check_Power_Off(); 

    // 1Hz 频率的管家任务 (电池检测、IMU采集)
    if (HAL_GetTick() - last_qmi_print >= 1000) {
        last_qmi_print = HAL_GetTick();

        // 读取 IMU 姿态数据
        QMI8658_Read_XYZ(acc, gyro);

        // 更新电池电压
        Update_Battery_Voltage();

        // 检查低电量状态
        Low_Battery_Check();
    }
    
    // 任务倒计时检查逻辑
    extern volatile uint8_t g_task_state;
    if (g_task_state == 1) { // 1 = 任务等待中 (TASK_STATE_PENDING)
        extern uint32_t g_task_start_tick;
        extern uint32_t g_task_delay_ms;
        if (HAL_GetTick() - g_task_start_tick >= g_task_delay_ms) {
            g_task_state = 2; // 2 = 任务运行中 (TASK_STATE_RUNNING)
            
            // 将任务状态同步到备份寄存器，用于死机复位后恢复
            BackupReg_Write(BKP_REG_TASK_STATE, 2);
            BackupReg_Write(BKP_REG_TASK_ID, g_task_id);
            
            char filename[32];
            snprintf(filename, sizeof(filename), "T_%lu.BIN", g_task_id); // 文件后缀为 .BIN
            
            extern volatile uint8_t g_use_binary_protocol;
            g_use_binary_protocol = 1;
            
            // 复用 stream handler 来干净地启动数据采集
            extern cmd_response_t cmd_stream_handler(char*);
            cmd_stream_handler(filename);
        }
    }
    
    app_run_loop();
  }
  /* USER CODE END 3 */
  
  return 0; // 正常情况下永远不会执行到这里 
	
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK
                              |RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5; // Increased sampling time to reduce noise
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_I2S_STANDARD;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  /* FrameInit and SlotInit are not used for SAI_I2S_STANDARD protocol */
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 0; // 48MHz / (4+2) = 8MHz (Stable for polling)
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_1_Pin|ADS1299_RESET_Pin|ADS1299_START_Pin|ADS1299_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, G_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, POWER_CTRL_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_1_Pin ADS1299_RESET_Pin ADS1299_START_Pin ADS1299_CS_Pin
                           G_CS_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|ADS1299_RESET_Pin|ADS1299_START_Pin|ADS1299_CS_Pin
                          |G_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_CTRL_Pin */
  GPIO_InitStruct.Pin = POWER_CTRL_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_KEY_Pin */
  GPIO_InitStruct.Pin = POWER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(POWER_KEY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /*Configure GPIO pin : ADS1299_DRDY_Pin */
  GPIO_InitStruct.Pin = ADS1299_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADS1299_DRDY_GPIO_Port, &GPIO_InitStruct);

  /* PB10/PB11: 废弃的硬件引脚，配置为下拉输出防干扰 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure PB3 as Input (Disable JTAG JTDO, keep SWD) */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL; // 根据外部电路可改为 GPIO_PULLUP/DOWN
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Write to RTC Backup Register (0-31)
  */
void BackupReg_Write(uint32_t index, uint32_t value) {
    if (index > 31) return;
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    *(__IO uint32_t *)(&(RTC->BKP0R) + index) = value;
    HAL_PWR_DisableBkUpAccess();
}

/**
  * @brief  Read from RTC Backup Register (0-31)
  */
uint32_t BackupReg_Read(uint32_t index) {
    if (index > 31) return 0;
    return *(__IO uint32_t *)(&(RTC->BKP0R) + index);
}

/**
  * @brief  Initialize Independent Watchdog (IWDG)
  * @note   Timeout = (64 * 2000) / 32kHz = 4.0s
  */
static void MX_IWDG_Init(void) {
    // 1. Enable DBGMCU freeze during debug
    __HAL_DBGMCU_FREEZE_IWDG();

    // 2. Start IWDG first to ensure LSI is running
    IWDG->KR = 0xCCCC; 

    // 3. Configure IWDG
    IWDG->KR = 0x5555;    // Unlock PR, RLR
    IWDG->PR = 4;         // 64 Prescaler
    IWDG->RLR = 2000;     // 4s timeout
    
    // 4. Wait for sync with timeout protection
    uint32_t timeout = 0x10000;
    while (IWDG->SR != 0 && --timeout); 
    
    IWDG->KR = 0xAAAA;    // Reload
}

void Power_Control_Init(void) {
    // Hot-Start Check: If Magic Number exists in Backup Register, latch power immediately
    if (BackupReg_Read(BKP_REG_MAGIC) == BKP_MAGIC_NUMBER) {
        HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
        USB_Print("[PWR] Hot Start Detected. Power Latch Restored.\r\n");
        return;
    }

    // Normal Start: Check Power Key (PA1) - Active Low
    // Note: PA1 is configured as INPUT in MX_GPIO_Init
    
    if (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(20); // 20ms debounce
        if (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
            uint32_t press_start = HAL_GetTick();
            while (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
                if (HAL_GetTick() - press_start >= 2000) {
                    // Confirm press for 2 seconds -> Power ON
                    HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_SET); // Latch Power (PA4 High)
                    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); // LED On (Active Low) -> Moved to main loop for blinking
                    
                    // Mark as "Powered On" in Backup Register to allow Hot-Start recovery
                    BackupReg_Write(BKP_REG_MAGIC, BKP_MAGIC_NUMBER);
                    return;
                }
            }
        }
    }
    // If not pressed for 2 seconds, do not latch power.
    // Power will cut off when button is released.
}

void Check_Power_Off(void) {
    // Power Off Detection
    if (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(20); // 20ms debounce
        if (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
            uint32_t press_start = HAL_GetTick();
            
            // 按下按键时，蓝灯0.25s快闪
            RGB_LED_SetSingle(LED_COLOR_BLUE, LED_MODE_FLASH_FAST);
            
            while (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
                // 在这里需要继续调用 RGB_LED_Process，否则 while 循环会阻塞灯的翻转
                RGB_LED_Process();
                
                if (HAL_GetTick() - press_start >= 2000) {
                    // Confirm press for 2 seconds -> Power OFF
                    
                    // Play 14.wav (DEVICE_SHUTDOWN) before power off
                    Play_System_Audio(14);
                    
                    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // LED Off
                    
                    // Clear Magic Number before manual power off
                    BackupReg_Write(BKP_REG_MAGIC, 0);
                    
                    HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_RESET); // Unlatch Power (PA4 Low)
                    
                    extern volatile uint8_t g_usb_host_connected;
                    if (g_usb_host_connected == 1) {
                         NVIC_SystemReset();
                    } else {
                         HAL_Delay(2000);
                         NVIC_SystemReset(); 
                    }
                }
            }
            
            // 如果按键没有达到2秒就松开了，退出循环，取消闪烁，恢复为蓝色常亮
            RGB_LED_SetSingle(LED_COLOR_BLUE, LED_MODE_SOLID_ON);
        }
    }
}

void Update_Battery_Voltage(void) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint32_t raw = HAL_ADC_GetValue(&hadc1);
        
        // Use user-defined calibration formula
        // Updated formula based on 4.2V (Raw 4031) and 3.7V (Raw 3519) measurements
        float voltage = ((float)raw * 0.00097656f) + 0.2635f;
        
        batt_raw_vol = voltage; // Update raw voltage for low battery check
        
        // Convert to percentage: 3.6V = 0%, 4.1V = 100%
        float percentage = (voltage - 3.6f) / (4.1f - 3.6f) * 100.0f;
        
        if (percentage < 0.0f) percentage = 0.0f;
        if (percentage > 100.0f) percentage = 100.0f;
        
        // Store percentage in Battery_Voltage variable to be sent out
        Battery_Voltage = percentage;
    }
    HAL_ADC_Stop(&hadc1);
}

void Low_Battery_Check(void) {
    static uint32_t last_bat_warn = 0;
    // 假设20%电量阈值约为3.5V
    if (batt_raw_vol > 0 && batt_raw_vol < 3.5f) {
        low_batt_cnt++;
        if (low_batt_cnt > 5) { // 连续5次低电
            // 每隔5分钟(300000ms)报一次警
            if (HAL_GetTick() - last_bat_warn > 300000 || last_bat_warn == 0) {
                Play_System_Audio(15); // BATTERY_LOW
                last_bat_warn = HAL_GetTick();
            }
            
            // Turn on Red LED (Highest Priority)
            RGB_LED_SetSingle(LED_COLOR_RED, LED_MODE_FLASH_FAST);
            
            low_batt_cnt = 5; // 防止溢出
        }
    } else {
        low_batt_cnt = 0;
    }
}

void Send_AUX_Packet(float *acc, float *gyro, float vbat) {
    // Print AUX data as text
    /*
    USB_Print("AUX:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", 
              acc[0], acc[1], acc[2], 
              gyro[0], gyro[1], gyro[2], 
              vbat);
    */
}

void USB_SendString(const char* str) {
    if (str) {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
        if (hcdc == NULL) return;

        // Wait until USB is free
        uint32_t start = HAL_GetTick();
        while (hcdc->TxState != 0) {
            if (HAL_GetTick() - start > 100) return; // Timeout 100ms
        }
        CDC_Transmit_FS((uint8_t*)str, strlen(str));
    }
}


#include "usbd_cdc.h"

void USB_Print(const char* format, ...) {
    extern volatile uint8_t g_use_binary_protocol;
    // 如果处于二进制协议模式，直接丢弃纯文本打印，防止阻塞主循环和干扰二进制数据流
    if (g_use_binary_protocol) {
        return;
    }

    static char buffer[256]; // Use static buffer to ensure memory validity during DMA/Interrupt transfer
    
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc == NULL) return;

    // Wait until USB is free BEFORE writing to the shared static buffer
    // This prevents overwriting data that is currently being sent
    uint32_t start = HAL_GetTick();
    while (hcdc->TxState != 0) {
        if (HAL_GetTick() - start > 100) return; // Timeout 100ms
    }

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
}

void Send_EEG_Packet(uint8_t *raw_27_bytes) {
    float voltages[8];
    // Vref=4.5V, Gain=24. 
    // Scale factor to convert 24-bit integer to uV.
    // 4.5V = 4500000uV. 2^23 - 1 = 8388607.
    float scale = 4500000.0f / 8388607.0f / 24.0f; 

    for(int i=0; i<8; i++) {
        // raw_27_bytes: [Status(3), Ch1(3), Ch2(3)...]
        int ch_offset = 3 + i * 3;
        int32_t val = (raw_27_bytes[ch_offset] << 16) | (raw_27_bytes[ch_offset+1] << 8) | raw_27_bytes[ch_offset+2];
        
        // Sign extension for 24-bit number
        if (val & 0x800000) val |= 0xFF000000;
        
        voltages[i] = (float)val * scale;
    }
    
    // Print as CSV text, matching reference project style
    // Add "EEG:" prefix to distinguish from AUX data
    /*
    USB_Print("EEG:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", 
        voltages[0], voltages[1], voltages[2], voltages[3],
        voltages[4], voltages[5], voltages[6], voltages[7]);
    */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
