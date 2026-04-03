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
#include "ad5933.h"
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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == ADS1299_DRDY_Pin)
  {
    ADS1299_DRDY_Handler();
  }
}

void SD_Init_Process(void)
{
    // 1. Mount File System
    USB_Print("[SD] Mounting FS... (ClockDiv: %d)\r\n", hsd1.Init.ClockDiv);
    // Retry mount up to 3 times
    for(int i=0; i<3; i++) {
        res = f_mount(&SDFatFS, SDPath, 1);
        if(res == FR_OK) break;
        USB_Print("[SD] Mount attempt %d failed: %d. Retrying...\r\n", i+1, res);
        HAL_Delay(100);
    }
    
    if(res != FR_OK)
    {
        USB_Print("[SD] Mount Failed! Error: %d\r\n", res);
        return;
    }
    USB_Print("[SD] Mount Success!\r\n");
    sd_mounted = 1;

    // 2. Test Write
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
    
    // Explicitly sync the file buffer to physical disk
    res = f_sync(&SDFile);
    if(res != FR_OK) {
        USB_Print("[SD] Sync Failed! Error: %d\r\n", res);
    }

    f_close(&SDFile);

    // 3. Test Read
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
    
    // 4. Clean up OTA firmware file to prevent re-flashing loop
    FILINFO fno_ota;
    if (f_stat("update.bin", &fno_ota) == FR_OK) {
        USB_Print("[BOOT] Found old update.bin, deleting...\r\n");
        f_unlink("update.bin");
        USB_Print("[BOOT] update.bin deleted.\r\n");
    }

    // Prepare data.bin for logging (optional, if we were logging)
    // res = f_open(&SDFile, "data.bin", FA_OPEN_APPEND | FA_WRITE);
    // if(res == FR_OK) file_opened = 1;
}

void User_BSP_Init(void)
{
    // 1. Power Control (Must be first)
    Power_Control_Init();
    
    // 2. Soft I2C for Sensors
    Soft_I2C_Init();
}

void User_Device_Init(void)
{
    // 1. Initialize Circular Buffer (Required for ADS1299)
    circular_buffer_init(&ADS1299_circula_buffer, CIRCULAR_BUFFER_MAX_SIZE);

    // 2. ADS1299 EEG AFE Init
    USB_Print("[BOOT] Initializing ADS1299...\r\n");
    ADS1299_Init();
    
    // 3. QMI8658 IMU Init
    USB_Print("[BOOT] Initializing QMI8658...\r\n");
    if (QMI8658_Init() == 0) {
        USB_Print("[BOOT] QMI8658 Init Success (ID=0x05)\r\n");
    } else {
        USB_Print("[BOOT] QMI8658 Init Failed!\r\n");
    }
    
    // 4. SD Card Init//
    USB_Print("[BOOT] Initializing SD Card...\r\n");
    SD_Init_Process(); 

    // 5. AD5933 Impedance Converter Scan & Init
    USB_Print("[BOOT] Initializing AD5933 on PB10/PB11 (Soft I2C)...\r\n");
    AD5933_Init();

    // 6. WM8978 Audio Codec Init
    USB_Print("[BOOT] Initializing WM8978...\r\n");
    if (WM8978_Init() == 0) {
        wm8978_initialized = 1;
        USB_Print("[BOOT] WM8978 Init Success!\r\n");
        
        // Basic Config for DAC output (Headphones)
        WM8978_ADDA_Cfg(1, 0);      // Enable DAC, Disable ADC
        WM8978_Write_Reg(11, 0xFF); // Digital Volume Max
        
        // Speaker Boost Removed for Headphone Mode
        // uint16_t r49 = WM8978_Read_Reg(49);
        // WM8978_Write_Reg(49, r49 | (1 << 2)); 

        WM8978_Input_Cfg(0, 0, 0);  // Disable Inputs
        WM8978_Output_Cfg(1, 0);    // Enable DAC Output
        WM8978_HPvol_Set(63, 63);   // Headphone Volume (Max 63)
        WM8978_SPKvol_Set(63);      // Speaker Volume (Max 63)
    } else {
        wm8978_initialized = 0;
        USB_Print("[BOOT] WM8978 Init Failed!\r\n");
    }
}

void User_App_Init(void)
{
    // Register CLI Commands
    register_all_commands();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 4 */
/* USER CODE END 4 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  // 1. Disable Interrupts (Safety measure)
  __disable_irq();

  // 3. De-init RCC (Reset Clock to default)
  HAL_RCC_DeInit();
  
  // 3.1 Force Vector Table Offset (Critical for Bootloader Jump)
  SCB->VTOR = FLASH_BASE | 0x8000;

  // 4. Init HAL (Configures SysTick, etc.)
  HAL_Init();
  
  // CRITICAL: Kick Watchdog immediately
  IWDG->KR = 0xAAAA;
  
  // CRITICAL: Lock Power ON immediately (Software Power Latch)
  // Enable GPIOA Clock first (in case it was disabled)
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = POWER_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_CTRL_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_SET);
  
  // 5. Re-enable Interrupts
  __enable_irq();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  Delay_Init();
/* USER CODE END SysInit */

  /* Initialize all configured peripherals */
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
  // 1. BSP Init (Power, Soft I2C)
  User_BSP_Init();

  // 2. Device Init (ADS1299, QMI8658, AD5933)
  User_Device_Init();

  /* 
   * Note: ADS1299 Auto-start and WM8978 test code were removed 
   * during refactoring to keep main() clean.
   * See User_Device_Init() for sensor details.
   */

  // 3. App Init (CLI, Buffers)
  User_App_Init();
  
  // Check if device is disabled via SD card flag
  {
      extern FIL SDFile;
      // Close any potentially opened file first just in case
      f_close(&SDFile); 
      if (f_open(&SDFile, "disabled.bin", FA_READ) == FR_OK) {
          f_close(&SDFile);
          extern volatile uint8_t g_task_state;
          g_task_state = 99; // Set device to disabled state immediately on boot
      }
  }
  
  USB_Print("\r\n=================================\r\n");
  USB_Print("   APP FIRMWARE VERSION: T1.0.1   \r\n");
  USB_Print("=================================\r\n");
  
  // Play 1.wav after init
  Wav_Play("AUDIO/1.wav");
  while(is_playing) {
      Wav_Process();
  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t led_blink_tick = 0;
  
  while (1)
  {
    IWDG->KR = 0xAAAA; // Feed Dog
    
    // Heartbeat LED: Blink every 1 second (500ms ON, 500ms OFF)
//    if (HAL_GetTick() - led_blink_tick >= 500) 
//		{
//        HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
//        led_blink_tick = HAL_GetTick();
//    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (usb_cmd_ready) {
        usb_cmd_ready = 0;
        process_command_ex(usb_cmd_buffer);
    }

    // Process OTA Buffer (Write to SD Card)
    extern void process_ota_data(void);
    process_ota_data();

    // Check for pending Binary Protocol Commands (Deferred from USB Interrupt)
     extern volatile uint8_t g_pending_binary_cmd;
     extern char g_pending_filename[]; // Defined in command_system.c
     extern uint32_t g_pending_ota_size;
     extern uint32_t g_pending_ota_crc; // Added CRC

     if (g_pending_binary_cmd != 0) {
         uint8_t cmd = g_pending_binary_cmd;
         g_pending_binary_cmd = 0; // Clear flag
        
        if (cmd == BP_CMD_START_STREAM) {
             extern volatile uint8_t g_use_binary_protocol;
             g_use_binary_protocol = 1;
             extern cmd_response_t cmd_stream_handler(char*);
             cmd_stream_handler(NULL);
        }
        else if (cmd == BP_CMD_STOP_STREAM) {
             extern cmd_response_t cmd_stop_handler(char*);
             cmd_stop_handler(NULL);
        }
        else if (cmd == BP_CMD_TEST_IMP) {
            // Impedance Test command (0x0E)
            extern cmd_response_t cmd_test_imp_all_handler(char* args);
            cmd_response_t resp = cmd_test_imp_all_handler(NULL);
            BP_SendResponse(BP_CMD_TEST_IMP, resp.success ? BP_RESP_OK : BP_RESP_ERR, resp.message);
        }
        else if (cmd == BP_CMD_DISABLE_DEVICE) {
            // Write a "disabled.bin" file to SD card
            extern FIL SDFile;
            // Force close any existing open file to prevent FR_DENIED (6)
            f_close(&SDFile);
            
            FRESULT fr = f_open(&SDFile, "disabled.bin", FA_CREATE_ALWAYS | FA_WRITE);
            if (fr == FR_OK) {
                uint8_t dummy = 1;
                UINT bw;
                f_write(&SDFile, &dummy, 1, &bw);
                // Ensure data is written to disk
                f_sync(&SDFile);
                f_close(&SDFile);
                extern volatile uint8_t g_task_state;
                g_task_state = 99; // Set device to disabled state immediately
                BP_SendResponse(BP_CMD_DISABLE_DEVICE, BP_RESP_OK, "Device Disabled");
            } else {
                // Formatting the error code into the string to help debugging
                char err_msg[32];
                snprintf(err_msg, sizeof(err_msg), "SD Write Failed (%d)", fr);
                BP_SendResponse(BP_CMD_DISABLE_DEVICE, BP_RESP_ERR, err_msg);
            }
        }
        else if (cmd == BP_CMD_ENABLE_DEVICE) {
            // Delete the "disabled.bin" file from SD card
            extern FIL SDFile;
            f_close(&SDFile); // Force close
            
            if (f_unlink("disabled.bin") == FR_OK) {
                extern volatile uint8_t g_task_state;
                if (g_task_state == 99) g_task_state = 0; // Clear disabled state
                BP_SendResponse(BP_CMD_ENABLE_DEVICE, BP_RESP_OK, "Device Enabled");
            } else {
                BP_SendResponse(BP_CMD_ENABLE_DEVICE, BP_RESP_ERR, "SD Unlink Failed or File Not Found");
            }
        }
        else if (cmd == BP_CMD_AUTO_STREAM) {
             extern volatile uint8_t g_use_binary_protocol;
             g_use_binary_protocol = 1;
             extern cmd_response_t cmd_auto_stream_handler(char*);
             // Default to "EEG_DATA.TXT" if no args
             cmd_auto_stream_handler("EEG_DATA.TXT");
        }
        else if (cmd == BP_CMD_AUDIO_ROUTE) {
             extern volatile uint8_t g_audio_route;
             if (g_audio_route == 0) { // Both
                 WM8978_SPKvol_Set(63);
                 WM8978_HPvol_Set(63, 63);
             } else if (g_audio_route == 1) { // Speaker only
                 WM8978_HPvol_Set(0, 0); // Mute HP
                 WM8978_SPKvol_Set(63);  // Unmute SPK
             } else if (g_audio_route == 2) { // Headphone only
                 WM8978_SPKvol_Set(0);   // Mute SPK
                 WM8978_HPvol_Set(63, 63); // Unmute HP
             }
        }
        else if (cmd == BP_CMD_LIST_FILES) {
             // List files
             DIR dir;
             FILINFO fno;
             FRESULT fr = f_opendir(&dir, "/");
             if (fr == FR_OK) {
                 char file_list[512] = {0}; // Max buffer
                 int offset = 0;
                 for (;;) {
                     fr = f_readdir(&dir, &fno);
                     if (fr != FR_OK || fno.fname[0] == 0) break;
                     if (fno.fattrib & AM_DIR) continue; // Skip directories
                     
                     // Format: Name|Size;
                     int len = snprintf(file_list + offset, sizeof(file_list) - offset, "%s|%lu;", fno.fname, fno.fsize);
                     if (len > 0) offset += len;
                     if (offset >= sizeof(file_list) - 32) break; // Avoid overflow
                 }
                 f_closedir(&dir);
                 
                 // Send Response with File List
                 uint8_t tx_buf[544];
                 uint8_t payload[512];
                 payload[0] = BP_CMD_LIST_FILES;
                 payload[1] = BP_RESP_OK;
                 if (offset > 508) offset = 508; // Safe cap
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
                         if (fno.fattrib & AM_DIR) continue; // Skip directories
                         
                         // Check if it's a target file (.BIN or .TXT)
                         if (strstr(fno.fname, ".BIN") != NULL || strstr(fno.fname, ".TXT") != NULL ||
                             strstr(fno.fname, ".bin") != NULL || strstr(fno.fname, ".txt") != NULL) {
                             
                             // Exclude protected files
                              if (strcmp(fno.fname, "update.bin") != 0 && strcmp(fno.fname, "UPDATE.BIN") != 0 &&
                                  strcmp(fno.fname, "disabled.bin") != 0 && strcmp(fno.fname, "DISABLED.BIN") != 0) {
                                 
                                 // We only delete one file per directory pass to be safe with FatFs
                                 if (f_unlink(fno.fname) == FR_OK) {
                                     deleted_count++;
                                     deleted_in_pass++;
                                     break; // Break the readdir loop, restart opendir
                                 }
                             }
                         }
                     }
                     f_closedir(&dir);
                 } else {
                     break; // opendir failed
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
        else if (cmd == BP_CMD_GET_FILE) {
             extern FIL SDFile;
             FRESULT fr = f_open(&SDFile, g_pending_filename, FA_READ);
             if (fr == FR_OK) {
                 uint32_t total_size = f_size(&SDFile);
                 uint32_t sent_bytes = 0;
                 uint8_t read_buf[256]; // Chunk size
                 UINT br;
                 
                 // Send Start Packet: CMD_RESP with size
                 char start_msg[64];
                 snprintf(start_msg, sizeof(start_msg), "SIZE:%lu", total_size);
                 BP_SendResponse(BP_CMD_GET_FILE, BP_RESP_OK, start_msg);
                 
                 // Stream Data
                 uint16_t seq = 0;
                 while (sent_bytes < total_size) {
                     fr = f_read(&SDFile, read_buf, sizeof(read_buf), &br);
                     if (fr != FR_OK || br == 0) break;
                     
                     // Send Data Packet (Type = BP_TYPE_FILE = 0x20 used for OTA, maybe reuse or new type?)
                     // Let's use BP_TYPE_CMD_RESP with a special status or just raw payload?
                     // Better: Use a dedicated Type for File Data to avoid confusion.
                     // Let's use BP_TYPE_OTA (0x20) as "File Transfer" type for now, or add BP_TYPE_FILE_DATA
                     // Actually, host expects CMD_RESP for handshake, then raw data?
                     // Let's pack it as BP_TYPE_CMD_RESP with a specific structure? No, overhead.
                     // Use BP_TYPE_OTA (0x20) for generic file data.
                     
                     uint8_t tx_buf[300];
                      uint16_t tx_len = BP_Pack(tx_buf, 0x20, seq++, read_buf, br); // 0x20 = OTA/File Type
                      
                      uint8_t res = CDC_Transmit_FS(tx_buf, tx_len);
                      if (res == USBD_BUSY) {
                          HAL_Delay(2); // Wait a bit
                          res = CDC_Transmit_FS(tx_buf, tx_len);
                      }
                      
                      sent_bytes += br;
                     // Flow control delay?
                     // HAL_Delay(1); 
                 }
                 f_close(&SDFile);
                 
                 // Send End Packet
                 BP_SendResponse(BP_CMD_GET_FILE, BP_RESP_OK, "EOF");
                 
             } else {
                  BP_SendResponse(BP_CMD_GET_FILE, BP_RESP_ERR, "Open Fail");
              }
         }
         /* 
         // ====== BINARY OTA LOGIC COMMENTED OUT TO PREVENT WHILE(1) HAZARDS ======
         // The system currently relies on ASCII text-based OTA via updata_helper.
         else if (cmd == BP_CMD_OTA_START) {
             // Reuse existing logic, but adapt for binary
             extern cmd_response_t cmd_ota_start_handler(char*);
             char size_str[32];
             snprintf(size_str, sizeof(size_str), "%lu", g_pending_ota_size);
             cmd_ota_start_handler(size_str);
             // Response sent by handler or we can send a dedicated one
             BP_SendResponse(BP_CMD_OTA_START, BP_RESP_OK, "OTA Ready");
         }
         else if (cmd == BP_CMD_OTA_APPLY) {
             extern volatile uint8_t g_ota_complete_flag;
             extern uint32_t calculate_file_crc32(const char* filename); // Need to implement or use STM32 CRC
             
             if (g_ota_complete_flag) {
                 // Verify CRC if provided
                 if (g_pending_ota_crc != 0) {
                     uint32_t calc_crc = calculate_file_crc32("update.bin");
                     if (calc_crc != g_pending_ota_crc) {
                         BP_SendResponse(BP_CMD_OTA_APPLY, BP_RESP_ERR, "CRC Mismatch! Aborting.");
                         f_unlink("update.bin"); // Delete bad file
                         continue; // Don't reset, skip to next loop iteration
                     }
                 }
                 
                 BP_SendResponse(BP_CMD_OTA_APPLY, BP_RESP_OK, "CRC OK. Shutting down...");
                 HAL_Delay(500);
                 
                 // Play shutdown sound
                 Wav_Play("AUDIO/2.wav");
                 while(is_playing) {
                     Wav_Process();
                 }
                 
                 // Shutdown
                 HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_RESET);
                 
                 extern volatile uint8_t g_usb_host_connected;
                 if (g_usb_host_connected == 1) {
                      NVIC_SystemReset();
                 } else {
                      while(1);
                 }
             } else {
                 BP_SendResponse(BP_CMD_OTA_APPLY, BP_RESP_ERR, "Transfer Incomplete");
             }
         }
         // =========================================================================
         */
     }
    
    // --- Restored Logic from Original Main ---
    // Handle Audio (if needed in loop context, though app_run_loop handles it)
    extern void Wav_Process(void);
    Wav_Process(); 
    
    // Handle Power Off Check
    extern void Check_Power_Off(void);
    Check_Power_Off(); // Important for hardware shutdown!

    // 1Hz Loop for Housekeeping (Battery, IMU)
    if (HAL_GetTick() - last_qmi_print >= 1000) {
        last_qmi_print = HAL_GetTick();

        // Read IMU
        QMI8658_Read_XYZ(acc, gyro);

        // Read Battery
        Update_Battery_Voltage();

        // Check Low Battery (New)
        Low_Battery_Check();
    }
    
    // Task Countdown Checking
    extern volatile uint8_t g_task_state;
    if (g_task_state == 1) { // TASK_STATE_PENDING
        extern uint32_t g_task_start_tick;
        extern uint32_t g_task_delay_ms;
        if (HAL_GetTick() - g_task_start_tick >= g_task_delay_ms) {
            g_task_state = 2; // TASK_STATE_RUNNING
            extern uint32_t g_task_id;
            char filename[32];
            snprintf(filename, sizeof(filename), "T_%lu.BIN", g_task_id); // Change extension to .BIN
            
            extern volatile uint8_t g_use_binary_protocol;
            g_use_binary_protocol = 1;
            
            // Re-use stream handler to start collection cleanly
            extern cmd_response_t cmd_stream_handler(char*);
            cmd_stream_handler(filename);
            
            // We can also send an async notification if we want, but it's optional.
        }
    }
    
    app_run_loop();
  }
  /* USER CODE END 3 */
  
  return 0; // Should never reach here
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

  /* SCL (PB10) 三轴IIC */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SDA (PB11) 三轴IIC */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // 
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Power_Control_Init(void) {
    // Check Power Key (PA1) - Active Low
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
            while (HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin) == GPIO_PIN_RESET) {
                if (HAL_GetTick() - press_start >= 2000) {
                    // Confirm press for 2 seconds -> Power OFF
                    
                    // Play 2.wav before power off
                    Wav_Play("AUDIO/2.wav");
                    while(is_playing) {
                        Wav_Process();
                    }
                    
                    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // LED Off
                    HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_RESET); // Unlatch Power (PA4 Low)
                    
                    extern volatile uint8_t g_usb_host_connected;
                    if (g_usb_host_connected == 1) {
                         NVIC_SystemReset();
                    } else {
											   HAL_Delay(2000);
                         NVIC_SystemReset(); 
											//while(1); // Wait for power cut
                    }
                }
            }
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
    // Check every second (called in main loop along with Update_Battery_Voltage)
    // Threshold: 3.5V
    if (batt_raw_vol < 3.5f && batt_raw_vol > 1.0f) { // > 1.0f to avoid triggering on 0V (e.g. startup)
        low_batt_cnt++;
        if (low_batt_cnt >= 5) { // 5 seconds continuous low
            USB_Print("[PWR] Low Battery (%.2fV). Shutting down...\r\n", batt_raw_vol);
            
            // Play Shutdown Sound
            Wav_Play("AUDIO/2.wav");
            while(is_playing) {
                Wav_Process();
            }
            
            // Perform Shutdown Sequence
            ADS1299_Stop();
            f_close(&SDFile);
            
            // Blink LED rapidly
            for(int i=0; i<10; i++) {
                HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
                HAL_Delay(100);
            }
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); // LED Off
            
            // Cut Power
            HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_RESET);
            
            // If USB is connected, it might provide power and prevent shutdown.
            // Avoid getting trapped in while(1) in that case.
            extern volatile uint8_t g_usb_host_connected;
            if (g_usb_host_connected == 1) {
                // If connected to USB, do not enter while(1). 
                // Just reset low_batt_cnt so it can try again later or let the user know.
                // Or soft reset. Soft reset is safer to clear states.
                NVIC_SystemReset();
            } else {
                // If not connected to USB host, we still might be connected to a dumb charger.
                // Instead of while(1) which causes freezing and heating, we perform a soft reset
                // after a short delay if power hasn't been cut yet.
                // This ensures the device either dies gracefully or reboots cleanly if externally powered.
                HAL_Delay(2000); 
                NVIC_SystemReset();
            }
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
