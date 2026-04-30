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
#include "ad5933.h"
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

// External functions/variables from main.c
extern float Battery_Voltage;
extern void Update_Battery_Voltage(void);
extern uint8_t wm8978_initialized;

// Global Configuration Variables
uint8_t g_cfg_gain = 24;
uint8_t g_cfg_ch_mask = 0xFF;
int g_cfg_sps = 250;
uint8_t g_cfg_input_type = 0; // 0=Normal, 1=Short, 5=TestSig
uint8_t g_cfg_bias_en = 1;
uint32_t g_cfg_duration_sec = 0; // 0 = Infinite
uint8_t g_cfg_test_sig = 0; // 0=External, 1=Internal

// OTA Global Variables
volatile uint8_t g_file_transfer_mode = 0;
static uint32_t ota_total_bytes = 0;
static uint32_t ota_received_bytes = 0;
static FIL ota_fil;
static uint32_t ota_start_tick = 0;
volatile uint8_t stream_bad_ch_history = 0; // Moved to top-level scope for visibility
volatile uint8_t g_ota_complete_flag = 0;   // Flag to signal main loop to finish OTA

// Task Global Variables
volatile uint8_t g_task_state = 0; // 0=NONE, 1=PENDING, 2=RUNNING, 3=COMPLETED
uint32_t g_task_id = 0;
uint32_t g_task_start_tick = 0;
uint32_t g_task_delay_ms = 0;

// Static Buffers for Binary Protocol (Prevent Stack Overflow)
static uint8_t g_bp_tx_buf[256];
static uint8_t g_bp_payload[128];

// Define Binary File Header and Frame Structures
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

// Simple Soft CRC32 implementation (Poly 0x04C11DB7)
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

uint32_t calculate_file_crc32(const char* filename) {
    extern FIL SDFile;
    if (f_open(&SDFile, filename, FA_READ) != FR_OK) return 0;
    
    uint32_t crc = 0xFFFFFFFF;
    uint8_t buf[512];
    UINT br;
    
    // Using STM32 CRC logic but software for now to be safe
    // Note: Standard CRC32 usually inits with 0xFFFFFFFF and inverts at end
    // But STM32 Hardware CRC uses 0xFFFFFFFF init and NO inversion by default.
    // Let's stick to standard Ethernet CRC32 for compatibility with Python zlib.crc32
    // Python zlib.crc32 is standard.
    // Let's implement standard CRC32 table-less for size
    
    while (1) {
        f_read(&SDFile, buf, sizeof(buf), &br);
        if (br == 0) break;
        
        // Update CRC
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

// OTA Buffer System (Decouple USB Interrupt from FatFS)
#define OTA_BUFFER_SIZE 4096 // 4KB Buffer (Double Buffer Concept: Fill one, Write one?)
// Actually a Ring Buffer is better.
// Or just a simple "Ping-Pong" buffer.
// Let's use a simple large buffer. USB packets are 64 bytes. 
// If main loop is slow (SD write takes 10ms), USB might overflow if we don't buffer enough.
// 4KB is enough for ~64 USB packets. At 1MB/s, 4KB fills in 4ms.
// SD write of 4KB might take 10-20ms. So we might need flow control.

static uint8_t ota_rx_buffer[OTA_BUFFER_SIZE];
static volatile uint16_t ota_buf_head = 0;
static volatile uint16_t ota_buf_tail = 0;
static volatile uint8_t ota_buf_overflow = 0;

// Called from USB Interrupt (CDC_Receive_FS -> handle_file_transfer)
void handle_file_transfer(uint8_t* buf, uint32_t len) {
    if (!g_file_transfer_mode) return;
    
    // Copy data to ring buffer
    for (uint32_t i = 0; i < len; i++) {
        uint16_t next_head = (ota_buf_head + 1) % OTA_BUFFER_SIZE;
        if (next_head != ota_buf_tail) {
            ota_rx_buffer[ota_buf_head] = buf[i];
            ota_buf_head = next_head;
        } else {
            ota_buf_overflow = 1; // Buffer Full! Drop data :(
            // Ideally we should NAK or flow control here.
        }
    }
}

// Called from Main Loop
void process_ota_data(void) {
    IWDG->KR = 0xAAAA; // Feed Dog during SD writes
    if (!g_file_transfer_mode) return;
    
    // Check overflow
    if (ota_buf_overflow) {
        g_file_transfer_mode = 0;
        f_close(&ota_fil);
        USB_Print("\r\n[OTA] Error: Buffer Overflow. Main loop too slow.\r\n");
        return;
    }

    // Check if enough data to write (e.g., 512 bytes or whatever is available)
    uint16_t head = ota_buf_head; // Snapshot
    uint16_t tail = ota_buf_tail;
    
    if (head == tail) return; // Empty
    
    // Calculate contiguous available data
    uint16_t count;
    if (head >= tail) {
        count = head - tail;
    } else {
        count = OTA_BUFFER_SIZE - tail; // Read until end first
    }
    
    // Cap write size to 512 for efficiency (or larger)
    // if (count > 512) count = 512; 
    
    UINT bw;
    FRESULT fr = f_write(&ota_fil, &ota_rx_buffer[tail], count, &bw);
    
    if (fr != FR_OK) {
        g_file_transfer_mode = 0;
        f_close(&ota_fil);
        USB_Print("\r\n[OTA] Error: Write failed (%d).\r\n", fr);
        return;
    }
    
    // Advance tail
    ota_buf_tail = (tail + count) % OTA_BUFFER_SIZE;
    
    ota_received_bytes += bw;
    
    // Check completion
    if (ota_received_bytes >= ota_total_bytes) {
         g_file_transfer_mode = 0;
         g_ota_complete_flag = 1;
         
         // Note: File closing and "Transfer Complete" message are handled 
         // in app_run_loop() to prevent double-closing the file (FatFS Error 9).
    } else {
        // Send Text Progress every ~10% or 64KB to avoid spamming the host
        if (ota_received_bytes % 65536 == 0) {
            USB_Print("[PROG] %lu / %lu\r\n", ota_received_bytes, ota_total_bytes);
        }
    }
}

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
    
    // Parse size
    uint32_t size = strtoul(args, NULL, 0);
    if (size == 0) {
        response.success = 0;
        strcpy(response.message, "ERR: Invalid size");
        return response;
    }
    
    // Open file
    FRESULT fr = f_open(&ota_fil, "update.bin", FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        response.success = 0;
        sprintf(response.message, "ERR: Failed to create update.bin (%d)", fr);
        return response;
    }
    
    // Pre-allocate if possible? FatFS f_expand? Not standard.
    // Just set variables
    ota_total_bytes = size;
    ota_received_bytes = 0;
    ota_start_tick = HAL_GetTick();
    g_file_transfer_mode = 1;
    
    // Reset Ring Buffer
    ota_buf_head = 0;
    ota_buf_tail = 0;
    ota_buf_overflow = 0;
    
    USB_Print("[OTA] Ready to receive %lu bytes. Send binary data now...\r\n", size);
    
    strcpy(response.message, "OK: OTA Transfer Started");
    return response;
}

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
    HAL_Delay(500); // Wait for message to be sent
    
    // Play shutdown sound to indicate OTA ready
    Wav_Play("AUDIO/2.wav");
    while(is_playing) {
        Wav_Process();
    }
    
    // Perform Shutdown Sequence (Matches old project behavior)
    // Pull POWER_CTRL Low (PA4)
    HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_RESET);
    
    // Wait for power cut
    extern volatile uint8_t g_usb_host_connected;
    if (g_usb_host_connected == 1) {
         NVIC_SystemReset();
    } else {
         while(1);
    }
    
    // Should not reach here
    strcpy(response.message, "OK: Shutting down...");
    return response;
}

// Local file system objects
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

// Impedance Calculation Variables
static float imp_max_val = -1e9;
static float imp_min_val = 1e9;
static int imp_window_count = 0;
static const int IMP_WINDOW_SIZE = 50; // 0.2s @ 250Hz (approx 1.5 cycles of 7.8Hz)

// Noise Test Variables
static double noise_sum[8];
static double noise_sum_sq[8];
static float noise_min[8];
static float noise_max[8];

// CMRR Test Variables
static double cmrr_sum_sq[8];
static const float CMRR_VIN_RMS = 0.7071f; // Assuming 2Vpp input = 0.7071 Vrms

// CMRR Full Test Variables
static double cmrr_step1_sum_sq[8]; // For Differential Gain
static double cmrr_step2_sum_sq[8]; // For Common Mode Gain
static double cmrr_diff_gain[8];    // Calculated Differential Gain

// Impedance Test Statistics
static double imp_sum = 0.0;
static uint32_t imp_count = 0;

static double rld_sum[8];
static double rld_sum_sq[8];

// Impedance Test All Channels
static float imp_all_max[8];
static float imp_all_min[8];

// --- Global Configuration for STREAM ---
// static uint8_t g_cfg_ch_mask = 0xFF; // Moved to global
// static uint8_t g_cfg_gain = 24;      // Moved to global
// static int     g_cfg_sps = 250;      // Moved to global
// static uint8_t g_cfg_input_type = 0; // Moved to global
// static uint8_t g_cfg_bias_en = 1;    // Moved to global
static uint8_t current_bias_sensp_mask = 0; // Runtime BIAS mask for AUTO_STREAM

/* Helper to convert Gain value to Register Bits */
static uint8_t get_gain_code(uint8_t gain) {
    switch(gain) {
        case 1:  return 0x00;
        case 2:  return 0x10;
        case 4:  return 0x20;
        case 6:  return 0x30;
        case 8:  return 0x40;
        case 12: return 0x50; 
        case 24: return 0x60; 
        default: return 0x60; // Default to 24
    }
}

/* Helper to check text file */
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

/* Helper to display progress */
static void display_file_progress(uint32_t current, uint32_t total) {
  if (total > 0) {
    float percent = (current * 100.0f) / total;
    USB_Print("[PROG] %lu/%lu bytes (%.1f%%)\r\n",
           (unsigned long)current,
           (unsigned long)total,
           percent);
  }
}

static int usb_wait_tx_idle(uint32_t timeout_ms) {
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc == NULL) return 0;
  uint32_t start = HAL_GetTick();
  while (hcdc->TxState != 0) {
    if (HAL_GetTick() - start > timeout_ms) return 0;
  }
  return 1;
}

/* Helper for checking filesystem */
static void check_filesystem_impl(void) {
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
  }
}

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

cmd_response_t cmd_diag_handler(char* args) {
  cmd_response_t response = {1, ""};
  (void)args;
  USB_SendString("\r\n=== SD Card Diagnosis ===\r\n");

  // 1. Check Physical Presence
  uint8_t is_detected = BSP_SD_IsDetected();
  USB_Print("SD Card Physical Status: %s\r\n", is_detected == SD_PRESENT ? "DETECTED" : "NOT DETECTED");

  // 2. Check Mount Flag
  USB_Print("File System Mount Flag: %s\r\n", sd_mounted ? "MOUNTED" : "NOT MOUNTED");

  if (is_detected != SD_PRESENT) {
      USB_SendString("Error: No SD card detected in slot.\r\n");
      strcpy(response.message, "[DIAG] Failed: No SD card");
      return response;
  }

  // 3. Try to mount if not mounted
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

cmd_response_t cmd_help_handler(char* args) {
  cmd_response_t response = {1, ""};
  print_all_commands();
  strcpy(response.message, "[HELP] Command information displayed");
  return response;
}

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

cmd_response_t cmd_createlarge_handler(char* args) {
  cmd_response_t response = {1, ""};
  (void)args;
  const UINT BUF_SIZE = 16384; // 16KB Buffer for faster write
  static uint8_t buffer[16384] __attribute__((aligned(4)));
  FRESULT fr;
  UINT bw;
  const uint32_t total_size = 10UL * 1024UL * 1024UL;
  uint32_t written = 0;
  uint32_t next_report = total_size / 10;
  uint32_t report_idx = 1;
  uint32_t pat_state = 0;
  uint32_t start_ms = 0;

  // Pre-fill buffer to save CPU time during write loop
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

/* ADS1299 Command Implementations */

    // Deleted cmd_single_handler

cmd_response_t cmd_stream_handler(char* args) {
    cmd_response_t response = {1, ""};
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    // Determine filename
    char filename[64];
    if (args != NULL && args[0] != '\0') {
        // Trim leading spaces and colon
        while(*args == ' ' || *args == ':') args++;
        
        // Use user provided filename
        if (strlen(args) > 50) { 
             args[50] = '\0';
        }
        
        // Check for extension
        if (strstr(args, ".") == NULL) {
            snprintf(filename, sizeof(filename), "%s.BIN", args);
        } else {
            snprintf(filename, sizeof(filename), "%s", args);
        }
    } else {
        strcpy(filename, "EEG_DATA.BIN");
    }

    // Use Global Configuration for SPS and Duration
    current_sps = g_cfg_sps;
    
    // Calculate Target Samples based on Global Config Duration
    if (g_cfg_duration_sec > 0) {
        target_samples = g_cfg_duration_sec * g_cfg_sps;
    } else {
        target_samples = 0xFFFFFFFF; // Infinite (approx 200 days at 250sps)
    }

    // Configure ADS1299
    // Use Global Config from CONF command (g_cfg_sps, g_cfg_gain, g_cfg_ch_mask)
    // NOTE: STREAM command does NOT reset these values. Use CONF command to change them.
    ADS1299_Stop();
    
    // 1. Set Sample Rate
    uint8_t sps_code = 0x06; // Default 250
    if (g_cfg_sps == 250) sps_code = 0x06;
    else if (g_cfg_sps == 500) sps_code = 0x05;
    else if (g_cfg_sps == 1000) sps_code = 0x04;
    else if (g_cfg_sps == 2000) sps_code = 0x03;
    else if (g_cfg_sps == 4000) sps_code = 0x02;
    else if (g_cfg_sps == 8000) sps_code = 0x01;
    else if (g_cfg_sps == 16000) sps_code = 0x00;

    ADS1299_Set_Sample_Parameter(sps_code, 6, 0); 
    
    // 1.5. Configure CONFIG2 (0x02) for Test Signal if needed
    if (g_cfg_test_sig == 1 || g_cfg_input_type == 5) {
        // 0xD0: INT_TEST=1 (Internal test signal), TEST_AMP=0 (1x), TEST_FREQ=00 (fCLK / 2^21)
        ADS1299_Write_Reg(0x02, 0xD0); 
    } else {
        // 0xC0: External Test Signal
        ADS1299_Write_Reg(0x02, 0xC0); 
    }

    // 2. Set Channels and Gain
    uint8_t gain_bits = get_gain_code(g_cfg_gain);
    
    // Enable configured channels 
    for(int i=1; i<=8; i++) { 
       if (g_cfg_ch_mask & (1 << (i-1))) {
           ADS1299_Write_Reg(0x04 + i, gain_bits | (g_cfg_input_type & 0x07)); // Use global input type
       } else {
           ADS1299_Write_Reg(0x04 + i, 0x80 | 0x01); // Power Down + Input Shorted
       }
    } 

    // 3. Configure Bias (RLD)
    // ONLY use CH6, CH7, CH8 for BIAS_SENSP (0xE0) to ensure RLD loop stability.
    // Never dynamically switch BIAS channels to prevent analog step response and oscillation.
    if (g_cfg_bias_en) {
        // Enable BIAS amplifier (PD_BIAS=1, Bit 2 in CONFIG3)
        // 0xEC = 1110 1100 (PD_REFBUF=1, PD_BIAS=1, BIASREF_INT=1, PD_BIAS_LOFF=0, BIAS_LOFF_SENS=0, BIAS_STAT=0)
        ADS1299_Write_Reg(0x03, 0xEC); 
        ADS1299_Write_Reg(0x0D, 0xE0); // Enable BIAS_SENSP for CH6, CH7, CH8 only (1110 0000)
        current_bias_sensp_mask = 0xE0; // Initialize tracking variable
    } else {
        // Disable BIAS amplifier (PD_BIAS=0)
        // 0xE8 = 1110 1000
        ADS1299_Write_Reg(0x03, 0xE8);
        ADS1299_Write_Reg(0x0D, 0x00);          // Disable BIAS_SENSP for all channels
        current_bias_sensp_mask = 0x00;
    } 
    
    // 4. Configure Lead-Off Detection (P-pole only as requested)
    // We enable this for STREAM mode to allow real-time lead-off status and bad channel removal
    
    // CONFIG4 (0x17): Enable Lead-Off Comparator (Bit 1)
    ADS1299_Write_Reg(0x17, 0x02); // PD_LOFF_COMP = 1

    // LOFF (0x04): DC Lead-off, 6nA (Bits: 000 0 00 00) => 0x00
    ADS1299_Write_Reg(0x04, 0x00); 

    // LOFF_SENSP (0x0F): Enable P-side LOFF for active channels
    ADS1299_Write_Reg(0x0F, g_cfg_ch_mask); 

    // LOFF_SENSN (0x10): Disable N-side LOFF (0x00)
    ADS1299_Write_Reg(0x10, 0x00);
    
    // LOFF_FLIP (0x11): Disable current direction flip
    ADS1299_Write_Reg(0x11, 0x00);
   
    USB_Print("\r\n[BOOT] ADS1299 Registers before START:\r\n"); 
    uint8_t regs[24]; 
    ADS1299_Read_Regs(0x00, regs, 24); 
    for(int i=0; i<24; i++) { 
       USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]); 
       HAL_Delay(5); 
    } 
    
    // Play 3.wav before starting collection
    // Since we are now in the main loop context (deferred execution),
    // it is safe to call Wav_Play.
    Wav_Play("AUDIO/3.wav");
    while(is_playing) {
        Wav_Process();
    }
    
    // Open file for recording (Stream & Save)
    FRESULT fr = f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        response.success = 0;
        sprintf(response.message, "ERR: Failed to open %s (%d)", filename, fr);
        return response;
    }
    
    // Write Binary Header
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
    
    // Clear bad channel history
    // extern uint8_t stream_bad_ch_history; // Removed redeclaration
    stream_bad_ch_history = 0;

    sprintf(response.message, "OK: STREAM started. File:%s (Mask=0x%02X, Gain=%d, SPS=%d)", 
            filename, g_cfg_ch_mask, g_cfg_gain, g_cfg_sps);
    return response;
}

cmd_response_t cmd_auto_stream_handler(char* args) {
    cmd_response_t response = {1, ""};
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    // Determine filename
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
    target_samples = 60 * 60 * sps; // 1 hour max

    // Configure ADS1299
    ADS1299_Stop();
    
    // 1. Set Sample Rate (Use global config)
    uint8_t sps_code = 0x06; // Default 250
    if (g_cfg_sps == 250) sps_code = 0x06;
    else if (g_cfg_sps == 500) sps_code = 0x05;
    else if (g_cfg_sps == 1000) sps_code = 0x04;
    else if (g_cfg_sps == 2000) sps_code = 0x03;
    else if (g_cfg_sps == 4000) sps_code = 0x02;
    else if (g_cfg_sps == 8000) sps_code = 0x01;
    else if (g_cfg_sps == 16000) sps_code = 0x00;

    ADS1299_Set_Sample_Parameter(sps_code, 6, 0); 
    
    // 1.5. Configure CONFIG2 (0x02)
    ADS1299_Write_Reg(0x02, 0xC0); // External signal - Reset to default to clear any test signals

    // 2. Set Channels and Gain
    uint8_t gain_bits = get_gain_code(g_cfg_gain);
    for(int i=1; i<=8; i++) { 
       if (g_cfg_ch_mask & (1 << (i-1))) {
           ADS1299_Write_Reg(0x04 + i, gain_bits | (g_cfg_input_type & 0x07)); 
       } else {
           ADS1299_Write_Reg(0x04 + i, 0x80 | 0x01); 
       }
    } 

    // 3. Configure Bias (RLD) - Initial Setup
    // ONLY use CH6, CH7, CH8 for BIAS_SENSP (0xE0) to ensure RLD loop stability.
    if (g_cfg_bias_en) {
        ADS1299_Write_Reg(0x03, 0xEC); 
        ADS1299_Write_Reg(0x0D, 0xE0); // Enable BIAS_SENSP for CH6, CH7, CH8 only (1110 0000)
        current_bias_sensp_mask = 0xE0; // Initialize tracking variable
    } else {
        ADS1299_Write_Reg(0x03, 0xE8);
        ADS1299_Write_Reg(0x0D, 0x00);
        current_bias_sensp_mask = 0x00;
    } 

    // 4. Configure Lead-Off Detection (P-pole only as requested)
    // CONFIG4 (0x17): Enable Lead-Off Comparator (Bit 1)
    ADS1299_Write_Reg(0x17, 0x02); // PD_LOFF_COMP = 1

    // LOFF (0x04): DC Lead-off, 6nA (Bits: 000 0 00 00) => 0x00
    ADS1299_Write_Reg(0x04, 0x00); 

    // LOFF_SENSP (0x0F): Enable P-side LOFF for active channels
    ADS1299_Write_Reg(0x0F, g_cfg_ch_mask); 

    // LOFF_SENSN (0x10): Disable N-side LOFF (0x00)
    ADS1299_Write_Reg(0x10, 0x00);
    
    // LOFF_FLIP (0x11): Disable current direction flip
    ADS1299_Write_Reg(0x11, 0x00);

    // Play 3.wav
    Wav_Play("AUDIO/3.wav");
    while(is_playing) {
        Wav_Process();
    }
    
    // Open file
    FRESULT fr = f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        response.success = 0;
        sprintf(response.message, "ERR: Failed to open %s (%d)", filename, fr);
        return response;
    }

    // Write Binary Header
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

    // Reset bad channel history for the new session
    stream_bad_ch_history = 0;

    // Dump Registers for verification
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

cmd_response_t cmd_conf_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    // Default values if parsing fails
    unsigned int mask = 0xFF;
    int gain = 24;
    int sps = 250;
    int input_type = 0;
    int bias_en = 1;
    
    // Parse args: CONF:MASK(hex),GAIN,SPS,INPUT_TYPE,BIAS_EN
    // Example: CONF:FF,24,250,0,1
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
    } else {
        response.success = 0;
        sprintf(response.message, "ERR: Usage CONF:MASK(hex),GAIN,SPS,INPUT_TYPE,BIAS_EN");
    }
    return response;
}

cmd_response_t cmd_power_test_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }
    
    // 1. Open SD File for continuous writing
    FRESULT fr = f_open(&SDFile, "PWR_TEST.BIN", FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        response.success = 0;
        sprintf(response.message, "ERR: SD Open failed (%d)", fr);
        return response;
    }
    
    // 2. Start ADS1299 (Max Power: All channels ON, Gain 24, 250 SPS)
    ADS1299_Stop();
    ADS1299_Set_Sample_Parameter(0x06, 6, 0); // 250SPS
    
    // Enable all channels, Gain 24, Normal Input
    for(int i=1; i<=8; i++) {
        ADS1299_Write_Reg(0x04 + i, 0x60); 
    }
    // Enable BIAS
    ADS1299_Write_Reg(0x03, 0xEC); 
    ADS1299_Write_Reg(0x0D, 0xFF); 
    
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);
    
    // 3. Start Audio Loop (Play 3.wav continuously in loop)
    Wav_Play("AUDIO/3.wav"); 
    
    // 4. Turn ON LED
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    
    // 5. Set State to POWER_TEST
    app_state = APP_POWER_TEST_RUNNING;
    start_time = HAL_GetTick();
    sample_count = 0;
    
    strcpy(response.message, "OK: POWER TEST STARTED (ADS+SD+AUDIO+LED). Send STOP to end.");
    return response;
}

cmd_response_t cmd_adc_test_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }
    
    // Open adc.txt for appending
    FRESULT fr = f_open(&SDFile, "adc.txt", FA_OPEN_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        response.success = 0;
        sprintf(response.message, "ERR: SD Open adc.txt failed (%d)", fr);
        return response;
    }
    
    // Move to end of file
    f_lseek(&SDFile, f_size(&SDFile));
    
    // Write header
    char header[] = "Timestamp_ms,Raw_ADC,Voltage_V,Percent\r\n";
    UINT bw;
    f_write(&SDFile, header, strlen(header), &bw);
    f_sync(&SDFile);
    
    app_state = APP_ADC_TEST_RUNNING;
    start_time = HAL_GetTick(); // Use start_time to track last log time
    
    strcpy(response.message, "OK: ADC Logging Started (0.5s interval). Send STOP to end.");
    return response;
}

cmd_response_t cmd_stop_handler(char* args) {
    cmd_response_t response = {1, "OK: Stopping..."};
    
    // 如果本来就是空闲状态，直接返回，避免在空闲时关闭未打开的文件或错误触发停止音效导致死锁
    if (app_state == APP_IDLE) {
        strcpy(response.message, "OK: Already Idle");
        return response;
    }
    
    // Immediately close file if recording to prevent lock errors on subsequent commands (e.g. GET)
    if (app_state == APP_QUAD_RUNNING || app_state == APP_STREAM_RUNNING || app_state == APP_AUTO_STREAM_RUNNING || app_state == APP_POWER_TEST_RUNNING || app_state == APP_ADC_TEST_RUNNING || app_state == APP_IMP_TEST_ALL_RUNNING) {
        if (app_state != APP_ADC_TEST_RUNNING) ADS1299_Stop(); // Only stop ADS1299 if it was running
        f_close(&SDFile);
    }
    
    // Also stop audio if playing during power test
    // Stop any AD5933 sweeps if running
    
    app_state = APP_STOP_REQUESTED;
    
    // In case we were in binary impedance mode, we should revert or stay depending on needs.
    // We can just leave g_use_binary_protocol as is, because the STOP command itself 
    // will send its response via the command system based on its origin.
    
    return response;
}

// Static float buffers for IMU data sharing
static float g_latest_acc[3] = {0};
static float g_latest_gyro[3] = {0};
static uint8_t g_latest_motion_state = 0;

void app_run_loop(void) {
    // Check for OTA Completion Flag from Interrupt
    if (g_ota_complete_flag) {
        g_ota_complete_flag = 0;
        
        // Now we are in Main Loop context, safe to call FatFS and Reset
        FRESULT fr_close = f_close(&ota_fil);
        uint32_t duration = HAL_GetTick() - ota_start_tick;
        
        if (fr_close == FR_OK) {
            USB_Print("\r\n[OTA] Transfer Complete. %lu bytes.\r\n", ota_received_bytes);
            USB_Print("[OTA] Time: %lu ms. Auto-resetting in 1s...\r\n", duration);
            
            // Wait for USB transmission to finish
            HAL_Delay(1000);
            
            // Perform System Reset
            NVIC_SystemReset();
        } else {
            USB_Print("\r\n[OTA] Error: File Close Failed (%d)\r\n", fr_close);
        }
    }

    if (app_state == APP_IDLE) return;

    if (app_state == APP_STOP_REQUESTED) {
        ADS1299_Stop();
        
        // Fix: Always try to close the file when stopping, to ensure no file is left open.
        // It's safe to call f_close on an already closed file object if we are careful, 
        // or we can track if it's open. 
        // Given the bug was "File not found (16)" which means LOCKED, we must ensure close.
        // We don't know the previous state here because it was overwritten to APP_STOP_REQUESTED.
        // So we just close it.
        f_close(&SDFile);
        USB_Print("File closed (if open).\r\n");

        app_state = APP_IDLE;
        
        // 只要回到空闲状态，无论从哪个模式，都恢复蓝灯常亮
        RGB_LED_SetSingle(LED_COLOR_BLUE, LED_MODE_SOLID_ON);
        
        if (g_task_state == 2) { // RUNNING
            g_task_state = 3; // COMPLETED
            
            // Clear Task State in Backup Register
            BackupReg_Write(BKP_REG_TASK_STATE, 0);
            BackupReg_Write(BKP_REG_TASK_ID, 0);
        }
        
        // Play 4.wav after stopping collection
        Wav_Play("AUDIO/4.wav");
        while(is_playing) {
            Wav_Process();
        }
        
        USB_Print("Acquisition Stopped. Saved %lu samples.\r\n", sample_count);
        
        // Report bad channels history if any
        if (stream_bad_ch_history != 0) {
             USB_Print("Warning: Bad channels detected during session: 0x%02X\r\n", stream_bad_ch_history);
        }
        
        return;
    }

    if (app_state == APP_ADC_TEST_RUNNING) {
        if (HAL_GetTick() - start_time >= 500) {
            start_time = HAL_GetTick();
            
            // Read ADC
            HAL_ADC_Start(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
                uint32_t raw = HAL_ADC_GetValue(&hadc1);
                // Updated formula based on 4.2V (Raw 4031) and 3.7V (Raw 3519) measurements
                float voltage = ((float)raw * 0.00097656f) + 0.2635f;
                float percentage = (voltage - 3.6f) / (4.1f - 3.6f) * 100.0f;
                if (percentage < 0) percentage = 0;
                if (percentage > 100) percentage = 100;
                
                // Log to SD
                char line[64];
                int len = snprintf(line, sizeof(line), "%lu,%lu,%.3f,%.1f\r\n", 
                                   HAL_GetTick(), raw, voltage, percentage);
                
                UINT bw;
                f_write(&SDFile, line, len, &bw);
                f_sync(&SDFile);
                
                // Print to USB for real-time check
                USB_Print("ADC: Raw=%lu, V=%.3f, P=%.1f%%\r\n", raw, voltage, percentage);
            }
        }
    }

    if (app_state == APP_SINGLE_RUNNING || app_state == APP_QUAD_RUNNING || app_state == APP_STREAM_RUNNING || app_state == APP_AUTO_STREAM_RUNNING || app_state == APP_IMP_TEST_RUNNING || app_state == APP_NOISE_TEST_RUNNING || app_state == APP_CMRR_TEST_RUNNING || app_state == APP_CMRR_STEP1_RUNNING || app_state == APP_CMRR_STEP2_RUNNING || app_state == APP_IMP_TEST_ALL_RUNNING || app_state == APP_RLD_TEST_RUNNING || app_state == APP_POWER_TEST_RUNNING) 
			{
        
        // --- AUX Data Handling (50Hz Sampling, 1Hz Output) for STREAM mode ---
        if (app_state == APP_STREAM_RUNNING) {
            
            static uint32_t last_aux_sample_time = 0;
            static uint32_t last_aux_output_time = 0;
            static float avg_motion = 0.0f;
            static float peak_energy = 0.0f;
            static float last_acc[3] = {0};
            static uint8_t first_sample = 1;

            // 50Hz Sampling Interval = 20ms
            if (HAL_GetTick() - last_aux_sample_time >= 20) {
                float acc[3], gyro[3];
                QMI8658_Read_XYZ(acc, gyro); // Read IMU
                
                // Update global cache for binary stream
                g_latest_acc[0] = acc[0]; g_latest_acc[1] = acc[1]; g_latest_acc[2] = acc[2];
                g_latest_gyro[0] = gyro[0]; g_latest_gyro[1] = gyro[1]; g_latest_gyro[2] = gyro[2];

                if (first_sample) {
                    last_acc[0] = acc[0];
                    last_acc[1] = acc[1];
                    last_acc[2] = acc[2];
                    first_sample = 0;
                }

                // 1. Calculate Acceleration Jerk (Manhattan Distance of diff)
                // |Ax_t - Ax_t-1| + |Ay_t - Ay_t-1| + |Az_t - Az_t-1|
                float acc_diff = fabsf(acc[0] - last_acc[0]) + 
                                 fabsf(acc[1] - last_acc[1]) + 
                                 fabsf(acc[2] - last_acc[2]);
                
                // Clamp extreme spikes (acc_diff > 0.5g)
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
                     uint16_t tx_len = BP_Pack(g_bp_tx_buf, BP_TYPE_AUX, aux_seq++, aux_payload, 16);
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

        // Check timeout
        if (sample_count >= target_samples) {
            app_state = APP_STOP_REQUESTED;
            return;
        }

        uint8_t packet[27];
        // ADS1299 packet: 3 bytes status + 8 * 3 bytes data = 27 bytes
        while (circular_buffer_get_data_count(&ADS1299_circula_buffer) >= 27) {
            // Frame Sync Check: peek at the first byte to verify it's the ADS1299 status header (0xC0)
            uint8_t header_byte;
            circular_buffer_pop(&ADS1299_circula_buffer, &header_byte);
            
            if ((header_byte & 0xF0) != 0xC0) {
                // Misaligned frame detected! Drop this byte and continue searching for the 0xC0 header.
                continue;
            }
            
            // Valid header found. Put it in packet[0] and read the remaining 26 bytes.
            packet[0] = header_byte;
            circular_buffer_read(&ADS1299_circula_buffer, &packet[1], 26);
            
            static char line[256];
            int len = 0;
            
            // Gain 24, Vref=4.5V
            // 4.5V / (2^23 - 1) = 0.53644 uV (LSB at Gain=1)
            // scale unit: uV / LSB
            float gain_val = (float)g_cfg_gain;
            if (gain_val < 1.0f) gain_val = 1.0f; // Safety check
            float scale = 0.5364f / gain_val; 

            if (app_state == APP_CMRR_STEP1_RUNNING) {
                // Step 1: Differential Gain
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
                // Step 2: Common Mode Gain & Final Calculation
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
                    const float Vin_comm_rms = 2.0f * 0.7071f; // 2Vpp = 0.7071 Vrms? Wait.
                    // Assuming user input 2Vpp means 2V Peak-to-Peak. Vrms = 2 / (2*sqrt(2)) = 0.7071
                    // Correct.
                    
                    for(int i=0; i<8; i++) {
                        double vout_comm_rms = sqrt(cmrr_step2_sum_sq[i] / target_samples);
                        double comm_gain = vout_comm_rms / 0.7071; // Vin_comm = 0.7071
                        
                        double cmrr_ratio = cmrr_diff_gain[i] / comm_gain;
                        double cmrr_db = 20.0 * log10(cmrr_ratio);
                        
                        USB_Print(" %d | %9.2f | %9.2e | %9.2f\r\n", 
                                  i+1, cmrr_diff_gain[i], comm_gain, cmrr_db);//
                    }
                }
            }
            else if (app_state == APP_CMRR_TEST_RUNNING) {
                for(int i=0; i<8; i++) {
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    float voltage = (float)val * scale * 1e-6f; // Convert uV to Volts for calculation
                    
                    // We care about AC signal magnitude (RMS)
                    // Simple RMS calculation assuming mean is close to 0 or removed by HPF
                    // For more accuracy, we should subtract mean first, but here we just sum squares
                    cmrr_sum_sq[i] += (double)(voltage * voltage);
                }

                if (sample_count >= target_samples - 1) {
                    USB_SendString("\r\n[TEST_CMRR] Analysis Result (Assuming Vin=0.707 Vrms):\r\n");
                    USB_SendString("CH | Vout (uVrms) | CMRR (dB)\r\n");
                    USB_SendString("---|--------------|----------\r\n");
                    
                    for(int i=0; i<8; i++) {
                        double vout_rms = sqrt(cmrr_sum_sq[i] / target_samples);
                        // CMRR = 20 * log10( (Gain * Vin) / Vout )
                        // Gain = 24
                        double ratio = (24.0 * CMRR_VIN_RMS) / vout_rms;
                        double cmrr_db = 20.0 * log10(ratio);
                        
                        USB_Print(" %d | %12.3f | %8.2f\r\n", 
                                  i+1, vout_rms * 1e6, cmrr_db);
                    }
                }
            }
            else if (app_state == APP_RLD_TEST_RUNNING) {
                // RLD Test: Calculate RMS of AC Component (Standard Deviation)
                for(int i=0; i<8; i++) {
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    float voltage = (float)val * scale; // uV (Note: scale is uV/LSB in line 673)
                    
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
                
                // Check if done (last sample)
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
                // Just writing to file, handled by file writing loop if applicable.
                // In this simplified version, we just let the data flow.
                // If we want to verify data integrity or format, we could add checks here.
                // For power test, the main goal is keeping peripherals busy.
                
                // Write data to SD card (if not using DMA directly to SD)
                // For simplicity, we are not writing EEG data to SD in this loop 
                // because SD writing is typically handled by buffering or separate task.
                // However, let's simulate SD activity by writing a chunk periodically.
                // Actually, 'PWR_TEST.BIN' was opened in write mode. 
                // Let's write the packet to it to maximize power consumption.
                UINT bw;
                f_write(&SDFile, packet, 27, &bw);
            }
            else if (app_state == APP_IMP_TEST_ALL_RUNNING) {
                // 8-Channel Impedance Logic
                for(int i=0; i<8; i++) {
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    float voltage = (float)val * scale; 

                    if (voltage > imp_all_max[i]) imp_all_max[i] = voltage;
                    if (voltage < imp_all_min[i]) imp_all_min[i] = voltage;
                }
                
                imp_window_count++;

                // Calculate Impedance every 0.5 second (125 samples)
                if (imp_window_count >= 125) {
                    float impedances[8];
                    
                    for(int i=0; i<8; i++) {
                        float vpp = imp_all_max[i] - imp_all_min[i]; 
                        // Z = Vpp(uV) * 1e-6 / (2 * 24nA) = Vpp / 0.048
                        // Current increased to 24nA to improve SNR
                        impedances[i] = (vpp * 1e-6f) / 48.0e-9f;
                        
                        // Reset
                        imp_all_max[i] = -1e9;
                        imp_all_min[i] = 1e9;
                    }
                    
                    // Format: IMP:F5:v,F3:v,FZ:v,F4:v,F6:v,AF8:v,FPZ:v,AF7:v
                    // Values are in kOhm, capped at 9999
                    
                    if (g_use_binary_protocol) {
                        uint8_t imp_payload[16];
                        for(int i=0; i<8; i++) {
                             // Unit: 1 kOhm as per Word Document request (was 0.1kOhm before)
                             // impedances[i] is in Ohms. Divide by 1000 to get kOhm.
                             float imp_kohm = impedances[i] / 1000.0f;
                             if (imp_kohm > 65535.0f) imp_kohm = 65535.0f;
                             uint16_t imp_val = (uint16_t)imp_kohm;
                             memcpy(&imp_payload[i*2], &imp_val, 2);
                        }
                        static uint16_t imp_seq = 0;
                        uint16_t tx_len = BP_Pack(g_bp_tx_buf, BP_TYPE_IMP, imp_seq++, imp_payload, 16);
                        CDC_Transmit_FS(g_bp_tx_buf, tx_len);
                    } else {
                        char imp_buf[256];
                        snprintf(imp_buf, sizeof(imp_buf), 
                            "IMP:F5:%d,F3:%d,FZ:%d,F4:%d,F6:%d,AF8:%d,FPZ:%d,AF7:%d\r\n",
                            (int)(impedances[0] > 9999000 ? 9999 : impedances[0]/1000),
                            (int)(impedances[1] > 9999000 ? 9999 : impedances[1]/1000),
                            (int)(impedances[2] > 9999000 ? 9999 : impedances[2]/1000),
                            (int)(impedances[3] > 9999000 ? 9999 : impedances[3]/1000),
                            (int)(impedances[4] > 9999000 ? 9999 : impedances[4]/1000),
                            (int)(impedances[5] > 9999000 ? 9999 : impedances[5]/1000),
                            (int)(impedances[6] > 9999000 ? 9999 : impedances[6]/1000),
                            (int)(impedances[7] > 9999000 ? 9999 : impedances[7]/1000)
                        );
                        USB_SendString(imp_buf);
                    }
                    
                    imp_window_count = 0;
                }
            }
            else if (app_state == APP_IMP_TEST_RUNNING) {
                // Impedance Test Logic (CH1 only for now)
                int ch_offset = 3 + (current_channel - 1) * 3;
                int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                if (val & 0x800000) val |= 0xFF000000;
                float voltage = (float)val * scale; // Voltage in uV

                // Debug: Print Raw Hex and Voltage periodically
                // Every 10th sample to avoid flooding too much, but enough to see
                if (imp_window_count % 10 == 0) {
                     len = snprintf(line, sizeof(line), "DBG: Hex=0x%06X, Val=%ld, Volts=%.2f uV\r\n", 
                                    (unsigned int)(val & 0xFFFFFF), (long)val, voltage);
                     CDC_Transmit_FS((uint8_t*)line, len);
                }

                // Update Max/Min
                if (voltage > imp_max_val) imp_max_val = voltage;
                if (voltage < imp_min_val) imp_min_val = voltage;
                imp_window_count++;

                // Calculate Impedance periodically
                if (imp_window_count >= IMP_WINDOW_SIZE) {
                    float vpp = imp_max_val - imp_min_val; // Peak-to-Peak Voltage (in uV)
                    
                    // Z = Vpp(uV) * 1e-6 / (2 * I_inj)
                    // I_inj = 6nA = 6e-9 A (Configured via LOFF reg 0x01 -> ILEAD_OFF=00 -> 6nA)
                    // Z = Vpp(uV) * 1e-6 / 12e-9 = Vpp(uV) / 0.012
                    float raw_impedance = (vpp * 1e-6f) / (2.0f * 6.0e-9f);
                    
                    // No calibration, use raw impedance
                    float impedance = raw_impedance;
                    
                    len = snprintf(line, sizeof(line), "Z: %.2f kOhm (Vpp: %.2f uV)\r\n", 
                                   impedance / 1000.0f, vpp);
                    CDC_Transmit_FS((uint8_t*)line, len);

                    // Accumulate for average
                    imp_sum += impedance;
                    imp_count++;

                    // Reset for next window
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
                // Note: SINGLE mode currently does not write to SD, as requested only for QUAD
                CDC_Transmit_FS((uint8_t*)line, len);
            } else if (app_state == APP_AUTO_STREAM_RUNNING) {
                // Check Lead-Off (P-Pole)
                // Status Byte 0: 1100 + IN8P..IN5P
                // Status Byte 1: IN4P..IN1P + IN8N..IN5N
                
                uint8_t loff_statp = 0;
                // IN8P..IN5P (Bits 3-0 of Byte 0) -> Map to Bits 7-4
                if (packet[0] & 0x08) loff_statp |= 0x80; // Ch8
                if (packet[0] & 0x04) loff_statp |= 0x40; // Ch7
                if (packet[0] & 0x02) loff_statp |= 0x20; // Ch6
                if (packet[0] & 0x01) loff_statp |= 0x10; // Ch5
                // IN4P..IN1P (Bits 7-4 of Byte 1) -> Map to Bits 3-0
                if (packet[1] & 0x80) loff_statp |= 0x08; // Ch4
                if (packet[1] & 0x40) loff_statp |= 0x04; // Ch3
                if (packet[1] & 0x20) loff_statp |= 0x02; // Ch2
                if (packet[1] & 0x10) loff_statp |= 0x01; // Ch1
                
                uint8_t bad_channels = loff_statp & current_bias_sensp_mask;
                
                // Track bad channels history
                stream_bad_ch_history |= bad_channels;
                
                if (bad_channels != 0) {
                     // [FIX] We no longer power down channels or modify BIAS on lead-off.
                     // We just record the bad channels to history for status reporting.
                     // The hardware continues to run all channels normally.
                     
                     // Optional: You could still print a warning here if needed, 
                     // but to avoid flooding the console, we rely on the binary status bits.
                     /*
                     USB_Print("[AUTO] Lead-off detected (Bad: 0x%02X). Continuing acquisition.\r\n", 
                               bad_channels);
                     */
                }
                
                // --- AUX Data Handling (50Hz Sampling, 1Hz Output) for AUTO_STREAM mode ---
                static uint32_t last_aux_sample_time = 0;
                static uint32_t last_aux_output_time = 0;
                static float avg_motion = 0.0f;
                static float peak_energy = 0.0f;
                static float last_acc[3] = {0};
                static uint8_t first_sample = 1;

                // 50Hz Sampling Interval = 20ms
                if (HAL_GetTick() - last_aux_sample_time >= 20) {
                    float acc[3], gyro[3];
                    QMI8658_Read_XYZ(acc, gyro); // Read IMU

                    g_latest_acc[0] = acc[0]; g_latest_acc[1] = acc[1]; g_latest_acc[2] = acc[2];
                    g_latest_gyro[0] = gyro[0]; g_latest_gyro[1] = gyro[1]; g_latest_gyro[2] = gyro[2];

                    if (first_sample) {
                        last_acc[0] = acc[0];
                        last_acc[1] = acc[1];
                        last_acc[2] = acc[2];
                        first_sample = 0;
                    }

                    // 1. Calculate Acceleration Jerk (Manhattan Distance of diff)
                    float acc_diff = fabsf(acc[0] - last_acc[0]) + 
                                     fabsf(acc[1] - last_acc[1]) + 
                                     fabsf(acc[2] - last_acc[2]);
                    
                    // Clamp extreme spikes (acc_diff > 0.5g)
                    if (acc_diff > 0.5f) acc_diff = 0.5f;

                    // Update last acc
                    last_acc[0] = acc[0];
                    last_acc[1] = acc[1];
                    last_acc[2] = acc[2];

                    // 2. Calculate Gyro Magnitude
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
                    QMI8658_Read_XYZ(acc, gyro); // Read one latest sample for display
                    
                    float final_energy = (avg_motion > peak_energy) ? avg_motion : peak_energy;
                    
                    // Motion Score Normalization (0-100)
                    float motion_score = final_energy * 5.0f;
                    if (motion_score > 100.0f) motion_score = 100.0f;
                    if (motion_score < 0.0f) motion_score = 0.0f;

                    motion_state_t final_state = Motion_State_Update(motion_score);
                    g_latest_motion_state = (uint8_t)final_state;

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
                        uint16_t tx_len = BP_Pack(g_bp_tx_buf, BP_TYPE_AUX, aux_seq_auto++, aux_payload, 16);
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
                
                // Normal Stream Logic (Same as APP_STREAM_RUNNING)
                float voltages[8];
                int32_t raw_vals[8];
                
                for(int i=0; i<8; i++) {
                    // [FIX] Removed data cloning for lead-off channels.
                    // We now parse the actual channel data regardless of its lead-off status.
                    // If the electrode is off, the上位机 will see the real noise/railing, 
                    // which is the medically correct behavior.
                    
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    
                    raw_vals[i] = val;
                    voltages[i] = (float)val * scale;
                }
                
                if (g_use_binary_protocol) {
                    // Status (3)
                    g_bp_payload[0] = packet[0];
                    g_bp_payload[1] = packet[1];
                    g_bp_payload[2] = packet[2];
                    
                    // Channels (32)
                    for(int i=0; i<8; i++) {
                        int32_t val = raw_vals[i];
                        
                        g_bp_payload[3 + i*4] = val & 0xFF;
                        g_bp_payload[3 + i*4 + 1] = (val >> 8) & 0xFF;
                        g_bp_payload[3 + i*4 + 2] = (val >> 16) & 0xFF;
                        g_bp_payload[3 + i*4 + 3] = (val >> 24) & 0xFF;
                    }
                    
                    // IMU (12)
                    for(int k=0; k<3; k++) {
                        int16_t acc_val = (int16_t)(g_latest_acc[k] * 1000.0f);//
                        int16_t gyro_val = (int16_t)(g_latest_gyro[k] * 10.0f);
                        
                        int acc_idx = 35 + k*2;
                        g_bp_payload[acc_idx] = acc_val & 0xFF;
                        g_bp_payload[acc_idx+1] = (acc_val >> 8) & 0xFF;
                        
                        int gyro_idx = 41 + k*2;
                        g_bp_payload[gyro_idx] = gyro_val & 0xFF;
                        g_bp_payload[gyro_idx+1] = (gyro_val >> 8) & 0xFF;
                    }
                    
                    static uint16_t bin_seq = 0;
                    uint16_t tx_len = BP_Pack(g_bp_tx_buf, BP_TYPE_EEG, bin_seq++, g_bp_payload, 47);
                    
                    // Direct Transmit (No Slow Down)
                    uint8_t usb_res = CDC_Transmit_FS(g_bp_tx_buf, tx_len);
                    if (usb_res == USBD_BUSY) {
                        for(volatile int w=0; w<1000; w++); 
                        usb_res = CDC_Transmit_FS(g_bp_tx_buf, tx_len);
                    }
                } else {
                    // EEG Data Transmission (Replaced LOFF status strings)
                    len = snprintf(line, sizeof(line), "EEG:F5:%.2f,F3:%.2f,FZ:%.2f,F4:%.2f,F6:%.2f,AF8:%.2f,FPZ:%.2f,AF7:%.2f\r\n",
                                   voltages[0], voltages[1], voltages[2], voltages[3],
                                   voltages[4], voltages[5], voltages[6], voltages[7]);
                    
                    uint8_t usb_res = CDC_Transmit_FS((uint8_t*)line, len);
                    if (usb_res == USBD_BUSY) {
                        uint32_t retry_start = HAL_GetTick();
                        while (CDC_Transmit_FS((uint8_t*)line, len) == USBD_BUSY) {
                            if (HAL_GetTick() - retry_start > 2) break; 
                        }
                    }
                }
                
                // Write to SD Card (Binary DataFrame)
                DataFrame_t frame = {0};
                frame.frame_seq = sample_count;
                for(int i=0; i<8; i++) {
                    frame.eeg_data[i] = raw_vals[i];
                }
                for(int k=0; k<3; k++) {
                    frame.acc[k] = (int16_t)(g_latest_acc[k] * 1000.0f);//
                    frame.gyro[k] = (int16_t)(g_latest_gyro[k] * 10.0f);
                }
                frame.motion_state = g_latest_motion_state;
                
                // Calculate simple checksum
                uint8_t *p_frame = (uint8_t*)&frame;
                uint8_t chk = 0;
                for(size_t i=0; i<sizeof(DataFrame_t)-1; i++) {
                    chk ^= p_frame[i];
                }
                frame.checksum = chk;
                
                // [FIX] Commented out SD card write to prevent blocking the main loop
                // which causes ADS1299 circular buffer overflow and frame misalignment
                
                UINT bw;
                FRESULT fr = f_write(&SDFile, &frame, sizeof(DataFrame_t), &bw);
                if (fr == FR_OK && bw == sizeof(DataFrame_t)) {
                    if (sample_count % 50 == 0) {
                        f_sync(&SDFile);
                    }
                }
                
            } else if (app_state == APP_STREAM_RUNNING) {
                // Check Lead-Off (P-Pole)
                // Status Byte 0: 1100 + IN8P..IN5P (Bits 3-0)
                // Status Byte 1: IN4P..IN1P (Bits 7-4) + IN8N..IN5N (Bits 3-0)
                
                uint8_t loff_statp = 0;
                // IN8P..IN5P (Bits 3-0 of Byte 0) -> Map to Bits 7-4
                if (packet[0] & 0x08) loff_statp |= 0x80; // Ch8
                if (packet[0] & 0x04) loff_statp |= 0x40; // Ch7
                if (packet[0] & 0x02) loff_statp |= 0x20; // Ch6
                if (packet[0] & 0x01) loff_statp |= 0x10; // Ch5
                // IN4P..IN1P (Bits 7-4 of Byte 1) -> Map to Bits 3-0
                if (packet[1] & 0x80) loff_statp |= 0x08; // Ch4
                if (packet[1] & 0x40) loff_statp |= 0x04; // Ch3
                if (packet[1] & 0x20) loff_statp |= 0x02; // Ch2
                if (packet[1] & 0x10) loff_statp |= 0x01; // Ch1
                
                // Identify channels that are bad AND currently enabled in Bias
                uint8_t new_bad_channels = loff_statp & current_bias_sensp_mask;
                
                if (new_bad_channels != 0) {
                     // [FIX] We no longer power down channels or modify BIAS on lead-off.
                     // We just record the bad channels to history for status reporting.
                     // The hardware continues to run all channels normally.
                     
                     // Accumulate history
                     stream_bad_ch_history |= new_bad_channels;
                     
                     // In ASCII mode, we can optionally print it (commented out to avoid flooding)
                     /*
                     if (!g_use_binary_protocol) {
                         USB_Print("[STREAM] Bad channels detected: 0x%02X. Continuing acquisition.\r\n", new_bad_channels);
                     }
                     */
                }
                
                float voltages[8];
                int32_t raw_vals[8];
                
                // Parse values with channel replacement logic for lead-off channels
                for(int i=0; i<8; i++) {
                    // [FIX] Removed data cloning for lead-off channels.
                    // We now parse the actual channel data regardless of its lead-off status.
                    // If the electrode is off, the上位机 will see the real noise/railing, 
                    // which is the medically correct behavior.
                    
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    
                    raw_vals[i] = val;
                    voltages[i] = (float)val * scale;
                }
                
                // Format: EEG:SEQ:x,v1,v2,v3,v4,v5,v6,v7,v8\n
                if (g_use_binary_protocol) {
                    // Update Status Bytes with historical lead-off status so UI stays red
                    // packet[0] Bits 3-0: CH8, CH7, CH6, CH5
                    // packet[1] Bits 7-4: CH4, CH3, CH2, CH1
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
                    uint16_t tx_len = BP_Pack(g_bp_tx_buf, BP_TYPE_EEG, bin_seq++, g_bp_payload, 47);
                    
                    // Direct Transmit (No Slow Down)
                    uint8_t usb_res = CDC_Transmit_FS(g_bp_tx_buf, tx_len);
                    if (usb_res == USBD_BUSY) {
                         // Let's try a very short busy wait
                         for(volatile int w=0; w<1000; w++); 
                         usb_res = CDC_Transmit_FS(g_bp_tx_buf, tx_len);
                    }
                } else {
								    len = snprintf(line, sizeof(line), "EEG:F5:%.2f,F3:%.2f,FZ:%.2f,F4:%.2f,F6:%.2f,AF8:%.2f,FPZ:%.2f,AF7:%.2f\r\n",
                                   voltages[0], voltages[1], voltages[2], voltages[3],
                                   voltages[4], voltages[5], voltages[6], voltages[7]);
                    uint8_t usb_res = CDC_Transmit_FS((uint8_t*)line, len);
                    if (usb_res == USBD_BUSY) {
                        uint32_t retry_start = HAL_GetTick();
                        while (CDC_Transmit_FS((uint8_t*)line, len) == USBD_BUSY) {
                            if (HAL_GetTick() - retry_start > 2) break; // 2ms timeout - don't block
                        }
                    }
                }
                
                // Note: If USB is disconnected, CDC_Transmit_FS might return USBD_FAIL or just work (dropping data).
                // The key is the timeout above prevents infinite waiting.
                
                // Write to SD Card (Stream & Save Binary)
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
                
                // Calculate simple checksum
                uint8_t *p_frame = (uint8_t*)&frame;
                uint8_t chk = 0;
                for(size_t i=0; i<sizeof(DataFrame_t)-1; i++) {
                    chk ^= p_frame[i];
                }
                frame.checksum = chk;
                
                // Write to SD Card (Stream & Save Binary)
                
                UINT bw;
                FRESULT fr = f_write(&SDFile, &frame, sizeof(DataFrame_t), &bw);
                if (fr != FR_OK || bw < sizeof(DataFrame_t)) {
                    // SD Write Error - continue for now
                } else {
                    // Sync periodically (every 50 samples ~ 0.2s)
                    if (sample_count % 50 == 0) {
                        f_sync(&SDFile);
                    }
                }
                
            } else {
                // QUAD (8-channel) mode
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
                
                // Calculate simple checksum
                uint8_t *p_frame = (uint8_t*)&frame;
                uint8_t chk = 0;
                for(size_t i=0; i<sizeof(DataFrame_t)-1; i++) {
                    chk ^= p_frame[i];
                }
                frame.checksum = chk;
                
                // Write to SD Card
                UINT bw;
                FRESULT fr = f_write(&SDFile, &frame, sizeof(DataFrame_t), &bw);
                if (fr != FR_OK || bw < sizeof(DataFrame_t)) {
                    if (g_use_binary_protocol == 0) USB_Print("SD Write Err: %d\r\n", fr);
                    app_state = APP_STOP_REQUESTED; // Stop on write error
                } else {
                    if (sample_count % 50 == 0) {
                        f_sync(&SDFile);
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

cmd_response_t cmd_reg_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition running, stop first");
        return response;
    }

    uint8_t regs[24];
    memset(regs, 0, 24);

    // Ensure SDATAC mode
    ADS1299_Send_Cmd(ADS1299_SDATAC);
    HAL_Delay(10); 

    // Read all registers (0x00 to 0x17)
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

cmd_response_t cmd_sweep_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }

    // AD5933 is disabled in this hardware revision
    response.success = 0;
    strcpy(response.message, "ERR: AD5933 disabled");
    return response;
}

cmd_response_t cmd_test_imp_handler(char* args) {
    cmd_response_t response = {1, ""};

    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    ADS1299_Stop();
    HAL_Delay(10);

    // Manual Register Configuration for Impedance Test
    ADS1299_Write_Reg(0x00, 0x3E); // ID Control (Read Only, but user asked to set it?) - Actually ID is RO, usually 0x3E for ADS1299
    ADS1299_Write_Reg(0x01, 0x96); // CONFIG1: Daisy-chain disable, oscillator clock output disable, 250 SPS
    ADS1299_Write_Reg(0x02, 0xC0); // CONFIG2: Internal test signal generation
    ADS1299_Write_Reg(0x03, 0xE6); // CONFIG3: Internal reference buffer enabled, VREF = 4.5V
    ADS1299_Write_Reg(0x04, 0x01); // LOFF: 24nA Current, AC Lead-off at fDR/4 (000 0 01 01)

    // CHnSET: 0x60 = Gain 24, Normal Input; 0x80 = Gain 24, Input Shorted? No, 0x80 is powered down?
    // Wait, 0x80 = 1000 0000 -> PD=1 (Power Down), GAIN=1 (1), MUX=0 (Normal) -> Power Down
    // 0x60 = 0110 0000 -> PD=0 (Active), GAIN=24 (110), MUX=0 (Normal)
    ADS1299_Write_Reg(0x05, 0x60); // CH1: Active, Gain 24, Normal Input
    ADS1299_Write_Reg(0x06, 0x80); // CH2: Power Down
    ADS1299_Write_Reg(0x07, 0x80); // CH3: Power Down
    ADS1299_Write_Reg(0x08, 0x80); // CH4: Power Down
    ADS1299_Write_Reg(0x09, 0x80); // CH5: Power Down
    ADS1299_Write_Reg(0x0A, 0x80); // CH6: Power Down
    ADS1299_Write_Reg(0x0B, 0x80); // CH7: Power Down
    ADS1299_Write_Reg(0x0C, 0x80); // CH8: Power Down

    ADS1299_Write_Reg(0x0D, 0x00); // BIAS_SENSP
    ADS1299_Write_Reg(0x0E, 0x00); // BIAS_SENSN
    ADS1299_Write_Reg(0x0F, 0x01); // LOFF_SENSP: LOFF8P? No, 0x01 is LOFF1P enabled
    ADS1299_Write_Reg(0x10, 0x00); // LOFF_SENSN
    ADS1299_Write_Reg(0x11, 0x00); // LOFF_FLIP
    ADS1299_Write_Reg(0x12, 0x00); // LOFF_STATP (RO)
    ADS1299_Write_Reg(0x13, 0x00); // LOFF_STATN (RO)
    ADS1299_Write_Reg(0x14, 0x00); // GPIO
    ADS1299_Write_Reg(0x15, 0x20); // MISC1: SRB1 connected to all N inputs (closed)
    ADS1299_Write_Reg(0x16, 0x00); // MISC2
    ADS1299_Write_Reg(0x17, 0x02); // CONFIG4: PD_LOFF_COMP enabled (Lead-Off Comparator Enabled)

    USB_Print("[TEST_IMP] Registers Configured.\r\n");
		
    USB_Print("\r\n[BOOT] ADS1299 Registers before START:\r\n");
		uint8_t regs[24];
		ADS1299_Read_Regs(0x00, regs, 24);
		for(int i=0; i<24; i++) {
				USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]);
				HAL_Delay(5);
		}

    // Start Acquisition
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    // 开始阻抗检测运行，蓝色0.5s闪烁 (中速闪烁)
    RGB_LED_SetSingle(LED_COLOR_BLUE, LED_MODE_FLASH_MID);

    app_state = APP_IMP_TEST_RUNNING; 
    start_time = HAL_GetTick();
    sample_count = 0;
    target_samples = 60 * 250; // Run for 1 minute
    last_display_time = start_time;
    current_channel = 1; // Focus on Channel 1

    // Reset Impedance Stats
    imp_max_val = -1e9;
    imp_min_val = 1e9;
    imp_window_count = 0;
    imp_sum = 0.0;
    imp_count = 0;

    strcpy(response.message, "OK: TEST_IMP Started (CH1)");
    return response;
}

cmd_response_t cmd_test_imp_all_handler(char* args) {
    cmd_response_t response = {1, ""};

    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    ADS1299_Stop();
    HAL_Delay(10);
    
    // Switch to binary protocol mode to send TYPE_IMP packets
    extern volatile uint8_t g_use_binary_protocol;
    g_use_binary_protocol = 1;

    // Manual Register Configuration for Impedance Test (All Channels)
    ADS1299_Write_Reg(0x01, 0x96); // CONFIG1: 250 SPS
    ADS1299_Write_Reg(0x02, 0xC0); // CONFIG2: Internal test signal generation
    ADS1299_Write_Reg(0x03, 0xE6); // CONFIG3: Internal reference buffer enabled, VREF = 4.5V
    ADS1299_Write_Reg(0x04, 0x05); // LOFF: 24nA Current, AC Lead-off at fDR/4 (000 0 01 01)

    // Enable all 8 channels (Gain 24, Normal Input)
    for(int i=1; i<=8; i++) {
        ADS1299_Write_Reg(0x04 + i, 0x60); 
    }

    ADS1299_Write_Reg(0x0D, 0x00); // BIAS_SENSP
    ADS1299_Write_Reg(0x0E, 0x00); // BIAS_SENSN
    ADS1299_Write_Reg(0x0F, 0xFF); // LOFF_SENSP: All P inputs enabled for LOFF
    ADS1299_Write_Reg(0x10, 0x00); // LOFF_SENSN
    ADS1299_Write_Reg(0x11, 0x00); // LOFF_FLIP
    ADS1299_Write_Reg(0x15, 0x20); // MISC1: SRB1 connected to all N inputs
    ADS1299_Write_Reg(0x17, 0x02); // CONFIG4: PD_LOFF_COMP enabled

    USB_Print("[TEST_IMP_ALL] Registers Configured (8CH).\r\n");
    
    // Read back and print registers
    USB_Print("\r\n[BOOT] ADS1299 Registers before START:\r\n");
    uint8_t regs[24];
    ADS1299_Read_Regs(0x00, regs, 24);
    
    // In binary mode, avoid spamming 24 lines of text which can cause USB buffer overflow
    extern volatile uint8_t g_use_binary_protocol;
    if (!g_use_binary_protocol) {
        for(int i=0; i<24; i++) {
            USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]);
            HAL_Delay(5);
        }
    } else {
        // Just print a summary or nothing to prevent blocking the main loop
        USB_Print("Registers dumped (omitted for binary mode).\r\n");
    }
    
    // Start Acquisition
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    // 开始阻抗检测运行，蓝色0.5s闪烁 (中速闪烁)
    RGB_LED_SetSingle(LED_COLOR_BLUE, LED_MODE_FLASH_MID);

    app_state = APP_IMP_TEST_ALL_RUNNING; 
    start_time = HAL_GetTick();
    sample_count = 0;
    target_samples = 0xFFFFFFFF; // Run indefinitely (until STOP)
    last_display_time = start_time;

    // Reset Impedance Stats
    for(int i=0; i<8; i++) {
        imp_all_max[i] = -1e9;
        imp_all_min[i] = 1e9;
    }
    imp_window_count = 0;

    strcpy(response.message, "OK: TEST_IMP_ALL Started (8CH). Send STOP to end.");
    return response;
}

cmd_response_t cmd_test_noise_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }

    ADS1299_Stop();
    HAL_Delay(10);

    // 1. Configure Registers
    // 0x00 is Read Only
    ADS1299_Write_Reg(0x01, 0x96); // CONFIG1: 250 SPS
    ADS1299_Write_Reg(0x02, 0xC0); // CONFIG2: Internal Test Signal On
    ADS1299_Write_Reg(0x03, 0xE6); // CONFIG3: Ref Buffer Enabled, VRef 4.5V
    ADS1299_Write_Reg(0x04, 0x00); // LOFF: Disabled

    // CH1-CH8: Gain 24, Input Shorted (0x61)
    for(int i=1; i<=8; i++) {
        ADS1299_Write_Reg(0x04 + i, 0x61); 
    }
    
    ADS1299_Write_Reg(0x0D, 0xFF); // BIAS_SENSP: All enabled
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

    // 2. Read Back and Print
    USB_Print("\r\n[BOOT] ADS1299 Registers after Config:\r\n");
    uint8_t regs[24];
    ADS1299_Read_Regs(0x00, regs, 24);
    for(int i=0; i<24; i++) {
        USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]);
        HAL_Delay(5);
    }

    // 3. Init Variables
    for(int i=0; i<8; i++) {
        noise_sum[i] = 0.0;
        noise_sum_sq[i] = 0.0;
        noise_min[i] = 1e9;
        noise_max[i] = -1e9;
    }

    // 4. Start Acquisition
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    app_state = APP_NOISE_TEST_RUNNING;
    start_time = HAL_GetTick();
    sample_count = 0;
    target_samples = 250; // 1 second @ 250SPS
    last_display_time = start_time;

    strcpy(response.message, "OK: TEST_NOISE started (1 sec)");
    return response;
}

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

    // 1. Configure Registers
    // 0x00 is Read Only (ID)
    ADS1299_Write_Reg(0x01, 0x96); // CONFIG1: 250 SPS
    ADS1299_Write_Reg(0x02, 0xC0); // CONFIG2: Int Test Signal ON (Internal)
    ADS1299_Write_Reg(0x03, 0xF6); // CONFIG3: VREF=4.5V, RLDREF_INT=1, PD_RLD=1, RLD_MEAS=1
    ADS1299_Write_Reg(0x04, 0x00); // LOFF: Disabled

    // CH1-CH8: Gain 24, Normal Input (0x60)
    for(int i=1; i<=8; i++) {
        ADS1299_Write_Reg(0x04 + i, 0x60); 
    }
    
    ADS1299_Write_Reg(0x0D, 0xFF); // BIAS_SENSP: All enabled
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

    // 2. Read Back and Print
    USB_Print("\r\n[BOOT] ADS1299 Registers after Config:\r\n");
    uint8_t regs[24];
    ADS1299_Read_Regs(0x00, regs, 24);
    for(int i=0; i<24; i++) {
        USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]);
        HAL_Delay(5);
    }

    // 3. Init Variables
    for(int i=0; i<8; i++) {
        cmrr_sum_sq[i] = 0.0;
    }

    // 4. Start Acquisition
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    app_state = APP_CMRR_TEST_RUNNING;
    start_time = HAL_GetTick();
    sample_count = 0;
    target_samples = 500; // 2 seconds @ 250SPS
    last_display_time = start_time;
    
    strcpy(response.message, "OK: TEST_CMRR started (2 sec)");
    return response;
}

cmd_response_t cmd_test_cmrr_full_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }

    if (args != NULL && strcmp(args, "STEP1") == 0) {
        // STEP 1: Differential Gain Test
        // Requires: 4mV Diff Signal
        USB_Print("[TEST_CMRR_FULL] STEP 1: Measuring Differential Gain.\r\n");
        USB_Print("Please Apply 4mV (1.414 mVrms) Differential Signal to all channels.\r\n");

        ADS1299_Stop();
        HAL_Delay(10);
        
        // Config for Normal Input (Gain 24)
        ADS1299_Write_Reg(0x01, 0x96); 
        ADS1299_Write_Reg(0x02, 0xC0); 
        ADS1299_Write_Reg(0x03, 0xF6); 
        ADS1299_Write_Reg(0x04, 0x00); 

        for(int i=1; i<=8; i++) ADS1299_Write_Reg(0x04 + i, 0x60); 
        
        ADS1299_Write_Reg(0x0D, 0xFF); 
        ADS1299_Write_Reg(0x0E, 0x00); 
        ADS1299_Write_Reg(0x15, 0x20); // MISC1: SRB1 connected to all N inputs
        
        // Init Vars
        for(int i=0; i<8; i++) cmrr_step1_sum_sq[i] = 0.0;
        
        ADS1299_Start();
        circular_buffer_clear(&ADS1299_circula_buffer);
        
        app_state = APP_CMRR_STEP1_RUNNING;
        start_time = HAL_GetTick();
        sample_count = 0;
        target_samples = 500; // 2 sec
        
        strcpy(response.message, "OK: STEP1 Started");
    } 
    else if (args != NULL && strcmp(args, "STEP2") == 0) {
        // STEP 2: Common Mode Gain Test
        // Requires: 2V Comm Signal
        USB_Print("[TEST_CMRR_FULL] STEP 2: Measuring Common Mode Gain.\r\n");
        USB_Print("Please Apply 2V (0.7071 Vrms) Common Mode Signal to all channels.\r\n");
        
        ADS1299_Stop();
        HAL_Delay(10);
        
        // Same Config
        ADS1299_Write_Reg(0x01, 0x96); 
        ADS1299_Write_Reg(0x02, 0xC0); 
        ADS1299_Write_Reg(0x03, 0xF6); 
        ADS1299_Write_Reg(0x04, 0x00); 
        for(int i=1; i<=8; i++) ADS1299_Write_Reg(0x04 + i, 0x60); 
        ADS1299_Write_Reg(0x0D, 0xFF); 
        ADS1299_Write_Reg(0x0E, 0x00); 
        ADS1299_Write_Reg(0x15, 0x20);

        // Init Vars
        for(int i=0; i<8; i++) cmrr_step2_sum_sq[i] = 0.0;
        
        ADS1299_Start();
        circular_buffer_clear(&ADS1299_circula_buffer);
        
        app_state = APP_CMRR_STEP2_RUNNING;
        start_time = HAL_GetTick();
        sample_count = 0;
        target_samples = 500; // 2 sec
        
        strcpy(response.message, "OK: STEP2 Started");
    }
    else {
        response.success = 0;
        strcpy(response.message, "ERR: Usage: TEST_CMRR_FULL:STEP1 or STEP2");
    }

    return response;
}

cmd_response_t cmd_get_state_handler(char* args) {
    cmd_response_t response = {1, ""};
    (void)args;

    // Update battery voltage reading before reporting
    Update_Battery_Voltage();

    // Check ADS1299 ID
    uint8_t ads_id = ADS1299_ReadRegister(0x00);
    int ads_ok = ((ads_id & 0x1F) == 0x1E) ? 1 : 0; // ADS1299 ID is typically 0x3E (rev+ID)

    // Check USB State
    int usb_ok = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1 : 0;

    USB_Print("\r\n=== DEVICE STATE REPORT ===\r\n");
    USB_Print("SN: AIT2601X04566\r\n");
    USB_Print("VERSION:T1.0.1\r\n");
    USB_Print("Battery: %.1f%%\r\n", Battery_Voltage);
    USB_Print("USB:     %s\r\n", usb_ok ? "CONNECTED" : "DISCONNECTED");
    USB_Print("Charging:  %s\r\n", usb_ok ? "CHARGING" : "DISCHARGING");
    USB_Print("SD Card: %s\r\n", sd_mounted ? "MOUNTED" : "NOT MOUNTED");
    USB_Print("ADS1299: %s (ID: 0x%02X)\r\n", ads_ok ? "OK" : "ERROR", ads_id);
    USB_Print("WM8978:  %s\r\n", wm8978_initialized ? "OK" : "ERROR");
    
    // Detailed ADS1299 Registers (Optional, keep it concise or expand if needed)
    // For now just ID status is good for "State"
    
    USB_Print("=== END REPORT ===\r\n");

    strcpy(response.message, "OK: State report sent");
    return response;
}

cmd_response_t cmd_playmusic_handler(char* args) {
    cmd_response_t response = {1, ""};
    if (args == NULL || args[0] == '\0') {
        response.success = 0;
        strcpy(response.message, "ERR: Missing filename");
        return response;
    }
    
    // Check if other app is running
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: App busy");
        return response;
    }

    char path[128];
    // If user provides "1", make it "AUDIO/1.wav"
    // If user provides "1.wav", make it "AUDIO/1.wav"
    
    // Trim spaces
    while (*args == ' ') args++;
    
    USB_Print("[CMD] PLAYMUSIC args: '%s'\r\n", args);

    // Construct path
    if (strstr(args, ".wav") || strstr(args, ".WAV")) {
        sprintf(path, "AUDIO/%s", args);
    } else {
        sprintf(path, "AUDIO/%s.wav", args);
    }
    
    USB_Print("[CMD] Playing path: '%s'\r\n", path);

    wav_loop_mode = 0; // Play once
    
    uint8_t res = Wav_Play(path);
    if (res == 0) {
        sprintf(response.message, "OK: Playing %s", path);
    } else {
        response.success = 0;
        sprintf(response.message, "ERR: Play failed (%d)", res);
    }
    
    return response;
}

cmd_response_t cmd_rx_test_handler(char* args) {
    cmd_response_t response = {1, ""};
    
    // Trim spaces
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
        // Just report status
        sprintf(response.message, "RX Test Mode: %s, Count: %lu", 
                g_rx_test_mode ? "ON" : "OFF", g_rx_test_count);
    }
    
    return response;
}

cmd_response_t cmd_rld_test_handler(char* args) {
    cmd_response_t response = {1, ""};

    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    ADS1299_Stop();
    HAL_Delay(10);

    // Manual Register Configuration for RLD Test as per user request
    // 0x00 ID (Read Only) - Skipped or verified later
    ADS1299_Write_Reg(0x01, 0x96); // CONFIG1: 250 SPS
    ADS1299_Write_Reg(0x02, 0xC0); // CONFIG2: Internal Test Signal On
    ADS1299_Write_Reg(0x03, 0xEC); // CONFIG3: PD_REFBUF=1, VREF=4V, RLD_MEAS=1, RLDREF_INT=1, PD_RLD=1, RLD_LOFF=0, RLD_STAT=0
    ADS1299_Write_Reg(0x04, 0x00); // LOFF: Disabled

    // CHnSET: 0x60 (Gain 24, Normal Input)
    for(int i=0; i<8; i++) {
        ADS1299_Write_Reg(0x05 + i, 0x60); 
    }

    ADS1299_Write_Reg(0x0D, 0xFF); // BIAS_SENSP: All channels connected to Bias (RLD)
    ADS1299_Write_Reg(0x0E, 0xFF); // BIAS_SENSN: All channels connected to Bias (RLD)
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
    
    // Read back verification (optional, can be helpful)
    USB_Print("[BOOT] ADS1299 Registers:\r\n");
    uint8_t regs[24];
    ADS1299_Read_Regs(0x00, regs, 24);
    for(int i=0; i<24; i++) {
        USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]);
        HAL_Delay(2);
    }

    // Initialize stats
    for(int i=0; i<8; i++) {
        rld_sum[i] = 0.0;
        rld_sum_sq[i] = 0.0;
    }

    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    app_state = APP_RLD_TEST_RUNNING;
    start_time = HAL_GetTick();
    sample_count = 0;
    target_samples = 10 * 250; // 10 seconds @ 250 SPS
    last_display_time = start_time;

    strcpy(response.message, "OK: RLD_TEST Started. Please wait 10s...");
    return response;
}

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
  register_command("SWEEP", cmd_sweep_handler, "SWEEP", "AD5933 Freq Sweep");
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
