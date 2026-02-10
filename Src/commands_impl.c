/* commands_impl.c */
#include "commands_impl.h"
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

extern FATFS SDFatFS;
extern FIL SDFile;
extern CircularBuffer ADS1299_circula_buffer;
extern uint8_t sd_mounted;
extern USBD_HandleTypeDef hUsbDeviceFS;

// External functions/variables from main.c
extern float Battery_Voltage;
extern void Update_Battery_Voltage(void);
extern uint8_t wm8978_initialized;

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

cmd_response_t cmd_single_handler(char* args) {
    cmd_response_t response = {1, ""};
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    int channel = 1;
    int duration = 60; // 1 minute default
    int sps = 250;

    current_channel = channel;
    current_sps = sps;
    target_samples = duration * sps;

    // Configure ADS1299
    ADS1299_Stop();
    // 0: 250SPS, 6: Gain 24, 0: Normal Input
    ADS1299_Set_Sample_Parameter(0, 6, 0);
    
    // Enable single channel (CH1), others shorted
    for(int i=1; i<=8; i++) {
        uint8_t reg = 0x04 + i;
        if (i == channel) ADS1299_Write_Reg(reg, 0x60); // Normal input, Gain 24
        else ADS1299_Write_Reg(reg, 0x81); // Disable others
    }
    
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    app_state = APP_SINGLE_RUNNING;
    start_time = HAL_GetTick();
    sample_count = 0;
    last_save_time = start_time;
    last_display_time = start_time;

    strcpy(response.message, "OK: SINGLE started (1 min, CH1)");
    return response;
}

cmd_response_t cmd_quad_handler(char* args) {
    cmd_response_t response = {1, ""};
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    // Open file for recording
    FRESULT fr = f_open(&SDFile, "EEG_DATA.TXT", FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        response.success = 0;
        sprintf(response.message, "ERR: Failed to open EEG_DATA.TXT (%d)", fr);
        return response;
    }

    int duration = 60 * 60; // 1 hour max (safety limit)
    int sps = 250;
    
    current_sps = sps;
    target_samples = duration * sps;

    // Configure ADS1299 (User provided config)
    ADS1299_Stop();
    ADS1299_Set_Sample_Parameter(6, 6, 0); // 250SPS, Gain 24, Normal Input 
   
    // Enable all 8 channels 
    for(int i=1; i<=8; i++) { 
       ADS1299_Write_Reg(0x04 + i, 0x60); // Gain 24, Normal Input 
    } 
   
    USB_Print("\r\n[BOOT] ADS1299 Registers before START:\r\n"); 
    uint8_t regs[24]; 
    ADS1299_Read_Regs(0x00, regs, 24); 
    for(int i=0; i<24; i++) { 
       USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]); 
       HAL_Delay(5); 
    } 
    
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    app_state = APP_QUAD_RUNNING; // Reuse QUAD state for 8-channel recording
    start_time = HAL_GetTick();
    sample_count = 0;
    last_save_time = start_time;
    last_display_time = start_time;

    strcpy(response.message, "OK: QUAD(8CH) started. Recording to EEG_DATA.TXT");
    return response;
}

cmd_response_t cmd_stream_handler(char* args) {
    cmd_response_t response = {1, ""};
    if (app_state != APP_IDLE) {
        response.success = 0;
        strcpy(response.message, "ERR: Acquisition already running");
        return response;
    }

    int duration = 60 * 60; // 1 hour max
    int sps = 250;
    
    current_sps = sps;
    target_samples = duration * sps;

    // Configure ADS1299
    ADS1299_Stop();
    ADS1299_Set_Sample_Parameter(6, 6, 0); // 250SPS, Gain 24, Normal Input 
   
    // Enable all 8 channels 
    for(int i=2; i<=8; i++) { 
       ADS1299_Write_Reg(0x04 + i, 0x60); // Gain 24, Normal Input 
    } 
   
    USB_Print("\r\n[BOOT] ADS1299 Registers before START:\r\n"); 
    uint8_t regs[24]; 
    ADS1299_Read_Regs(0x00, regs, 24); 
    for(int i=0; i<24; i++) { 
       USB_Print("Reg[0x%02X] = 0x%02X\r\n", i, regs[i]); 
       HAL_Delay(5); 
    } 
    
    // Play 3.wav before starting collection
    Wav_Play("AUDIO/3.wav");
    while(is_playing) {
        Wav_Process();
    }
    
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    app_state = APP_STREAM_RUNNING;
    start_time = HAL_GetTick();
    stream_seq_num = 0;
    sample_count = 0;
    last_save_time = start_time;
    last_display_time = start_time;

    strcpy(response.message, "OK: STREAM(8CH) started.");
    return response;
}

cmd_response_t cmd_stop_handler(char* args) {
    cmd_response_t response = {1, ""};
    if (app_state == APP_IDLE) {
        strcpy(response.message, "OK: Already stopped");
        return response;
    }
    app_state = APP_STOP_REQUESTED;
    strcpy(response.message, "OK: Stopping...");
    return response;
}

void app_run_loop(void) {
    if (app_state == APP_IDLE) return;

    if (app_state == APP_STOP_REQUESTED) {
        ADS1299_Stop();
        if (app_state == APP_QUAD_RUNNING) {
            f_close(&SDFile);
            USB_Print("File closed.\r\n");
        }
        app_state = APP_IDLE;
        
        // Play 4.wav after stopping collection
        Wav_Play("AUDIO/4.wav");
        while(is_playing) {
            Wav_Process();
        }
        
        USB_Print("Acquisition Stopped. Saved %lu samples.\r\n", sample_count);
        return;
    }

    if (app_state == APP_SINGLE_RUNNING || app_state == APP_QUAD_RUNNING || app_state == APP_STREAM_RUNNING || app_state == APP_IMP_TEST_RUNNING || app_state == APP_NOISE_TEST_RUNNING || app_state == APP_CMRR_TEST_RUNNING || app_state == APP_CMRR_STEP1_RUNNING || app_state == APP_CMRR_STEP2_RUNNING || app_state == APP_IMP_TEST_ALL_RUNNING || app_state == APP_RLD_TEST_RUNNING) 
			{
        
        // --- AUX Data Handling (1Hz) for STREAM mode ---
        if (app_state == APP_STREAM_RUNNING) {
            static uint32_t last_aux_time = 0;
            if (HAL_GetTick() - last_aux_time >= 1000) {
                float acc[3], gyro[3];
                // Read IMU
                QMI8658_Read_XYZ(acc, gyro);
                // Read Battery
                Update_Battery_Voltage();
                
                // Send AUX Packet: AUX:accx,accy,accz,gyrox,gyroy,gyroz,vbat
                // Note: QMI8658_Read_XYZ returns g for acc and dps for gyro.
                // Assuming Send_AUX_Packet format from main.c reference:
                // "AUX:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n"
                char aux_line[128];
                int aux_len = snprintf(aux_line, sizeof(aux_line), 
                    "AUX:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", 
                    acc[0], acc[1], acc[2], 
                    gyro[0], gyro[1], gyro[2], 
                    Battery_Voltage);
                CDC_Transmit_FS((uint8_t*)aux_line, aux_len);
                
                last_aux_time = HAL_GetTick();
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
            circular_buffer_read(&ADS1299_circula_buffer, packet, 27);
            
            static char line[256];
            int len = 0;
            
            // Gain 24, Vref=4.5V
            // 4.5V / (2^23 - 1) = 0.53644 uV (LSB at Gain=1)
            // scale unit: uV / LSB
            float scale = 0.5364f / 24.0f; 

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
                                  i+1, cmrr_diff_gain[i], comm_gain, cmrr_db);
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

                // Calculate Impedance every 1 second (250 samples)
                if (imp_window_count >= 250) {
                    USB_Print("[IMP] ");
                    for(int i=0; i<8; i++) {
                        float vpp = imp_all_max[i] - imp_all_min[i]; 
                        // Z = Vpp(uV) * 1e-6 / (2 * 6nA) = Vpp / 0.012
                        float impedance = (vpp * 1e-6f) / 12.0e-9f;
                        
                        USB_Print("CH%d:%.1fk ", i+1, impedance / 1000.0f);
                        
                        // Reset
                        imp_all_max[i] = -1e9;
                        imp_all_min[i] = 1e9;
                    }
                    USB_Print("\r\n");
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
            } else if (app_state == APP_STREAM_RUNNING) {
                float voltages[8];
                for(int i=0; i<8; i++) {
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    voltages[i] = (float)val * scale;
                }
                
                // Format: EEG:SEQ:x,v1,v2,v3,v4,v5,v6,v7,v8\n
								len = snprintf(line, sizeof(line), "EEG:F5:%.2f,F3:%.2f,FZ:%.2f,F4:%.2f,F6:%.2f,AF8:%.2f,FPZ:%.2f,AF7:%.2f\r\n",
                               voltages[1], voltages[1], voltages[2], voltages[3],
                               voltages[4], voltages[5], voltages[6], voltages[7]);//复制了一份通道二给通道1
//                len = snprintf(line, sizeof(line), "EEG:SEQ:%lu,F5:%.2f,F3:%.2f,FZ:%.2f,F4:%.2f,F6:%.2f,AF8:%.2f,FPZ:%.2f,AF7:%.2f\r\n",
//                               stream_seq_num++,
//                               voltages[0], voltages[1], voltages[2], voltages[3],
//                               voltages[4], voltages[5], voltages[6], voltages[7]);
//								len = snprintf(line, sizeof(line), "EEG:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
//                               voltages[0], voltages[1], voltages[2], voltages[3],
//                               voltages[4], voltages[5], voltages[6], voltages[7]);
                
                // Transmit with simple retry for reliability
                if (CDC_Transmit_FS((uint8_t*)line, len) == USBD_BUSY) {
                    uint32_t retry_start = HAL_GetTick();
                    while (CDC_Transmit_FS((uint8_t*)line, len) == USBD_BUSY) {
                        if (HAL_GetTick() - retry_start > 2) break; // 2ms timeout
                    }
                }
            } else {
                // QUAD (8-channel) mode
                float voltages[8];
                for(int i=0; i<8; i++) {
                    int ch_offset = 3 + i * 3;
                    int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                    if (val & 0x800000) val |= 0xFF000000;
                    voltages[i] = (float)val * scale;
                }
                
                // Format: v1,v2,v3,v4,v5,v6,v7,v8\n
                len = snprintf(line, sizeof(line), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                               voltages[0], voltages[1], voltages[2], voltages[3],
                               voltages[4], voltages[5], voltages[6], voltages[7]);
                
                // Write to SD Card
                UINT bw;
                FRESULT fr = f_write(&SDFile, line, len, &bw);
                if (fr != FR_OK || bw < len) {
                    USB_Print("SD Write Err: %d\r\n", fr);
                    app_state = APP_STOP_REQUESTED; // Stop on write error
                } else {
                    // Sync every ~4KB or periodically (here we use sample count for simplicity)
                    // 250 SPS, approx 60-80 bytes per line -> ~15-20KB/s
                    // Sync every 50 samples (~0.2s) to be safe against power loss
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

    USB_Print("[CMD] Starting AD5933 Sweep...\r\n");
    AD5933_Sweep();
    
    strcpy(response.message, "OK: Sweep Completed");
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

    // Manual Register Configuration for Impedance Test (All Channels)
    ADS1299_Write_Reg(0x01, 0x96); // CONFIG1: 250 SPS
    ADS1299_Write_Reg(0x02, 0xC0); // CONFIG2: Internal test signal generation
    ADS1299_Write_Reg(0x03, 0xE6); // CONFIG3: Internal reference buffer enabled, VREF = 4.5V
    ADS1299_Write_Reg(0x04, 0x01); // LOFF: 24nA Current, AC Lead-off at fDR/4

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
    
    // Start Acquisition
    ADS1299_Start();
    circular_buffer_clear(&ADS1299_circula_buffer);

    app_state = APP_IMP_TEST_ALL_RUNNING; 
    start_time = HAL_GetTick();
    sample_count = 0;
    target_samples = 60 * 250; // Default 1 min
    last_display_time = start_time;

    // Reset Impedance Stats
    for(int i=0; i<8; i++) {
        imp_all_max[i] = -1e9;
        imp_all_min[i] = 1e9;
    }
    imp_window_count = 0;

    strcpy(response.message, "OK: TEST_IMP_ALL Started (8CH)");
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
    USB_Print("VERSION:T1.0.0\r\n");
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

  register_command("SINGLE", cmd_single_handler, "SINGLE", "Single channel acquisition (1 min)");
  register_command("QUAD", cmd_quad_handler, "QUAD", "4 channel acquisition (1 min)");
  register_command("STREAM", cmd_stream_handler, "STREAM", "8 channel stream (1 hour)");
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

    // USB_Print("[CMD] Command system: Registered %d commands\r\n", 15);
}
