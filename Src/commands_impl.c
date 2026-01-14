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

extern FATFS SDFatFS;
extern FIL SDFile;
extern CircularBuffer ADS1299_circula_buffer;
extern uint8_t sd_mounted;
extern USBD_HandleTypeDef hUsbDeviceFS;

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
        USB_Print("Acquisition Stopped. Saved %lu samples.\r\n", sample_count);
        return;
    }

    if (app_state == APP_SINGLE_RUNNING || app_state == APP_QUAD_RUNNING) {
        // Check timeout
        if (sample_count >= target_samples) {
            app_state = APP_STOP_REQUESTED;
            return;
        }

        uint8_t packet[27];
        // ADS1299 packet: 3 bytes status + 8 * 3 bytes data = 27 bytes
        while (circular_buffer_get_data_count(&ADS1299_circula_buffer) >= 27) {
            circular_buffer_read(&ADS1299_circula_buffer, packet, 27);
            
            char line[256];
            int len = 0;
            
            // Gain 24, Vref=4.5V
            float scale = 0.5364f / 24.0f; // 4.5 / (2^23 - 1) / 24

            if (app_state == APP_SINGLE_RUNNING) {
                int ch_offset = 3 + (current_channel - 1) * 3;
                int32_t val = (packet[ch_offset] << 16) | (packet[ch_offset+1] << 8) | packet[ch_offset+2];
                if (val & 0x800000) val |= 0xFF000000;
                float voltage = (float)val * scale;
                
                len = snprintf(line, sizeof(line), "%.2f\n", voltage);
                // Note: SINGLE mode currently does not write to SD, as requested only for QUAD
                CDC_Transmit_FS((uint8_t*)line, len);
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
                
                // Optional: Monitor via USB (can be removed if too slow)
                // CDC_Transmit_FS((uint8_t*)line, len); 
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

void register_all_commands(void) {
  command_system_init();

  register_command("LIST", cmd_list_handler, "LIST", "List all .txt files");
  register_command("GET", cmd_get_handler, "GET:<filename>", "Send file");
  register_command("DIAG", cmd_diag_handler, "DIAG", "SD card diagnosis");
  register_command("HELP", cmd_help_handler, "HELP", "Display command help");
  register_command("CREATELARGE", cmd_createlarge_handler, "CREATELARGE", "Create large test file");
  register_command("DELETE", cmd_delete_handler, "DELETE:<filename>", "Delete file");
  register_command("INFO", cmd_info_handler, "INFO:<filename>", "Display file information");

  register_command("SINGLE", cmd_single_handler, "SINGLE", "Single channel acquisition (1 min)");
  register_command("QUAD", cmd_quad_handler, "QUAD", "4 channel acquisition (1 min)");
  register_command("STOP", cmd_stop_handler, "STOP", "Stop acquisition");
  register_command("REG", cmd_reg_handler, "REG", "Read ADS1299 registers");

  USB_Print("[CMD] Command system: Registered %d commands\r\n", 10);
}
