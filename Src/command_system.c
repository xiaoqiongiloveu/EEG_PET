/* command_system.c */
#include "command_system.h"
#include "main.h"
#include "binary_protocol.h" // New Protocol
#include "fatfs.h"
#include <stdio.h>
#include "usbd_cdc_if.h"

// Forward declare static variables if needed, or include headers
// For file listing
static DIR dir;
static FILINFO fno;

/* Private global variables */
static command_entry_t* command_table = NULL;
static int command_count = 0;
static int table_capacity = 0;

/* USB print functions - defined in main or usbd_cdc_if */
/* We will assume they are available or we might need to add them */
#ifdef __cplusplus
extern "C" {
#endif
/* Forward declaration if not in header */
void USB_Print(const char* format, ...);
void USB_SendString(const char* str);
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len); // Needed for binary response
#ifdef __cplusplus
}
#endif

// --- Binary Protocol Handler ---

#include "commands_impl.h" // For app_state, APP_STREAM_RUNNING

// Global Flag for Pending Binary Command
volatile uint8_t g_pending_binary_cmd = 0;
volatile uint8_t g_audio_route = 0; // 0=Both, 1=Speaker, 2=Headphone
// Buffer to hold filename for GET_FILE / DELETE_FILE commands (deferred)
char g_pending_filename[64] = {0};
// Buffer for OTA Start Size
uint32_t g_pending_ota_size = 0;
// Buffer for OTA CRC32
uint32_t g_pending_ota_crc = 0;

// Helper to send binary response
void Write_Log_To_SD(const char* log_str) {
    // [FIX] 直接返回，彻底禁止空闲状态下写入或创建 0 字节的 CMD_LOG.TXT 文件
    return;
    
    // 1. 安全检查：只有空闲状态且 SD 卡挂载时才允许写日志
    extern volatile uint8_t g_task_state;
    extern uint8_t sd_mounted;
    extern FIL SDFile;
    
    // 我们用 g_task_state == 0 来表示真正的空闲状态 (0=空闲, 1=倒计时, 2=采集中)
    if (g_task_state != 0 || !sd_mounted) return;

    // 2. 以“追加模式”打开日志文件
    if (f_open(&SDFile, "CMD_LOG.TXT", FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
        // 3. 写入带时间戳的日志
        char timestamp_str[32];
        int ts_len = snprintf(timestamp_str, sizeof(timestamp_str), "[%lu] ", HAL_GetTick());
        
        UINT bw;
        // 暂时注释掉空闲状态下的SD卡日志写入
        // f_write(&SDFile, timestamp_str, ts_len, &bw);
        // f_write(&SDFile, log_str, strlen(log_str), &bw);
        // f_write(&SDFile, "\r\n", 2, &bw);
        
        // 4. 关闭文件
        f_close(&SDFile);
    }
}

void BP_SendResponse(uint8_t cmd_id, uint8_t status, const char* msg) {
    uint8_t payload[64];
    payload[0] = cmd_id;
    payload[1] = status;
    int msg_len = strlen(msg);
    if (msg_len > 60) msg_len = 60;
    memcpy(&payload[2], msg, msg_len);
    
    uint8_t tx_buf[128];
    uint16_t len = BP_Pack(tx_buf, BP_TYPE_CMD_RESP, 0, payload, 2 + msg_len);
    CDC_Transmit_FS(tx_buf, len);
    
    // ----- 记录二进制命令响应日志 -----
    // char log_buf[128];
    // snprintf(log_buf, sizeof(log_buf), "[BIN] RX_CMD: 0x%02X -> TX_ACK: %s (Status: %d)", cmd_id, msg ? msg : "NULL", status);
    // Write_Log_To_SD(log_buf);
    // ---------------------------------
}

// Callback from parser
void BP_Command_Callback(uint8_t type, uint16_t seq, uint32_t ts, uint8_t* payload, uint16_t len) {
    if (type == BP_TYPE_CMD) {
        if (len < 1) return;
        uint8_t cmd_id = payload[0];
        // uint8_t* args = &payload[1];
        
        switch (cmd_id) {
            case BP_CMD_SET_CONF:
                 // Struct: SPS(2) Gain(1) Mask(1) Input(1) Bias(1) TestSig(1) Dur(4) = 11 Bytes Args (Total 12)
                 if (len >= 12) {
                     extern volatile uint8_t g_task_state;
                     if (g_task_state == 1 || g_task_state == 2) { // PENDING or RUNNING
                         BP_SendResponse(cmd_id, BP_RESP_ERR, "Task Pending/Running");
                         break;
                     }
                     
                     uint16_t sps;
                     memcpy(&sps, &payload[1], 2);
                     uint8_t gain = payload[3];
                     uint8_t mask = payload[4];
                     uint8_t input = payload[5];
                     uint8_t bias = payload[6];
                     uint8_t test = payload[7];
                     uint32_t dur;
                     memcpy(&dur, &payload[8], 4);
                     
                     // Update Globals
                     extern uint8_t g_cfg_gain;
                     extern uint8_t g_cfg_ch_mask;
                     extern int g_cfg_sps;
                     extern uint8_t g_cfg_input_type;
                     extern uint8_t g_cfg_bias_en;
                     extern uint32_t g_cfg_duration_sec; 
                     extern uint8_t g_cfg_test_sig;      
                     
                     g_cfg_sps = sps;
                     g_cfg_gain = gain;
                     g_cfg_ch_mask = mask;
                     g_cfg_input_type = input;
                     g_cfg_bias_en = bias;
                     g_cfg_duration_sec = dur;
                     g_cfg_test_sig = test; // 0=Ext(Default), 1=Int
                     
                     // Check for extended task scheduling parameters (12 + 2 + 4 = 18 bytes)
                     if (len >= 18) {
                         extern uint32_t g_task_id;
                         extern uint32_t g_task_start_tick;
                         extern uint32_t g_task_delay_ms;
                         
                         uint16_t delay_min;
                         memcpy(&delay_min, &payload[12], 2);
                         uint32_t task_id;
                         memcpy(&task_id, &payload[14], 4);
                         
                         if (delay_min > 0 || task_id > 0) { // If task ID or delay provided
                             g_task_id = task_id;
                             g_task_delay_ms = delay_min * 60000;
                             g_task_start_tick = HAL_GetTick();
                             g_task_state = 1; // PENDING
                             
                             // Sync Task State (1=Pending) to Backup Register
                             BackupReg_Write(BKP_REG_TASK_STATE, 1);
                             BackupReg_Write(BKP_REG_TASK_ID, g_task_id);
                             
                             BP_SendResponse(cmd_id, BP_RESP_OK, "Task Scheduled");
                             break;
                         }
                     }
                     
                     BP_SendResponse(cmd_id, BP_RESP_OK, "Config Updated");
                 } else {
                     BP_SendResponse(cmd_id, BP_RESP_ERR, "Len Err");
                 }
                 break;

            case BP_CMD_GET_STATE:
            {
                 g_pending_binary_cmd = BP_CMD_GET_STATE;
                 break;
            }
            
            case BP_CMD_START_STREAM:
                 g_pending_binary_cmd = BP_CMD_START_STREAM;
                 BP_SendResponse(cmd_id, BP_RESP_OK, "Stream Start Pending");
                 break;
                     
            case BP_CMD_STOP_STREAM:
                 // Check if there is a pending task, cancel it
                 {
                     extern volatile uint8_t g_task_state;
                     if (g_task_state == 1) { // 1 = PENDING
                         g_task_state = 0; // NONE
                         extern uint32_t g_task_id;
                         g_task_id = 0;
                         
                         // Clear Task State in Backup Register
                         BackupReg_Write(BKP_REG_TASK_STATE, 0);
                         BackupReg_Write(BKP_REG_TASK_ID, 0);
                         
                         BP_SendResponse(cmd_id, BP_RESP_OK, "Pending Task Cancelled");
                         break;
                     }
                 }
                 
                 // Otherwise, proceed to stop running stream
                 g_pending_binary_cmd = BP_CMD_STOP_STREAM;
                 BP_SendResponse(cmd_id, BP_RESP_OK, "Stream Stop Pending");
                 break;

            case BP_CMD_ENABLE_DEVICE:
                 {
                     extern volatile uint8_t g_task_state;
                     g_task_state = 0; // Set device back to idle/ready state
                     g_pending_binary_cmd = BP_CMD_ENABLE_DEVICE;
                     // BP_SendResponse is deferred to main loop to avoid double response
                 }
                 break;

            case BP_CMD_DISABLE_DEVICE:
                 {
                     extern volatile uint8_t g_task_state;
                     g_task_state = 99; // Set device to disabled state
                     g_pending_binary_cmd = BP_CMD_DISABLE_DEVICE;
                     // BP_SendResponse is deferred to main loop to avoid double response
                 }
                 break;

            case BP_CMD_AUTO_STREAM:
                 g_pending_binary_cmd = BP_CMD_AUTO_STREAM;
                 BP_SendResponse(cmd_id, BP_RESP_OK, "Auto Stream Pending");
                 break;

            case BP_CMD_AUDIO_ROUTE:
                 if (len >= 2) {
                     extern volatile uint8_t g_audio_route;
                     g_audio_route = payload[1]; // 0=Both, 1=Speaker, 2=Headphone
                     g_pending_binary_cmd = BP_CMD_AUDIO_ROUTE;
                     BP_SendResponse(cmd_id, BP_RESP_OK, "Audio Route Pending");
                 } else {
                     BP_SendResponse(cmd_id, BP_RESP_ERR, "Missing Route Param");
                 }
                 break;

            case BP_CMD_TEST_IMP:
                 g_pending_binary_cmd = BP_CMD_TEST_IMP; // Set pending impedance test
                 BP_SendResponse(cmd_id, BP_RESP_OK, "Impedance Test Pending");
                 break;

            case BP_CMD_LIST_FILES:
                 g_pending_binary_cmd = BP_CMD_LIST_FILES;
                 BP_SendResponse(cmd_id, BP_RESP_OK, "Listing Files...");
                 break;

            case BP_CMD_CLEAR_DATA:
                 g_pending_binary_cmd = BP_CMD_CLEAR_DATA;
                 BP_SendResponse(cmd_id, BP_RESP_OK, "Clearing SD Data...");
                 break;
                 
            case BP_CMD_PAUSE:
                 g_pending_binary_cmd = BP_CMD_PAUSE;
                 BP_SendResponse(cmd_id, BP_RESP_OK, "Pause Pending");
                 break;
                 
            case BP_CMD_RESUME:
                 g_pending_binary_cmd = BP_CMD_RESUME;
                 BP_SendResponse(cmd_id, BP_RESP_OK, "Resume Pending");
                 break;
                 
            case BP_CMD_STOP_IMP:
                 g_pending_binary_cmd = BP_CMD_STOP_IMP;
                 BP_SendResponse(cmd_id, BP_RESP_OK, "Stop Imp Pending");
                 break;
                 
            case BP_CMD_GET_FILE:
                 if (len > 1) {
                     // Payload: CMD_ID(1) + Filename(N)
                     uint8_t fname_len = len - 1;
                     if (fname_len > 63) fname_len = 63;
                     memcpy(g_pending_filename, &payload[1], fname_len);
                     g_pending_filename[fname_len] = '\0';
                     g_pending_binary_cmd = BP_CMD_GET_FILE;
                     BP_SendResponse(cmd_id, BP_RESP_OK, "Get File Pending");
                 } else {
                     BP_SendResponse(cmd_id, BP_RESP_ERR, "Missing Filename");
                 }
                 break;
                 
            case BP_CMD_DELETE_FILE:
                 if (len > 1) {
                     uint8_t fname_len = len - 1;
                     if (fname_len > 63) fname_len = 63;
                     memcpy(g_pending_filename, &payload[1], fname_len);
                     g_pending_filename[fname_len] = '\0';
                     g_pending_binary_cmd = BP_CMD_DELETE_FILE;
                     BP_SendResponse(cmd_id, BP_RESP_OK, "Delete File Pending");
                 } else {
                     BP_SendResponse(cmd_id, BP_RESP_ERR, "Missing Filename");
                 }
                 break;
                 
            case BP_CMD_OTA_START:
                 // Legacy OTA does not use binary protocol for start.
                 // We keep this just in case, but it might not be needed.
                 // Or we can disable it to force ASCII mode.
                 // For now, let's leave it but ensure the handler is consistent.
                 if (len >= 5) { // CMD(1) + SIZE(4)
                     memcpy(&g_pending_ota_size, &payload[1], 4);
                     if (len >= 9) memcpy(&g_pending_ota_crc, &payload[5], 4);
                     else g_pending_ota_crc = 0;
                     
                     g_pending_binary_cmd = BP_CMD_OTA_START;
                     BP_SendResponse(cmd_id, BP_RESP_OK, "OTA Start Pending");
                 } else {
                     BP_SendResponse(cmd_id, BP_RESP_ERR, "Missing Size");
                 }
                 break;
                 
            case BP_CMD_OTA_APPLY:
                 // Map binary apply to legacy apply (Shutdown)
                 g_pending_binary_cmd = BP_CMD_OTA_APPLY;
                 BP_SendResponse(cmd_id, BP_RESP_OK, "OTA Apply Pending");
                 break;
                 
            default:
                // BP_SendResponse(cmd_id, BP_RESP_ERR, "Unknown CMD");
                break;
        }
    }
}

/* Initialize command system */
void command_system_init(void) {
    table_capacity = 10;
    command_table = (command_entry_t*)malloc(table_capacity * sizeof(command_entry_t));
    
    if (command_table == NULL) {
        // USB_Print("[CMD] Failed to allocate command table memory\r\n");
        return;
    }
    
    command_count = 0;

    /* Register system commands */
    /* Note: These handlers should be declared in a header or extern'd here */
    extern cmd_response_t cmd_ota_apply_handler(char*);
    extern cmd_response_t cmd_power_test_handler(char*);
    extern cmd_response_t cmd_adc_test_handler(char*);
    extern cmd_response_t cmd_stop_handler(char*);

    register_command("OTA_APPLY", cmd_ota_apply_handler, "OTA_APPLY", "Apply OTA Update (Restart)");
    register_command("POWER_TEST", cmd_power_test_handler, "POWER_TEST", "Enable all peripherals for power test");
    register_command("ADCTEST", cmd_adc_test_handler, "ADCTEST", "Log Battery ADC to SD card (0.5s)");
    register_command("STOP", cmd_stop_handler, "STOP", "Stop running command");
    // Deleted "QUAD" text command registration
    // Deleted "SINGLE" text command registration
    // Deleted "CHECK_LOSS" text command registration

    // USB_SendString("[CMD] Command system initialized successfully\r\n");
}

/* Register command */
void register_command(const char* name, command_handler_t handler, 
                      const char* syntax, const char* description) {
    if (command_table == NULL) return;

    for (int i = 0; i < command_count; i++) {
        if (strcmp(command_table[i].name, name) == 0) {
            // USB_Print("[CMD] Warning: Command '%s' already exists, skipping registration\r\n", name);
            return;
        }
    }

    if (command_count >= table_capacity) {
        table_capacity *= 2;
        command_entry_t* new_table = (command_entry_t*)realloc(command_table, 
            table_capacity * sizeof(command_entry_t));
        if (new_table == NULL) {
            // USB_Print("[CMD] Failed to expand command table\r\n");
            return;
        }
        command_table = new_table;
    }

    command_table[command_count].name = name;
    command_table[command_count].handler = handler;
    command_table[command_count].syntax = syntax;
    command_table[command_count].description = description;
    command_count++;
    
    // USB_Print("[CMD] Registered command: %s\r\n", name);
}

/* Find command */
const command_entry_t* find_command(const char* name) {
    if (command_table == NULL) return NULL;
    for (int i = 0; i < command_count; i++) {
        if (strcmp(command_table[i].name, name) == 0) {
            return &command_table[i];
        }
    }
    return NULL;
}

/* Process command */
cmd_response_t process_command_ex(char* input) {
    cmd_response_t response = {0, ""};
    
    if (input == NULL || input[0] == '\0') {
        response.success = 0;
        strcpy(response.message, "ERR:111");
        return response;
    }

    int len = strlen(input);
    while (len > 0 && (input[len-1] == '\r' || input[len-1] == '\n' || input[len-1] == ' ')) {
        input[len-1] = '\0';
        len--;
    }

    char* args = NULL;
    char command_name[64] = {0};
    
    char* colon = strchr(input, ':');
    if (colon != NULL) {
        int name_len = colon - input;
        if (name_len >= sizeof(command_name)) {
            response.success = 0;
            strcpy(response.message, "ERR:2222");
            return response;
        }
        strncpy(command_name, input, name_len);
        command_name[name_len] = '\0';
        args = colon + 1;
    } else {
        strncpy(command_name, input, sizeof(command_name)-1);
        args = "";
    }
    
    for (int i = 0; command_name[i]; i++) {
        command_name[i] = toupper(command_name[i]);
    }

    const command_entry_t* cmd = find_command(command_name);
    if (cmd == NULL) {
        response.success = 0;
        // USB_Print(response.message, sizeof(response.message), "ERR:333 '%s'", command_name);
        sprintf(response.message, "ERR:333 '%s'", command_name);
        return response;
    }

    // USB_Print("[CMD] Executing: %s", cmd->name);
    if (args[0] != '\0') {
        // USB_SendString(" Arguments: ");
        // USB_SendString(args);
    }
    // USB_SendString("\r\n");

    response = cmd->handler(args);
    
    // Print response message to USB if exists
    if (response.message[0] != '\0') {
        USB_Print("%s\r\n", response.message);
    }

    // ----- 记录文本命令日志 -----
    // char log_buf[128];
    // snprintf(log_buf, sizeof(log_buf), "[TXT] RX: %s -> TX: %s", command_name, response.message);
    // Write_Log_To_SD(log_buf);
    // ----------------------------

    return response;
}

/* Print all commands */
void print_all_commands(void) {
    if (command_table == NULL) return;

    USB_SendString("\r\n===== Command List (");
    char count_buf[16];
    sprintf(count_buf, "%d", command_count);
    USB_SendString(count_buf);
    USB_SendString(") =====\r\n");

    /* Bubble sort for display */
    /* Note: Modifying the table order might affect index based access if any, but finding is by name */
    for (int i = 0; i < command_count - 1; i++) {
        for (int j = 0; j < command_count - i - 1; j++) {
            if (strcmp(command_table[j].name, command_table[j+1].name) > 0) {
                command_entry_t temp = command_table[j];
                command_table[j] = command_table[j+1];
                command_table[j+1] = temp;
            }
        }
    }

    for (int i = 0; i < command_count; i++) {
        // USB_Print("%-12s - %s\r\n", command_table[i].name, command_table[i].description);
        USB_SendString(command_table[i].name);
        USB_SendString(" - ");
        USB_SendString(command_table[i].description);
        USB_SendString("\r\n");

        if (command_table[i].syntax[0] != '\0') {
            USB_SendString("              Syntax: ");
            USB_SendString(command_table[i].syntax);
            USB_SendString("\r\n");
        }
    }
    USB_SendString("============================\r\n");
}
