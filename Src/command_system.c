/* command_system.c */
#include "command_system.h"
#include "fatfs.h"
#include <stdio.h>
#include "usbd_cdc_if.h"

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
#ifdef __cplusplus
}
#endif

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
