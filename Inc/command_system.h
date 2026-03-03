/* command_system.h */
#ifndef __COMMAND_SYSTEM_H
#define __COMMAND_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* System Constants */
#define MAX_COMMANDS            20
#define MAX_CMD_NAME_LEN        16
#define MAX_CMD_DESC_LEN        64
#define CMD_HISTORY_SIZE        10

/* File System Constants */
#define FILE_BUFFER_SIZE        512
#define MAX_FILENAME_LEN        128

/* Command Response Structure */
typedef struct {
    int success;            // 1=Success, 0=Fail
    char message[128];      // Response message
} cmd_response_t;

/* Command Handler Type */
typedef cmd_response_t (*command_handler_t)(char* args);

/* Command Entry Structure */
typedef struct {
    const char* name;           // Command name e.g. "LIST"
    command_handler_t handler;  // Function pointer
    const char* syntax;         // Syntax help
    const char* description;    // Description
} command_entry_t;

/* ========== API ========== */
void command_system_init(void);
cmd_response_t process_command_ex(char* input);
void print_all_commands(void);
const command_entry_t* find_command(const char* name);

/* Register Command */
void register_command(const char* name, command_handler_t handler, 
                      const char* syntax, const char* description);

/* Helper Functions */
void trim_string(char* str);
char* split_command_args(char* input, char** args);
char* resolve_command_alias(const char* input);

/* File Transfer Interface */
extern volatile uint8_t g_file_transfer_mode;
void handle_file_transfer(uint8_t* buf, uint32_t len);

/* History (Optional) */
void add_to_history(const char* cmd);
void show_history(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMMAND_SYSTEM_H */
