/* commands_impl.h */
#ifndef __COMMANDS_IMPL_H
#define __COMMANDS_IMPL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "command_system.h"

typedef enum {
    APP_IDLE = 0,
    APP_SINGLE_RUNNING,
    APP_QUAD_RUNNING,
    APP_STOP_REQUESTED
} AppState;

extern volatile AppState app_state;

/* Command Handlers */
cmd_response_t cmd_list_handler(char* args);
cmd_response_t cmd_get_handler(char* args);
cmd_response_t cmd_createlarge_handler(char* args);
cmd_response_t cmd_diag_handler(char* args);
cmd_response_t cmd_help_handler(char* args);
cmd_response_t cmd_delete_handler(char* args);
cmd_response_t cmd_info_handler(char* args);

/* ADS1299 related commands */
cmd_response_t cmd_single_handler(char* args);
cmd_response_t cmd_stop_handler(char* args);
cmd_response_t cmd_quad_handler(char* args);
cmd_response_t cmd_reg_handler(char* args);

/* Register all commands */
void register_all_commands(void);

/* Helper functions */
void create_large_test_file_impl(void);
void diagnose_sd_card_impl(void);

/* App Loop */
void app_run_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMMANDS_IMPL_H */
