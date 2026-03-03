#ifndef __COMMANDS_IMPL_H
#define __COMMANDS_IMPL_H

#include "command_system.h"
#include <stdint.h>

// Application State Enum
typedef enum {
    APP_IDLE = 0,
    APP_SINGLE_RUNNING,
    APP_QUAD_RUNNING,
    APP_IMP_TEST_RUNNING,
	  APP_STREAM_RUNNING,
    APP_NOISE_TEST_RUNNING,
    APP_CMRR_TEST_RUNNING,
    APP_CMRR_STEP1_RUNNING,
    APP_CMRR_STEP2_RUNNING,
    APP_IMP_TEST_ALL_RUNNING,
    APP_STOP_REQUESTED,
    APP_IMPEDANCE_TEST,
    APP_NOISE_TEST,
    APP_CMRR_TEST,
    APP_SQUARE_WAVE_TEST,
    APP_RLD_TEST_RUNNING,
    APP_AUTO_STREAM_RUNNING,
    APP_POWER_TEST_RUNNING,
    APP_ADC_TEST_RUNNING
} AppState;

extern volatile AppState app_state;

// Command Handlers
void register_all_commands(void);
cmd_response_t cmd_get_handler(char* args);
cmd_response_t cmd_list_handler(char* args);
cmd_response_t cmd_diag_handler(char* args);
cmd_response_t cmd_conf_handler(char* args);
cmd_response_t cmd_auto_stream_handler(char* args);

// Other helper functions
void app_run_loop(void);

#endif /* __COMMANDS_IMPL_H */
