#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h" // 包含硬件相关定义

// --- RGB LED 状态枚举 ---
typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_SOLID_ON,
    LED_MODE_FLASH_FAST,  // 0.25s (250ms)
    LED_MODE_FLASH_MID,   // 0.5s  (500ms)
    LED_MODE_FLASH_SLOW   // 1s    (1000ms)
} LED_Mode_t;

typedef enum {
    LED_COLOR_GREEN = 0,  // PC1
    LED_COLOR_RED,        // PA0
    LED_COLOR_BLUE        // PA3
} LED_Color_t;

// --- API 函数声明 ---
void RGB_LED_Init(void);
void RGB_LED_SetMode(LED_Color_t color, LED_Mode_t mode);
void RGB_LED_SetSingle(LED_Color_t color, LED_Mode_t mode);
void RGB_LED_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
