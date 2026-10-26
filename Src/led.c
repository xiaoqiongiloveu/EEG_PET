#include "led.h"

// --- RGB LED 状态管理结构体 ---
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    LED_Mode_t current_mode;
    uint32_t last_tick;
    uint32_t toggle_interval_ms;
} LED_Control_t;

// 定义三色灯的控制数组: 0=Green(PA3), 1=Red(PA2), 2=Blue(PC1)
static LED_Control_t leds[3] = {
    {GPIOA, GPIO_PIN_3, LED_MODE_OFF, 0, 0}, // Green -> PA3
    {GPIOA, GPIO_PIN_2, LED_MODE_OFF, 0, 0}, // Red   -> PA2
    {GPIOC, GPIO_PIN_1, LED_MODE_OFF, 0, 0}  // Blue  -> PC1
};

/**
  * @brief  初始化 RGB 三色灯的 GPIO
  */
void RGB_LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 使能时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // 默认输出低电平 (灭灯)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

    // 配置 PA2, PA3
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PC1 已经在 MX_GPIO_Init 中配置过了，这里为了安全可以再次显式配置
    // 或者依赖 CubeMX 的配置。这里我们确保它被正确设置。
}

/**
  * @brief  设置指定颜色灯的模式
  * @param  color 颜色枚举 (LED_COLOR_GREEN, LED_COLOR_RED, LED_COLOR_BLUE)
  * @param  mode 模式枚举 (LED_MODE_OFF, LED_MODE_SOLID_ON, LED_MODE_FLASH_FAST, etc.)
  */
void RGB_LED_SetMode(LED_Color_t color, LED_Mode_t mode) {
    if (color > 2) return; // 安全检查

    leds[color].current_mode = mode;
    
    switch (mode) {
        case LED_MODE_OFF:
            HAL_GPIO_WritePin(leds[color].port, leds[color].pin, GPIO_PIN_SET);
            break;
        case LED_MODE_SOLID_ON:
            HAL_GPIO_WritePin(leds[color].port, leds[color].pin, GPIO_PIN_RESET);
            break;
        case LED_MODE_FLASH_FAST:
            leds[color].toggle_interval_ms = 250; // 0.25s
            leds[color].last_tick = HAL_GetTick(); // 重置计时器
            break;
        case LED_MODE_FLASH_MID:
            leds[color].toggle_interval_ms = 500; // 0.5s
            leds[color].last_tick = HAL_GetTick();
            break;
        case LED_MODE_FLASH_SLOW:
            leds[color].toggle_interval_ms = 1000; // 1s
            leds[color].last_tick = HAL_GetTick();
            break;
    }
}

/**
  * @brief  在主循环中轮询处理灯的闪烁状态 (非阻塞)
  */
void RGB_LED_Process(void) {
    uint32_t current_tick = HAL_GetTick();

    for (int i = 0; i < 3; i++) {
        // 只有闪烁模式才需要在这里处理翻转逻辑
        if (leds[i].current_mode >= LED_MODE_FLASH_FAST) {
            if (current_tick - leds[i].last_tick >= leds[i].toggle_interval_ms) {
                HAL_GPIO_TogglePin(leds[i].port, leds[i].pin);
                leds[i].last_tick = current_tick;
            }
        }
    }
}

/**
  * @brief  高级排他性单色控制：强制熄灭其他颜色，仅点亮/闪烁目标颜色
  * @param  color 目标颜色 (LED_COLOR_GREEN, LED_COLOR_RED, LED_COLOR_BLUE)
  * @param  mode  目标模式 (LED_MODE_OFF, LED_MODE_SOLID_ON, etc.)
  */
void RGB_LED_SetSingle(LED_Color_t color, LED_Mode_t mode) {
    // 1. 无脑将三种颜色全部强制设为 OFF
    RGB_LED_SetMode(LED_COLOR_GREEN, LED_MODE_OFF);
    RGB_LED_SetMode(LED_COLOR_RED,   LED_MODE_OFF);
    RGB_LED_SetMode(LED_COLOR_BLUE,  LED_MODE_OFF);
    
    // 2. 如果目标不是全灭，则将目标颜色设为对应模式
    if (mode != LED_MODE_OFF && color <= 2) {
        RGB_LED_SetMode(color, mode);
    }
}
