/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void USB_Print(const char* format, ...);
void USB_SendString(const char* str);
void BackupReg_Write(uint32_t index, uint32_t value);
uint32_t BackupReg_Read(uint32_t index);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOC
#define KEY_PWR_Pin GPIO_PIN_1
#define KEY_PWR_GPIO_Port GPIOA
#define ADS1299_SCK_Pin GPIO_PIN_5
#define ADS1299_SCK_GPIO_Port GPIOA
#define ADS1299_DOUT_Pin GPIO_PIN_6
#define ADS1299_DOUT_GPIO_Port GPIOA
#define ADS1299_DIN_Pin GPIO_PIN_7
#define ADS1299_DIN_GPIO_Port GPIOA
#define ADS1299_RESET_Pin GPIO_PIN_4
#define ADS1299_RESET_GPIO_Port GPIOC
#define ADS1299_START_Pin GPIO_PIN_5
#define ADS1299_START_GPIO_Port GPIOC
#define ADS1299_DRDY_Pin GPIO_PIN_0
#define ADS1299_DRDY_GPIO_Port GPIOB
#define G_SCL_Pin GPIO_PIN_10
#define G_SCL_GPIO_Port GPIOB
#define G_SDA_Pin GPIO_PIN_11
#define G_SDA_GPIO_Port GPIOB
#define ADS1299_CS_Pin GPIO_PIN_6
#define ADS1299_CS_GPIO_Port GPIOC
#define G_CS_Pin GPIO_PIN_7
#define G_CS_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define POWER_CTRL_Pin GPIO_PIN_4
#define POWER_CTRL_GPIO_Port GPIOA
#define POWER_KEY_Pin GPIO_PIN_1
#define POWER_KEY_GPIO_Port GPIOA

#define BKP_MAGIC_NUMBER    0x55AA1234
#define BKP_REG_MAGIC       0
#define BKP_REG_TASK_STATE  1
#define BKP_REG_TASK_ID     2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
