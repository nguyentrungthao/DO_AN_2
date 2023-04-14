/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Led_test1_Pin GPIO_PIN_13
#define Led_test1_GPIO_Port GPIOC
#define Led_test2_Pin GPIO_PIN_14
#define Led_test2_GPIO_Port GPIOC
#define Led_test3_Pin GPIO_PIN_15
#define Led_test3_GPIO_Port GPIOC
#define ESP_TX_Pin GPIO_PIN_2
#define ESP_TX_GPIO_Port GPIOA
#define ESP_RX_Pin GPIO_PIN_3
#define ESP_RX_GPIO_Port GPIOA
#define SRCLK_Pin GPIO_PIN_4
#define SRCLK_GPIO_Port GPIOA
#define RCLK_Pin GPIO_PIN_5
#define RCLK_GPIO_Port GPIOA
#define OE_Pin GPIO_PIN_6
#define OE_GPIO_Port GPIOA
#define SER_Pin GPIO_PIN_7
#define SER_GPIO_Port GPIOA
#define ACDET_Pin GPIO_PIN_0
#define ACDET_GPIO_Port GPIOB
#define TRIAC_Pin GPIO_PIN_1
#define TRIAC_GPIO_Port GPIOB
#define LOG_TX_Pin GPIO_PIN_10
#define LOG_TX_GPIO_Port GPIOB
#define LOG_RX_Pin GPIO_PIN_11
#define LOG_RX_GPIO_Port GPIOB
#define SD_CS_Pin GPIO_PIN_14
#define SD_CS_GPIO_Port GPIOB
#define SD_DET_Pin GPIO_PIN_15
#define SD_DET_GPIO_Port GPIOB
#define EnC_BT_Pin GPIO_PIN_10
#define EnC_BT_GPIO_Port GPIOA
#define LORA_CS_Pin GPIO_PIN_15
#define LORA_CS_GPIO_Port GPIOA
#define LORA_SCK_Pin GPIO_PIN_3
#define LORA_SCK_GPIO_Port GPIOB
#define LORA_MISO_Pin GPIO_PIN_4
#define LORA_MISO_GPIO_Port GPIOB
#define LORA_MOSI_Pin GPIO_PIN_5
#define LORA_MOSI_GPIO_Port GPIOB
#define LORA_RST_Pin GPIO_PIN_6
#define LORA_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
