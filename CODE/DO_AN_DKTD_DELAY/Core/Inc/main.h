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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Led3_7seg_Pin GPIO_PIN_0
#define Led3_7seg_GPIO_Port GPIOA
#define Led2_7seg_Pin GPIO_PIN_1
#define Led2_7seg_GPIO_Port GPIOA
#define Led1_7seg_Pin GPIO_PIN_2
#define Led1_7seg_GPIO_Port GPIOA
#define ACDET_Pin GPIO_PIN_0
#define ACDET_GPIO_Port GPIOB
#define ACDET_EXTI_IRQn EXTI0_IRQn
#define TRIAC_Pin GPIO_PIN_1
#define TRIAC_GPIO_Port GPIOB
#define EnC_BT_Pin GPIO_PIN_4
#define EnC_BT_GPIO_Port GPIOB
#define EnC_BT_EXTI_IRQn EXTI4_IRQn
#define SER_Pin GPIO_PIN_6
#define SER_GPIO_Port GPIOB
#define OE_Pin GPIO_PIN_7
#define OE_GPIO_Port GPIOB
#define RCLK_Pin GPIO_PIN_8
#define RCLK_GPIO_Port GPIOB
#define SRCLK_Pin GPIO_PIN_9
#define SRCLK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
