/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LED_ON 0
#define LED_OFF 1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint32_t tgKich = 0;
volatile uint8_t flag = 0;
uint8_t tgDelayTaoGocKich = 0; // giá trị không được vượt qá 10ms
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void TaoXungKickTriac();
//void DelayFunction(uint32_t tgDelay, void(*Function)());
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t tgDelayTaoXung = 0;
  uint8_t ledStatus = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //tg kích ms
//	  if(flag == 0) {
//		  tgKich = HAL_GetTick();
//	  }
//	  else if(HAL_GetTick() - tgKich >= tgDelayTaoGocKich){
//		  TaoXungKickTriac();
//		  flag = 0;
//	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(flag == 1){
//		  HAL_Delay(tgDelayTaoGocKich);
		  // tìm cách delay khúc này =>
		  if(tgDelayTaoGocKich > 0){
			  HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, 1);
		  }
		  HAL_Delay(1);
		  HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, 0);
		  flag = 0;
	  }
	  if(HAL_GetTick() - tgDelayTaoXung >= 1000){
		  tgDelayTaoXung = HAL_GetTick();
		  ledStatus = ~ledStatus;
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledStatus);
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ACDET_Pin */
  GPIO_InitStruct.Pin = ACDET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACDET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIAC_Pin */
  GPIO_InitStruct.Pin = TRIAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TRIAC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	HAL_Delay(0); // delay 5ms mất nữa sóng
//	// tạo xung kích cho triac
//	HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, 1);
//	HAL_Delay(1);
//	HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, 0);
	if(tgDelayTaoGocKich == 0){
		HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, 1);
	}
	flag = 1;
}
//void TaoXungKickTriac(){
//	static uint8_t step = 0;
//	static uint32_t T_TRIAC = 0;
//	switch(step){
//	case 0:
//		HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, 1);
//		T_TRIAC = HAL_GetTick();
//		step = 1;
//		break;
//	case 1:
//		if(HAL_GetTick() - T_TRIAC >= 1){
//			HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, 0);
//			step = 0;
//			flag = 0;
//		}
//	}
//}

//void DelayFunction(uint32_t tgDelay, void(*Function)()){
//	static uint8_t step = 0;
//	static uint32_t T = 0;
//	switch(step){
//	case 0:
//		T = HAL_GetTick();
//		step = 1;
//		break;
//	case 1:
//		if(HAL_GetTick() - T >= tgDelay){
//			Function();
//			step = 0;
//		}
//	}
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
