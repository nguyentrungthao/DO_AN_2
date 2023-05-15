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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SHT31.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  double giaTriTinh_HienTai, giaTriTinh_Truoc, giaTriDieuKhien;
  double e, e1, e2, nhietDo, giaTriDat;
} PID_type;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define CHECKFLAG(FLAG, variable) (variable & FLAG ? 1 : 0)
#define SETFLAG(FLAG, variable) (variable |= (FLAG))
#define CLEARFLAG(FLAG, variable) (variable &= ~(FLAG))

#define FLAG_ACDET_TRIGGER 	(1 << 0)
#define FLAG_TIM_OVERFLOW 	(1 << 1)
#define FLAG_COMPLETE_RECEIVE_UART (1 << 2)
#define FLAG_BLINK_7SEGMENT (1 << 3)
#define FLAG_SETUP_SETPOINT (1 << 4)


#define SEGMENT_A	(1 << 0)
#define SEGMENT_B	(1 << 2)
#define SEGMENT_C	(1 << 4)
#define SEGMENT_D	(1 << 6)
#define SEGMENT_E	(1 << 7)
#define SEGMENT_F	(1 << 1)
#define SEGMENT_G	(1 << 3)
#define SEGMENT_DOT	(1 << 5)


#define SO_LAN_DO_SHT31 50

#define SETPOINT 50
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

//array data export to 7 segment

double KP = 0.65, KI = 0.0001, KD = 0, TG_LAY_MAU = 1;

PID_type triac = {0, 0, 0, 0, 0, 0, 0, 0};

SHT31_type SHT31;

uint8_t s7seg[7] = {0};
//7 segment code
const char code7seg[] = {0xD7, 0x14, 0xCD, 0x5D, 0x1E, 0x5B, 0xDB, 0x15, 0xDF, 0x5F};

uint8_t flag = 0;

uint8_t str[50] = {0};
uint8_t rx_str_uart3[10] = {0};
uint8_t Get_rx_str[1] = {0};
volatile uint16_t encoderValue = 0;
//uint8_t i;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

HAL_StatusTypeDef PID_Caculator(PID_type* x);
void sendData74595(uint16_t data);
void Scan7seg();

void xuLyChuoiNhanVeTuLaptop();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t t = 0;
	uint32_t tBlink = 0;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart3, Get_rx_str, 1);
  HAL_UART_Receive_IT(&huart2, Get_rx_str, 1);
//  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Stop_IT(&htim4);
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);

  triac.giaTriDat = SETPOINT;
  __HAL_TIM_SET_COUNTER(&htim1, SETPOINT * 10);
  s7seg[3] = s7seg[4] = s7seg[5] = s7seg[6] = SEGMENT_G;

  while(SHT31_init(&SHT31, &hi2c1) != HAL_OK){
	if(HAL_GetTick() - t >= 500){
		t = HAL_GetTick();
		if(s7seg[0] == 0x00){
			s7seg[0] = SEGMENT_A | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G;
			s7seg[1] = s7seg[2] = SEGMENT_E | SEGMENT_G;
		}
		else{
			s7seg[0] = s7seg[1] = s7seg[2] = 0x00;
		}
	}
	Scan7seg();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GetTick() - t >= (uint32_t)(1000 * TG_LAY_MAU)){
		  t = HAL_GetTick();

		  SHT31_WriteCMD(&SHT31, SHT31_MEASUREMENT_FAST);
		  while(SHT31_ReadTemperature(&SHT31, &triac.nhietDo) != HAL_OK);

		  PID_Caculator(&triac);

		  sprintf(str, "%-2.2lf°C, %-2.2lf°C, %-2.2lf°C, %-2.2lf\n", triac.nhietDo, triac.giaTriDat, triac.e, triac.giaTriDieuKhien);
		  HAL_UART_Transmit(&huart3, str, sizeof(str), 10000);
		  s7seg[0] = code7seg[(uint16_t)triac.nhietDo / 10];
		  s7seg[1] = code7seg[(uint16_t)triac.nhietDo % 10] | SEGMENT_DOT;
		  s7seg[2] = code7seg[(uint16_t)(triac.nhietDo * 10) % 10];
	  }

	  if(CHECKFLAG(FLAG_COMPLETE_RECEIVE_UART, flag)){
		  xuLyChuoiNhanVeTuLaptop();
		  memset(str, 0, sizeof(str));
		  CLEARFLAG(FLAG_COMPLETE_RECEIVE_UART, flag);
	  }

	  if(!CHECKFLAG(FLAG_SETUP_SETPOINT, flag)){
		  encoderValue = __HAL_TIM_GetCounter(&htim1);
		  triac.giaTriDat = (double)encoderValue / 10;
			if(HAL_GetTick() - tBlink >= 300){
				tBlink = HAL_GetTick();
				if(s7seg[3] == 0x00){
					s7seg[3] = code7seg[encoderValue / 100];
					s7seg[4] = code7seg[encoderValue / 10 % 10] | SEGMENT_DOT;
					s7seg[5] = code7seg[encoderValue % 10];
				}
				else{
					s7seg[3] = s7seg[4] = s7seg[5] = 0x00;
				}
			}

	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Scan7seg();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SER_Pin|OE_Pin|RCLK_Pin|SRCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SER_Pin OE_Pin RCLK_Pin SRCLK_Pin */
  GPIO_InitStruct.Pin = SER_Pin|OE_Pin|RCLK_Pin|SRCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EnC_BT_Pin */
  GPIO_InitStruct.Pin = EnC_BT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EnC_BT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

HAL_StatusTypeDef PID_Caculator(PID_type* x) {
  static double alpha, beta, gamma;

  x->e = x->giaTriDat - x->nhietDo;
  alpha = 2 * TG_LAY_MAU * KP + KI * TG_LAY_MAU * TG_LAY_MAU + 2 * KD;
  beta = TG_LAY_MAU * TG_LAY_MAU * KI - 4 * KD - 2 * TG_LAY_MAU * KP;
  gamma = 2 * KD;
  x->giaTriTinh_HienTai = (alpha * x->e + beta * x->e1 +
		gamma * x->e2 + 2 * TG_LAY_MAU * x->giaTriTinh_Truoc) / (2 * TG_LAY_MAU);

  x->giaTriTinh_Truoc = x->giaTriTinh_HienTai;
  x->e2 = x->e1;
  x->e1 = x->e;

  if (x->giaTriTinh_HienTai > 9)
    x->giaTriTinh_HienTai = 9;
  else if (x->giaTriTinh_HienTai <= 1)
    x->giaTriTinh_HienTai = 1;

  x->giaTriDieuKhien = x->giaTriTinh_HienTai; /***************/

  return HAL_OK;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	  if (htim->Instance == TIM4) {
	    if (CHECKFLAG(FLAG_ACDET_TRIGGER, flag)) {
	      CLEARFLAG(FLAG_ACDET_TRIGGER, flag);
	      HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, 1);
	      for(uint8_t i = 0; i < 60; i++);
	      HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, 0);
	    }
	  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == ACDET_Pin){
		SETFLAG(FLAG_ACDET_TRIGGER, flag);
		__HAL_TIM_SET_COUNTER(&htim4, (uint16_t)(triac.giaTriDieuKhien * 1000));
//		__HAL_TIM_SET_COUNTER(&htim4, 9000);
	}
	if(GPIO_Pin == EnC_BT_Pin){
		if(CHECKFLAG(FLAG_SETUP_SETPOINT, flag)){
			HAL_TIM_Base_Stop_IT(&htim4);
			CLEARFLAG(FLAG_SETUP_SETPOINT, flag);
			s7seg[3] = s7seg[4] = s7seg[5] = s7seg[6] = SEGMENT_G;
		}
		else{
			SETFLAG(FLAG_SETUP_SETPOINT, flag);
			s7seg[3] = code7seg[encoderValue / 100];
			s7seg[4] = code7seg[encoderValue / 10 % 10] | SEGMENT_DOT;
			s7seg[5] = code7seg[encoderValue % 10];
			s7seg[6] = SEGMENT_A | SEGMENT_B | SEGMENT_F | SEGMENT_G;
			HAL_TIM_Base_Start_IT(&htim4);
		}
	}
}

/*
 * brief: send data 16 bit to two 74595
 * */
void sendData74595(uint16_t data){
	for (uint8_t i = 0; i < 16; i++) {
		HAL_GPIO_WritePin(SER_GPIO_Port, SER_Pin, (data >> i) & 0x01);
		//nhịp 1 xung cạnh xuống đưa dữ liệu vào tầng lưu trữ
		HAL_GPIO_WritePin(SRCLK_GPIO_Port, SRCLK_Pin, 0);
		HAL_GPIO_WritePin(SRCLK_GPIO_Port, SRCLK_Pin, 1);
	}
	HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin, 0);
	HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin, 1);
}
/*
 * brief: export data in S7seg variable
 * warning SHOUDLE BE CALL WHILE(TRUE)
 * */
void Scan7seg(){
	static uint32_t T_delay = 0;
	static uint8_t index = 0;
	if(index < 7){
		if(HAL_GetTick() - T_delay >= 1) {
			T_delay = HAL_GetTick();
			sendData74595((0x0100 << index) | s7seg[index]);
			index++;
		}
	}
	else{
		index = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	static uint8_t i = 0;
	if(huart->Instance == huart3.Instance){
		HAL_UART_Receive_IT(&huart3, Get_rx_str, 1);
		if(Get_rx_str[0] == 'x'){
			SETFLAG(FLAG_COMPLETE_RECEIVE_UART, flag);
			i = 0;
		}
		else{
			rx_str_uart3[i++] = Get_rx_str[0];
		}
	}
	if(huart->Instance == huart2.Instance){
		HAL_UART_Receive_IT(&huart2, Get_rx_str, 1);
		switch(Get_rx_str[0]){
		case 'a':
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
			break;
		case 'b':
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			break;
		case 'c':
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
			break;
		case 'd':
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		}
	}
}

void xuLyChuoiNhanVeTuLaptop(){
	double val = atof(rx_str_uart3 + 2);

	if(rx_str_uart3[0] == 'k'){
		if(rx_str_uart3[1] == 'p'){
			KP = val;
		}
		else if(rx_str_uart3[1] == 'i'){
			KI = val;
		}
		else if(rx_str_uart3[1] == 'd'){
			KD = val;
		}
	}
	else if(rx_str_uart3[0] == 's' && rx_str_uart3[1] == 'p'){
		triac.giaTriDat = val;
		//	triac.giaTriDat = atof(rx_str);
		s7seg[3] = code7seg[(uint8_t)(triac.giaTriDat / 10)];
		s7seg[4] = code7seg[(uint8_t)triac.giaTriDat % 10] | SEGMENT_DOT;
		s7seg[5] = code7seg[(uint16_t)(triac.giaTriDat * 10) % 10];
		s7seg[6] = SEGMENT_G | SEGMENT_E | SEGMENT_D;

	}
	else if(rx_str_uart3[0] == 't' && rx_str_uart3[1] == 'g'){
		TG_LAY_MAU = val;
	}
	sprintf(str, "\n\n%lf, %lf, %lf, %lf\n\n", KP, KI, KD, TG_LAY_MAU);
	HAL_UART_Transmit(&huart3, str, sizeof(str), 10000);
}
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
