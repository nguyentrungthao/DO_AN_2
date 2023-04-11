/* USER CODE BEGIN Header */
/**
  * DO_AN_DKTD
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
#include "SHT31.h"
#include <stdio.h>
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
#define CHECKFLAG(FLAG) (flag & FLAG ? 1 : 0)
#define SETFLAG(FLAG) (flag |= (FLAG))
#define CLEARFLAG(FLAG) (flag &= ~(FLAG))
#define FLAG_ENCODER_NEW_PULSE (1<<0)
#define FLAG_ENCODER_DIR_CW (1<<1)
#define FLAG_ENCODER_DIR_CCW (1<<2)
#define FLAG_ENCODER_DELAY_DONE (1<<3)
#define FLAG_ENCODER_HANDLE_DONE (1<<4)
#define ENCODER_DELAY_MS 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define KP 10
#define KI 0.01
#define KD 0.1
#define TG_LAY_MAU 0.1

#define SO_LAN_DO_SHT31 10
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SHT31_type SHT31;

volatile uint8_t f_ACDET = 0, f_EnCBT = 0;

volatile double giaTri = 0;
uint16_t giaTriCounter;
uint16_t flag;

const char m7d[] = {0x7D, 0x11, 0xDC, 0xD5, 0xB1, 0xE5, 0xED, 0x51, 0xFD, 0xF5};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
//dùng nút nhấn trong encoder chuyển chế độ
//HAL_StatusTypeDef LayGiaTriDatTuEncoder(double *giaTriDat);
// đo nhi�?u nhi�?u lần lấy giá trị trung bình
HAL_StatusTypeDef DoNhietDoThucTe(double *nhietDo);
// đưa dữ liệu để dịch ra
HAL_StatusTypeDef QuetLed7Doan(double giaTri);
// dịch dữ liệu ra 595
HAL_StatusTypeDef DichDuLieu74595(uint8_t giaTri);
//tính toán giá trị đi�?u khiển
HAL_StatusTypeDef TinhToanPID(PID_type*);
//đi�?u khiển Triac
//???????????????????????????????????????????????????????????????????????????????
HAL_StatusTypeDef DieuKhienTriac(uint8_t giaTriDieuKhien);
double EncoderHandle();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
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
//  PID_type triac;
//	double giaTri = 0;
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  SHT31_init(&SHT31, &hi2c2);
  HAL_TIM_Base_Start_IT(&htim4);
  SETFLAG(FLAG_ENCODER_HANDLE_DONE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
//	  LayGiaTriDatTuEncoder(&giaTri);
	  if(CHECKFLAG(FLAG_ENCODER_DELAY_DONE)){
		  HAL_GPIO_TogglePin(Led_test_GPIO_Port, Led_test_Pin);
		  HAL_Delay(1);
	  }
	  QuetLed7Doan(EncoderHandle());
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(Led_test_GPIO_Port, Led_test_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Led3_7seg_Pin|Led2_7seg_Pin|Led1_7seg_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRIAC_Pin|SER_Pin|OE_Pin|RCLK_Pin
                          |SRCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_test_Pin */
  GPIO_InitStruct.Pin = Led_test_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Led_test_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Led3_7seg_Pin Led2_7seg_Pin Led1_7seg_Pin */
  GPIO_InitStruct.Pin = Led3_7seg_Pin|Led2_7seg_Pin|Led1_7seg_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
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

  /*Configure GPIO pin : EnC_CH2_Pin */
  GPIO_InitStruct.Pin = EnC_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EnC_CH2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_CH1_Pin EnC_BT_Pin */
  GPIO_InitStruct.Pin = ENC_CH1_Pin|EnC_BT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SER_Pin OE_Pin RCLK_Pin SRCLK_Pin */
  GPIO_InitStruct.Pin = SER_Pin|OE_Pin|RCLK_Pin|SRCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
//dùng nút nhấn trong encoder chuyển chế độ
//HAL_StatusTypeDef LayGiaTriDatTuEncoder(double *giaTriDat){
//	*giaTriDat = (double)(__HAL_TIM_GET_COUNTER(&htim2) / 4) / 10;
//	return HAL_OK;
//}
// đo nhiệt độ nhi�?u lần lấy giá trị trung bình
HAL_StatusTypeDef DoNhietDoThucTe(double *nhietDo){
	static uint8_t soLanDaDo = 0;
	static double tongNhietDo = 0;
	double layGiaTriNhietDo = 0;

	if(soLanDaDo < SO_LAN_DO_SHT31){
		SHT31_WriteCMD(&SHT31, SHT31_MEASUREMENT_FAST);
		SHT31_ReadTemperature(&SHT31, &layGiaTriNhietDo);
		tongNhietDo += layGiaTriNhietDo;
	}
	else{
		*nhietDo = tongNhietDo / 10.0f;
		soLanDaDo = 0;
	}
	soLanDaDo++;

	return HAL_OK;
}
// đưa dữ liệu để dịch ra
HAL_StatusTypeDef QuetLed7Doan(double giaTri){
	// số đơn vị nằm bên phải
	DichDuLieu74595(m7d[((uint16_t)(giaTri * 10)) % 10]);
	HAL_GPIO_WritePin(Led1_7seg_GPIO_Port, Led1_7seg_Pin, 1); // bật led 1
	HAL_GPIO_WritePin(Led2_7seg_GPIO_Port, Led2_7seg_Pin, 0); // tắt led 2
	HAL_GPIO_WritePin(Led3_7seg_GPIO_Port, Led3_7seg_Pin, 0); // tắt led 3
	HAL_Delay(1);

	DichDuLieu74595(m7d[((uint16_t)giaTri) % 10]);
	HAL_GPIO_WritePin(Led1_7seg_GPIO_Port, Led1_7seg_Pin, 0); // tắt led 1
	HAL_GPIO_WritePin(Led2_7seg_GPIO_Port, Led2_7seg_Pin, 1); // bật led 2
	HAL_GPIO_WritePin(Led3_7seg_GPIO_Port, Led3_7seg_Pin, 0); // tắt led 3
	HAL_Delay(1);

	DichDuLieu74595(m7d[((uint16_t)giaTri) / 10]);
	HAL_GPIO_WritePin(Led1_7seg_GPIO_Port, Led1_7seg_Pin, 0); // tắt led 1
	HAL_GPIO_WritePin(Led2_7seg_GPIO_Port, Led2_7seg_Pin, 0); // tắt led 2
	HAL_GPIO_WritePin(Led3_7seg_GPIO_Port, Led3_7seg_Pin, 1); // bật led 3
	HAL_Delay(1);
	return HAL_OK;
}
// dịch dữ liệu ra 595
HAL_StatusTypeDef DichDuLieu74595(uint8_t giaTri){
	//xuất bit cao nhất trước
	for (unsigned char i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(SER_GPIO_Port, SER_Pin, (giaTri >> i) & 0x01);
		//nhịp 1 xung cạnh xuống đưa dữ liệu vào tầng lưu trữ
		HAL_GPIO_WritePin(SRCLK_GPIO_Port, SRCLK_Pin, 0);
		HAL_GPIO_WritePin(SRCLK_GPIO_Port, SRCLK_Pin, 1);
	}
	HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin, 0);
	HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin, 1);

	return HAL_OK;
}
//tính toán giá trị đi�?u khiển
HAL_StatusTypeDef TinhToanPID(PID_type *x){
	static double alpha, beta, gamma;

	x->e = x->giaTriDat - x->nhietDo;
	alpha = 2*TG_LAY_MAU*KP + KI*TG_LAY_MAU*TG_LAY_MAU + 2*KD;
	beta = TG_LAY_MAU*TG_LAY_MAU*KI - 4*KD -2*TG_LAY_MAU*KP;
	gamma = 2*KD;
	x->giaTriTinh_HienTai = (alpha*x->e + beta*x->e1 + gamma*x->e2 + 2*TG_LAY_MAU*x->giaTriTinh_Truoc)/(2*TG_LAY_MAU);

	x->giaTriTinh_Truoc = x->giaTriTinh_HienTai;
	x->e2 = x->e1;
	x->e1 = x->e;

	if (x->giaTriTinh_HienTai > 10)
		x->giaTriTinh_HienTai = 10;
	else if (x->giaTriTinh_HienTai <= 0)
		x->giaTriTinh_HienTai = 0;
	x->giaTriDieuKhien = 10.0 - x->giaTriTinh_HienTai; /***************/

	return HAL_OK;

	return HAL_OK;
}
//đi�?u khiển Triac
HAL_StatusTypeDef DieuKhienTriac(uint8_t giaTriDieuKhien){

	//!cần cách nào đó delay tới hàng microSecond
	//DÙNG 1 TIMER �?Ể DELAY + 1 TIMER TẠO ONE PULSE MODE


	//1 cách nào đó để tạo xung kích DELAY
	//DELAY
	HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, 0);
	return HAL_OK;
}
double EncoderHandle()
{
	if(CHECKFLAG(FLAG_ENCODER_DELAY_DONE)){
		CLEARFLAG(FLAG_ENCODER_DELAY_DONE);
		if(CHECKFLAG(FLAG_ENCODER_DIR_CW)&&
			!HAL_GPIO_ReadPin(ENC_CH1_GPIO_Port, ENC_CH1_Pin)) {
			giaTriCounter++;
		}
		else if(CHECKFLAG(FLAG_ENCODER_DIR_CCW)&&
			!HAL_GPIO_ReadPin(ENC_CH1_GPIO_Port, ENC_CH1_Pin)){
			giaTriCounter--;
		}
		SETFLAG(FLAG_ENCODER_HANDLE_DONE);
	}
	 return (double)giaTriCounter / (double)10;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4){
		static uint8_t delayCount=0;
		if(CHECKFLAG(FLAG_ENCODER_NEW_PULSE)){
			delayCount++;
		}
		if(delayCount >= ENCODER_DELAY_MS){
			delayCount = 0;
			CLEARFLAG(FLAG_ENCODER_NEW_PULSE);
			SETFLAG(FLAG_ENCODER_DELAY_DONE);
		}
	}
}

//xử lý ngắt khi đ�?c nút nhấn và đ�?c ACDET
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == ACDET_Pin){
		f_ACDET = 1;
	}
	else if(GPIO_Pin == EnC_BT_Pin){
		f_EnCBT = 1;
	}
	if(GPIO_Pin == ENC_CH1_Pin){
		if(CHECKFLAG(FLAG_ENCODER_HANDLE_DONE)) {
			CLEARFLAG(FLAG_ENCODER_HANDLE_DONE);
			SETFLAG(FLAG_ENCODER_NEW_PULSE);
			if(HAL_GPIO_ReadPin(EnC_CH2_GPIO_Port, EnC_CH2_Pin)) {
				SETFLAG(FLAG_ENCODER_DIR_CW);
				CLEARFLAG(FLAG_ENCODER_DIR_CCW);
			}
			else {
				SETFLAG(FLAG_ENCODER_DIR_CCW);
				CLEARFLAG(FLAG_ENCODER_DIR_CW);
			}
		}

	}
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
