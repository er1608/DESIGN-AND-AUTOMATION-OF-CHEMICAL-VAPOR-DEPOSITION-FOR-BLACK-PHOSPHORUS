/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Macros for Stepper
#define Steps_Per_Cycle 166   // 1.8°/step (166 vi bien tro)
#define MAX_ANGLE 300.0
#define MIN_ANGLE 0.0

// Macros for SSR
#define SSR_PIN GPIO_PIN_7
#define SSR_PORT GPIOA
#define time_control 1000 // 1000ms

// Macros for Temperature
#define MAX_TEMP 650
#define MIN_TEMP 25

// Macros for Temperature_SPI
#define MAX6675_CS_PORT GPIOB
#define MAX6675_CS_PIN  GPIO_PIN_12

#define MAX6675_CS_LOW()   HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN, GPIO_PIN_RESET)
#define MAX6675_CS_HIGH()  HAL_GPIO_WritePin(MAX6675_CS_PORT, MAX6675_CS_PIN, GPIO_PIN_SET)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// VARIABLES for processing PID-------------------------------------
volatile uint8_t DIR = 0;
volatile uint8_t time_count = 0;
volatile uint8_t time_hold = 0;
volatile uint8_t previous_time = 0;
volatile uint8_t SETT = 0;
volatile uint8_t TempSET1_count = 0;
volatile uint8_t TempSET3_count = 0;
volatile uint8_t TempSET5_count = 0;
uint32_t pre_time = 0;
float PID_SSR = 0;
uint8_t TIM2_SET = 0;
volatile char rxBuffer[100];
volatile uint8_t PID_previous = 1;

typedef enum {
	TempSET1,
	TempSET2,
	TempSET3,
	TempSET4,
	TempSET5
} TempSET;

TempSET Tset = TempSET1;

// VARIABLES for Stepper---------------------------------------------
volatile uint8_t Previous_Position = 0;	// Number of CURRENT STEPs
volatile uint8_t Stepper_flag = 0;
volatile uint8_t UART_flag = 0;
volatile int8_t Steps2Move = 50;

// VARIABLES for temperature-----------------------------------------
uint8_t SET_UART[30];
float Current_Temperature = 25;
float Current_Temperature_Kalman = 25;
float Current_Temperature_SPI = 25;
KalmanFilter myFilter;
uint8_t Current_Temperature_UART[30];	// SEND BACK TEMPERATURE FROM MCU
float Setpoint = 66;	// NHIET DO BTH 25
uint8_t Setpoint_Temperature_UART[30];	// SEND BACK SETPOINT FROM MCU

// Structure for PID-------------------------------------------------
struct Controller {
	float Kp;	// HE SO KHUECH DAI
	float Ki;	// HE SO TICH PHAN
	float Kd;	// HE SO DAO HAM
	float Tp;	// CHU KY LAY MAU
	float Previous_Error;
	float Previous_u_Ki;
};

struct Controller PID;

// VARIABLES for MAX6675---------------------------------------------
extern SPI_HandleTypeDef hspi2;

uint8_t max6675_rx_buf[2];
uint8_t max6675_data_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Send_UART() {
	  // Time out 100ms
//	  sprintf((char *)SET_UART, "%u\n", SETT);
//	  // Send SET
//	  HAL_UART_Transmit(&huart1, (uint8_t *)SET_UART, strlen((char *)SET_UART), 100);
//
//	  sprintf((char *)Current_Temperature_UART, "Temperature: %.2f\n", Current_Temperature);
//	  // Send current TEMP
//	  HAL_UART_Transmit(&huart1, (uint8_t *)Current_Temperature_UART, strlen((char *)Current_Temperature_UART), 100);
//
//	  sprintf((char *)Setpoint_Temperature_UART, "Set: %.2f\n", Setpoint);
//	  // Send set point
//	  HAL_UART_Transmit(&huart1, (uint8_t *)Setpoint_Temperature_UART, strlen((char *)Setpoint_Temperature_UART), 100);

	  sprintf((char *)Current_Temperature_UART, "Temperature: %.2f\n", Current_Temperature);
	  // Send current TEMP
	  HAL_UART_Transmit(&huart1, (uint8_t *)Current_Temperature_UART, strlen((char *)Current_Temperature_UART), 100);

	  sprintf((char *)Setpoint_Temperature_UART, "Temperature Kalman: %.2f\n", Current_Temperature_Kalman);
	  // Send set point
	  HAL_UART_Transmit(&huart1, (uint8_t *)Setpoint_Temperature_UART, strlen((char *)Setpoint_Temperature_UART), 100);

	  sprintf((char *)SET_UART, "Temperature MAX6675: %.2f\n", Current_Temperature_SPI);
	  // Send SET
	  HAL_UART_Transmit(&huart1, (uint8_t *)SET_UART, strlen((char *)SET_UART), 100);
}

float PID_Controller() {
	float sum = 0;
	float u_Kp, u_Ki, u_Kd, error;

	error = Setpoint - Current_Temperature;

	u_Kp = PID.Kp * error;		// KHUECH DAI
	u_Ki = PID.Ki * PID.Tp * (error + PID.Previous_Error) / 2.0 + PID.Previous_u_Ki;	// TICH PHAN HINH THANG
	u_Kd = PID.Kd * (error - PID.Previous_Error) / PID.Tp;		// DAO HAM

	PID.Previous_Error = error;	// Update error

	sum = u_Kp + u_Ki + u_Kd;	// TINH TONG
	return sum;
}

//--------------------SSR PART---------------------------
void SSR_Control()
{
	if (PID_SSR > 1.0f) PID_SSR = 1.0f;
	if (PID_SSR < 0.0f) PID_SSR = 0.0f;

    uint32_t now = HAL_GetTick();
    if ((now - pre_time) > time_control) {
    	pre_time += time_control;
    	PID_SSR = PID_Controller();
    }

    uint32_t SSR_on_time = PID_SSR * time_control;
    if ((now - pre_time) < SSR_on_time) {
        HAL_GPIO_WritePin(SSR_PORT, SSR_PIN, GPIO_PIN_SET);  // SSR ON
    } else {
        HAL_GPIO_WritePin(SSR_PORT, SSR_PIN, GPIO_PIN_RESET);  // SSR OFF
    }

//	if (Current_Temperature_Kalman <= 45) {
//		HAL_GPIO_WritePin(SSR_PORT, SSR_PIN, GPIO_PIN_SET);  // SSR ON
//	}
//	else if (Current_Temperature_Kalman >= 50){
//		HAL_GPIO_WritePin(SSR_PORT, SSR_PIN, GPIO_PIN_RESET);  // SSR OFF
//	}
}

//--------------------STEPPER PART---------------------------
int8_t PID2STEPS(float PID) {
	uint8_t target_Position;
	int8_t Steps2Move;
	float angle = PID * MAX_ANGLE; // PID (0;1)

	// Avoid out of Stepper Range
	if (angle > MAX_ANGLE) angle = MAX_ANGLE;
	if (angle < MIN_ANGLE) angle = MIN_ANGLE;

	target_Position = (uint8_t)(angle * Steps_Per_Cycle / 300.0);	// TINH VI TRI XOAY
	Steps2Move = target_Position - Previous_Position;	// TINH SO BUOC CAN QUAY THEM
	Previous_Position = target_Position;

	return Steps2Move;
}

void Stepper(int8_t Steps_Move)
{
	uint8_t steps = 0;

	if (Steps_Move > 0) {
		DIR = 1;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, DIR);
	}

	if (Steps_Move < 0) {
		DIR = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, DIR);
		Steps_Move *= (-1);
	}

	while (steps < Steps_Move)
	{
			steps++;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
			HAL_Delay(3);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
		  	HAL_Delay(3);
	}

	PID_previous = 1;
}
//--------------------------------------------------------

void MAX6675_StartRead(void)
{
    max6675_data_ready = 0;
    MAX6675_CS_LOW();

    HAL_SPI_Receive_IT(&hspi2, max6675_rx_buf, 2);
}

float MAX6675_GetTemperature(void)
{
    if (!max6675_data_ready) return 0;

    uint16_t raw = (max6675_rx_buf[0] << 8) | max6675_rx_buf[1];

    if (raw & 0x4) return -1.0f;

    raw >>= 3;
    return raw * 0.25f;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuffer, 1);
  // ----------------------Initialize PID----------------
  PID.Kp = 0.03928;
  PID.Ki = 0.0002692;
  PID.Kd = 0.01957;
  PID.Tp = 1;
  PID.Previous_Error = 0;
  PID.Previous_u_Ki = 0;

  // ----------------------Initialize Kalman Filter----------------
  float Q = 0.01f;
  float R = 0.1f;

  KalmanFilter_Init(&myFilter, Q, R);

  float Temp_MAX6675;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (Stepper_flag) {
		  PID_previous = 0;
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	//ENA
		  Stepper(Steps2Move);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	//ENA
		  Stepper_flag = 0;
	  }

	  if (UART_flag) {
		  //-------------------MAX6675----------------
		  MAX6675_StartRead();

		  Temp_MAX6675 = MAX6675_GetTemperature();
		  Current_Temperature_SPI = (Temp_MAX6675 != 0 && Temp_MAX6675 != -1) ? Temp_MAX6675 : Current_Temperature_SPI;
		  //Current_Temperature_SPI = KalmanFilter_Update(&myFilter, Current_Temperature_SPI);
		  //------------------------------------------

		  Send_UART();
		  UART_flag = 0;
	  }

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        MAX6675_CS_HIGH();
        max6675_data_ready = 1;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t adc_value;
	if (hadc->Instance == ADC1)
	{
		adc_value = HAL_ADC_GetValue(&hadc1);
		Current_Temperature = (adc_value * (800.0f / 4096)) + 30;
		Current_Temperature_Kalman = KalmanFilter_Update(&myFilter, Current_Temperature);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
    	HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuffer, 1);

        rxBuffer[sizeof(rxBuffer) - 1] = '\0';
        char received_data[100];
        strncpy(received_data, (char *)rxBuffer, sizeof(rxBuffer) - 1);

        if (strcmp(received_data, "1") == 0 && TIM2_SET == 0) // "1" ON
        {
            HAL_TIM_Base_Start_IT(&htim2);
            TIM2_SET = 1;
        }

        else if (strcmp(received_data, "2") == 0) // "2" OFF
        {
			Tset = TempSET1;
			TempSET5_count = 0;
			previous_time = 0;
			time_count = 0;
			time_hold = 0;
			TIM2_SET = 0;
			HAL_TIM_Base_Stop_IT(&htim2);
        }

        else if (strcmp(received_data, "3") == 0) // "3" reset
        {
			Tset = TempSET1;
			TempSET5_count = 0;
			previous_time = 0;
			time_count = 0;
			time_hold = 0;
			TIM2_SET = 0;
			HAL_TIM_Base_Stop_IT(&htim2);

			HAL_TIM_Base_Start_IT(&htim2);
			TIM2_SET = 1;
        }

        else if (strcmp(received_data, "4") == 0)
        {
        	Current_Temperature = 50;
        }

        else if (strcmp(received_data, "5") == 0)
		{
			Current_Temperature = 200;
		}

        else if (strcmp(received_data, "6") == 0)
		{
			Current_Temperature = 400;
		}

		else if (strcmp(received_data, "7") == 0)
		{
			Current_Temperature = 550;
		}
    }
}

//float T_ambient = 25.0f;           // Nhiệt độ môi trưong
//float alpha = 0.1f;                // Hệ số tác động của PID
//float beta = 0.05f;                // Hệ số mất nhiệt

//void Simulate_Temperature(float PID_Output) {
//	if (PID_Output > 1) PID_Output = 1;
//	if (PID_Output < 0) PID_Output = 0;
//	Current_Temperature += alpha * PID_Output - beta * (Current_Temperature - T_ambient);
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM2) {

//		HAL_ADC_Start_IT(&hadc1);

		float PID_Output = 0;

		time_count++;

		if (Current_Temperature <= 100) PID_Output = 0.25;
		else if (Current_Temperature > 100 && Current_Temperature <= 300) PID_Output = 0.5;
		else if (Current_Temperature > 300 && Current_Temperature <= 500) PID_Output = 0.75;
		else if (Current_Temperature > 500 && Current_Temperature <= 600) PID_Output = 1;

		if (PID_previous == 1)
		{
			Steps2Move = PID2STEPS(PID_Output);
			Stepper_flag = 1;
		}

		if (time_count - previous_time == 3) {
			previous_time = time_count;
			UART_flag = 1;
		}
	}
}

/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM2) {
		float PID_Output;

		HAL_ADC_Start_IT(&hadc1);

		time_count++;

		// TEMP------------GET TEMPERATURE-------------
		switch (Tset) {
			case TempSET1:
				SETT = 1;

				if (time_count < 9 && PID_previous == 1) { //600
					// Holding the temperature in 10mins
					PID_Output = PID_Controller();
					Steps2Move = PID2STEPS(PID_Output);
//					Simulate_Temperature(PID_Output);
					Stepper_flag = 1;
				}

				if (time_count - previous_time == 3) { //10
					previous_time = time_count;
					UART_flag = 1;
				}

				if (time_count == 9) { // 10min = 600
					if (TempSET1_count == 3) { // 650/200 = 3
						Tset = TempSET2;
						TempSET1_count = 0;
						previous_time = 0;
						time_count = 0;
						time_hold = 0;
						break;
					}

					time_hold += time_count;
					if (time_hold == 54)  {
						TempSET1_count++; // 1h = 3600s
						time_hold = 0;
					}
					previous_time = 0;
					time_count = 0;

					// ----------Command to set PID---------
					// Increasing 33C after 10mins // 200C/6 (6 times 10mins) = 33
					Setpoint += 33;
					if (Setpoint > MAX_TEMP) Setpoint = MAX_TEMP;
				}
				break;

			case TempSET2:
				SETT = 2;

				if (time_count < 27  && PID_previous == 1) { //1800
					// Holding the temperature at 650C in 30mins
					PID_Output = PID_Controller();
					Steps2Move = PID2STEPS(PID_Output);
//					Simulate_Temperature(PID_Output);
					Stepper_flag = 1;
				}
				else {
					Tset = TempSET3;
					previous_time = 0;
					time_count = 0;
					Setpoint -= 12;
				}

				if (time_count - previous_time == 3) { //10
					previous_time = time_count;
					UART_flag = 1;
				}

				break;

			case TempSET3:
				SETT = 3;

				if (time_count < 54 && PID_previous == 1) { //3600
					// Holding the temperature in 1h
					PID_Output = PID_Controller();
					Steps2Move = PID2STEPS(PID_Output);
//					Simulate_Temperature(PID_Output);
					Stepper_flag = 1;
				}

				if (time_count - previous_time == 3) { //10
					previous_time = time_count;
					UART_flag = 1;
				}

				if (time_count == 54) { //3600
					if (TempSET3_count == 7) {
						Tset = TempSET4;
						TempSET3_count = 0;
						previous_time = 0;
						time_count = 0;
						break;
					}

					TempSET3_count++; // 1h = 3600
					previous_time = 0;
					time_count = 0;

					// ----------Command to set PID---------
					// Decreasing 12C after 1h
					Setpoint -= 12;
					if (Setpoint < MIN_TEMP) Setpoint = MIN_TEMP;
				}
				break;

			case TempSET4:
				SETT = 4;

				if (time_count < 54 && PID_previous == 1) { //3600
					// Holding the temperature at 550C in 1h
					PID_Output = PID_Controller();
					Steps2Move = PID2STEPS(PID_Output);
//					Simulate_Temperature(PID_Output);
					Stepper_flag = 1;
				}
				else {
					Tset = TempSET5;
					previous_time = 0;
					time_count = 0;
					Setpoint -= 17;
				}

				if (time_count - previous_time == 3) { //10
					previous_time = time_count;
					UART_flag = 1;
				}

				break;

			case TempSET5:
				SETT = 5;

				if (time_count < 9 && PID_previous == 1) { //600
					// Holding the temperature in 10mins
					PID_Output = PID_Controller();
					Steps2Move = PID2STEPS(PID_Output);
//					Simulate_Temperature(PID_Output);
					Stepper_flag = 1;
				}

				if (time_count - previous_time == 3) { //10
					previous_time = time_count;
					UART_flag = 1;
				}

				if (time_count == 9) { //600
					if (TempSET5_count == 5) {
						Tset = TempSET1;
						TempSET5_count = 0;
						previous_time = 0;
						time_count = 0;
						time_hold = 0;
						TIM2_SET = 0;
						HAL_TIM_Base_Stop_IT(&htim2);
						break;
					}

					time_hold += time_count;
					if (time_hold == 54) {
						TempSET5_count++; // 1h = 3600
						time_hold = 0;
					}
					previous_time = 0;
					time_count = 0;

					// ----------Command to set PID---------
					// Decreasing 17C after 10mins
					Setpoint -= 17;
					if (Setpoint < MIN_TEMP) Setpoint = MIN_TEMP;
				}
				break;

			default:
				///// ERROR
				break;
		}
	}
}*/
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
