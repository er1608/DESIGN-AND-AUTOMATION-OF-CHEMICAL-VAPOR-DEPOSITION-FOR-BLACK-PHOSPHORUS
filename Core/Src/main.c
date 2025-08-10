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
#define Steps_Per_Cycle 150   // 1.8°/step (166 vi bien tro)
#define MAX_ANGLE 280.0
#define MIN_ANGLE 0.0

// Macros for SSR
#define SSR_PIN GPIO_PIN_7
#define SSR_PORT GPIOA
#define time_control 1000 // 1000ms

// Macros for Temperature
#define MAX_TEMP 650
#define MIN_TEMP 25
#define RX_BUFFER_SIZE 100

// Macros for Temperature_SPI
#define MAX6675_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define MAX6675_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
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
volatile uint8_t SETT = 0;
uint32_t pre_time = 0;
float PID_SSR = 0;
uint8_t TIM2_SET = 0;
char rxBuffer[RX_BUFFER_SIZE];
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
volatile int Previous_Position = 0;	// Number of CURRENT STEPs
volatile uint8_t Stepper_flag = 0;
volatile uint8_t UART_flag = 0;
volatile int16_t Steps2Move = 0;

// VARIABLES for temperature-----------------------------------------
uint8_t SET_UART[64];
float Current_Temperature = 30;
float Current_Temperature_Kalman = 30;
float Current_Temperature_SPI = 30;
KalmanFilter myFilter;
uint8_t Current_Temperature_UART[64];	// SEND BACK TEMPERATURE FROM MCU
float Setpoint = 30;	// NHIET DO BTH 25-30
uint8_t SPI_Temperature_UART[64];	// SEND BACK SETPOINT FROM MCU
uint8_t SETPOINT[64];
volatile uint16_t TempSET_last[4] = {33, 650, 12, 550};
volatile uint16_t TimeSet[5] = {100, 200, 300, 400};


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
uint8_t rxIndex = 0;
uint8_t rxByte;
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
	  sprintf((char *)SET_UART, "%u\n", SETT);
	  // Send SET
	  HAL_UART_Transmit(&huart1, (uint8_t *)SET_UART, strlen((char *)SET_UART), 100);

	  sprintf((char *)SETPOINT, "Set: %.2f\n", Setpoint);
	  // Send SET
	  HAL_UART_Transmit(&huart1, (uint8_t *)SETPOINT, strlen((char *)SETPOINT), 100);

	  sprintf((char *)Current_Temperature_UART, "Temperature: %.2f\n", Current_Temperature_Kalman);
	  // Send current TEMP
	  HAL_UART_Transmit(&huart1, (uint8_t *)Current_Temperature_UART, strlen((char *)Current_Temperature_UART), 100);

	  sprintf((char *)SPI_Temperature_UART, "Temperature_SPI: %.2f\n", Current_Temperature_SPI);
	  // Send current TEMP
	  HAL_UART_Transmit(&huart1, (uint8_t *)SPI_Temperature_UART, strlen((char *)SPI_Temperature_UART), 100);
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
}

//--------------------STEPPER PART---------------------------
void PID2STEPS(float PID) {
	int target_Position;

    if (PID > 1.0f) PID = 1.0f;
    if (PID < 0.0f) PID = 0.0f;

	float angle = PID * MAX_ANGLE;

	target_Position = (uint8_t)(angle * Steps_Per_Cycle / 300.0);	// TINH VI TRI XOAY
    if (target_Position > Steps_Per_Cycle) target_Position = Steps_Per_Cycle;
	if (target_Position < 0) target_Position = 0;

	Steps2Move = target_Position - Previous_Position;
	Previous_Position = target_Position;
}

void Stepper()
{
	uint8_t steps = 0;

	if (Steps2Move > 0) {
		DIR = 1;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, DIR);
	}

	if (Steps2Move < 0) {
		DIR = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, DIR);
		Steps2Move *= (-1);
	}

	while (steps < Steps2Move)
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
float MAX6675_GetTemperature(void)
{
	MAX6675_CS_LOW();
	HAL_SPI_Receive(&hspi2, max6675_rx_buf, 1, 50);
	MAX6675_CS_HIGH();
    float Temp = (max6675_rx_buf[0]|(max6675_rx_buf[1] << 8)) >> 3;
    Temp *= 0.25f;

    return Temp;
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
  HAL_UART_Receive_IT(&huart1, &rxByte, 1);
  // ----------------------Initialize PID----------------
  PID.Kp = 1.62;
  PID.Ki = 0.001;
  PID.Kd = 25;
  PID.Tp = 1;
  PID.Previous_Error = 0;
  PID.Previous_u_Ki = 0;

  // ----------------------Initialize Kalman Filter----------------
  float Q = 0.01f;
  float R = 0.1f;

  KalmanFilter_Init(&myFilter, Q, R);

  float PID_Output;

  PID2STEPS(0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  Stepper();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  Tset = TempSET1;
  time_count = 0;
  TIM2_SET = 0;
  Current_Temperature = Current_Temperature_Kalman;
  Previous_Position = 0;
  Steps2Move = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (Stepper_flag) {
		  PID_previous = 0;

		  if (Current_Temperature_Kalman < Setpoint)
		  {
			  PID_Output = PID_Controller();
			  PID2STEPS(PID_Output);
		  }
		  else PID2STEPS(0);

		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	//ENA
		  Stepper();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	//ENA
		  Stepper_flag = 0;
	  }

	  if (UART_flag) {
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t adc_value;
	float Temp_MAX6675;
	if (hadc->Instance == ADC1)
	{
		adc_value = HAL_ADC_GetValue(&hadc1);
		Current_Temperature = (adc_value * (800.0f / 4096)) - 10;
		Current_Temperature_Kalman = KalmanFilter_Update(&myFilter, Current_Temperature);

		//-------------------MAX6675----------------
		Temp_MAX6675 = MAX6675_GetTemperature();
		Current_Temperature_SPI = (Temp_MAX6675 > 0) ? Temp_MAX6675 : Current_Temperature_SPI;
		//Current_Temperature_SPI = KalmanFilter_Update(&myFilter, Current_Temperature_SPI);
		//------------------------------------------

	}
}

void process_command(char *cmd)
{
    if (strcmp(cmd, "1") == 0 && TIM2_SET == 0)
    {
        HAL_TIM_Base_Start_IT(&htim2);
        TIM2_SET = 1;
    }
    else if (strcmp(cmd, "2") == 0)
    {
        PID2STEPS(0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // ENA
        Stepper();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // ENA
        Tset = TempSET1;
        time_count = 0;
        TIM2_SET = 0;
        HAL_TIM_Base_Stop_IT(&htim2);
    }
    else if (strcmp(cmd, "3") == 0)
    {
        PID2STEPS(0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        Stepper();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        Tset = TempSET1;
        time_count = 0;
        TIM2_SET = 0;
        Current_Temperature = Current_Temperature_Kalman;
        Previous_Position = 0;
        Steps2Move = 0;
        HAL_TIM_Base_Stop_IT(&htim2);
        HAL_TIM_Base_Start_IT(&htim2);
        TIM2_SET = 1;
    }
    else if (cmd[0] == 'T')
    {
        int idx = cmd[1] - '0';
        if (idx >= 1 && idx <= 5) {
            char *ptr = strchr(cmd, ':');
            if (ptr != NULL) {
                int time_val = atoi(ptr + 1);
                TimeSet[idx - 1] = time_val;
            }
        }
    }
    else if (cmd[0] == 'S')
    {
        int idx = cmd[1] - '0';
        if (idx >= 1 && idx <= 4) {
            char *ptr = strchr(cmd, ':');
            if (ptr != NULL) {
                float set_val = atof(ptr + 1);
                TempSET_last[idx - 1] = set_val;
            }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rxByte == '\n')  // gặp dấu kết thúc chuỗi
        {
            rxBuffer[rxIndex] = '\0'; // null-terminate
            process_command(rxBuffer); // xử lý chuỗi
            rxIndex = 0; // reset lại index
        }
        else
        {
            if (rxIndex < RX_BUFFER_SIZE - 1) {
                rxBuffer[rxIndex++] = rxByte;
            } else {
                rxIndex = 0; // bảo vệ chống tràn bộ đệm
            }
        }

        HAL_UART_Receive_IT(&huart1, &rxByte, 1); // tiếp tục nhận byte tiếp theo
    }
}


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART1)
//    {
//    	HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuffer, 1);
//
//        rxBuffer[sizeof(rxBuffer) - 1] = '\0';
//        char received_data[100];
//        strncpy(received_data, (char *)rxBuffer, sizeof(rxBuffer) - 1);
//
//        if (strcmp(received_data, "1") == 0 && TIM2_SET == 0) // "1" ON
//        {
//            HAL_TIM_Base_Start_IT(&htim2);
//            TIM2_SET = 1;
//        }
//
//        else if (strcmp(received_data, "2") == 0) // "2" OFF
//        {
//        	PID2STEPS(0);
//        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	//ENA
//        	Stepper();
//        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	//ENA
//
//			Tset = TempSET1;
//			time_count = 0;
//			TIM2_SET = 0;
//			HAL_TIM_Base_Stop_IT(&htim2);
//        }
//
//        else if (strcmp(received_data, "3") == 0) // "3" reset
//        {
//        	PID2STEPS(0);
//        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	//ENA
//        	Stepper();
//        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	//ENA
//
//			Tset = TempSET1;
//			time_count = 0;
//			TIM2_SET = 0;
//			Current_Temperature = Current_Temperature_Kalman;
//			Previous_Position = 0;
//			Steps2Move = 0;
//
//			HAL_TIM_Base_Stop_IT(&htim2);
//
//			HAL_TIM_Base_Start_IT(&htim2);
//			TIM2_SET = 1;
//        }
//
//        else if (received_data[0] == 'T') {
//			int idx = received_data[1] - '0';
//			if (idx >= 1 && idx <= 5) {
//				char *ptr = strchr(received_data, ':');
//				if (ptr != NULL) {
//					int time_val = atoi(ptr + 1);
//					TimeSet[idx - 1] = time_val;
//				}
//			}
//		}
//
//		else if (received_data[0] == 'S') {
//			int idx = received_data[1] - '0';
//			if (idx == 1 || idx == 3) {
//				char *ptr = strchr(received_data, ':');
//				if (ptr != NULL) {
//					float set_val = atof(ptr + 1);
//					TempSET_last[idx - 1] = set_val;
//				}
//			}
//		}
//    }
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Tick every 2 seconds
	if(htim->Instance == TIM2 && PID_previous == 1) {
		HAL_ADC_Start_IT(&hadc1);

		time_count++;

		// TEMP------------GET TEMPERATURE-------------
		switch (Tset) {
			case TempSET1:
				SETT = 1;
				Stepper_flag = 1;
				if (time_count >= TimeSet[0]) {
					// Holding the temperature in TimeSet[0] seconds
					Stepper_flag = 1;
					time_count = 0;
					Setpoint += TempSET_last[0];

					if (Setpoint > TempSET_last[1]) Setpoint = TempSET_last[1];

					if (Current_Temperature_Kalman >= TempSET_last[1]) {
						Tset = TempSET2;
						time_count = 0;
						break;
					}

					if (Setpoint > MAX_TEMP) Setpoint = MAX_TEMP;
				}
				UART_flag = 1;
				break;
			case TempSET2:
				SETT = 2;
				Stepper_flag = 1;
				if (time_count >= TimeSet[1]) {
					Stepper_flag = 1;
					Tset = TempSET3;
					time_count = 0;
				}
				UART_flag = 1;
				break;
			case TempSET3:
				SETT = 3;
				Stepper_flag = 1;
				if (time_count >= TimeSet[2]) {
					Stepper_flag = 1;
					time_count = 0;
					Setpoint -= TempSET_last[2];

					if (Setpoint < TempSET_last[1]) Setpoint = TempSET_last[3];

					if (Current_Temperature_Kalman <= TempSET_last[3]) {
						Tset = TempSET4;
						time_count = 0;
						break;
					}

					if (Setpoint < MIN_TEMP) Setpoint = MIN_TEMP;
				}
				UART_flag = 1;
				break;
			case TempSET4:
				SETT = 4;
				Stepper_flag = 1;
				if (time_count >= TimeSet[3]) {
					Stepper_flag = 1;
					Tset = TempSET5;
					time_count = 0;
				}
				UART_flag = 1;
				break;
			case TempSET5:
				SETT = 5;
				Stepper_flag = 1;
				if (time_count >= TimeSet[4]) {
					Stepper_flag = 1;
					time_count = 0;
					Setpoint -= TempSET_last[2];

					if (Current_Temperature_Kalman <= 35) { // Normal temperature
						Tset = TempSET1;
						time_count = 0;
						TIM2_SET = 0;
						Previous_Position = 0;
						Steps2Move = 0;
						Current_Temperature = Current_Temperature_Kalman;
						HAL_TIM_Base_Stop_IT(&htim2);
						break;
					}
					if (Setpoint < MIN_TEMP) Setpoint = MIN_TEMP;
				}
				UART_flag = 1;
				break;
			default:
				///// ERROR
				break;
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
