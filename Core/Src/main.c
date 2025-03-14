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
#define Steps_Per_Cycle 200   // 1.8°/step
#define MAX_ANGLE 360.0
#define MIN_ANGLE 0.0

// Macros for Temperature
#define MAX_TEMP 650
#define MIN_TEMP 25
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// VARIABLES for processing PID-------------------------------------
volatile uint8_t DIR = 0;
volatile uint16_t time_count = 0;
volatile uint16_t TempSET1_count = 0;
volatile uint16_t TempSET3_count = 0;
volatile uint16_t TempSET5_count = 0;
uint8_t TIM2_SET = 0;

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
volatile int8_t Steps2Move = 50;

// VARIABLES for temperature-----------------------------------------
float Current_Temperature = 0;
uint8_t Current_Temperature_UART[30];	// SEND BACK TEMPERATURE FROM MCU
float Setpoint = 25;	// NHIET DO BTH 25
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Send_UART() {
	  // Time out 100ms
	  sprintf((char *)Current_Temperature_UART, "%.2f\n", Current_Temperature);
	  // Send current TEMP
	  HAL_UART_Transmit(&huart1, (uint8_t *)Current_Temperature_UART, strlen((char *)Current_Temperature_UART), 100);
	  sprintf((char *)Setpoint_Temperature_UART, "%.2f\n", Setpoint);
	  // Send set point
	  HAL_UART_Transmit(&huart1, (uint8_t *)Setpoint_Temperature_UART, strlen((char *)Setpoint_Temperature_UART), 100);
}

void Get_Current_Temperature() {
	uint32_t adc_value;

	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	{
	  adc_value = HAL_ADC_GetValue(&hadc1);
	  Current_Temperature = adc_value * (660.0f / 4096);
	}
	HAL_ADC_Stop(&hadc1);
}

float PID_Controller() {
	float sum = 0;
	float u_Kp, u_Ki, u_Kd, error;

	error = Setpoint - Current_Temperature;

	u_Kp = PID.Kp * error;		// KHUECH DAI
	u_Ki = PID.Ki * PID.Tp * (error + PID.Previous_Error) / 2.0 + PID.Previous_u_Ki;	// TICH PHAN HINH THANG
	u_Kd = (error - PID.Previous_Error) / PID.Tp;		// DAO HAM

	PID.Previous_Error = error;	// Update error

	sum = u_Kp + u_Ki + u_Kd;	// TINH TONG
	return sum;
}

int8_t PID2STEPS(float PID) {
	uint8_t target_Position;
	int8_t Steps2Move;
	float angle = PID * MAX_ANGLE; // PID (0;1)

	// Avoid out of Stepper Range
	if (angle > MAX_ANGLE) angle = MAX_ANGLE;
	if (angle < MIN_ANGLE) angle = MIN_ANGLE;

	target_Position = (uint8_t)(angle * Steps_Per_Cycle / 360.0);	// TINH VI TRI XOAY
	Steps2Move = target_Position - Previous_Position;	// TINH SO BUOC CAN QUAY

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
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
			HAL_Delay(3);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
		  	HAL_Delay(3);
	}
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
  /* USER CODE BEGIN 2 */
  // Initializing PID
    PID.Kp = 0.03928;
    PID.Ki = 0.0002692;
    PID.Kd = 0.01957;
    PID.Tp = 1;
    PID.Previous_Error = 0;
    PID.Previous_u_Ki = 0;


    // NEED BUTTON TO HANDLE TURN ON/OFF STEPPER
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, DIR); //DIR
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);	//ENA

    //NEED Interrupt TO HANDLE TIMER2
    TIM2_SET = 1;
    if (TIM2_SET) {
  	  HAL_TIM_Base_Start_IT(&htim2);
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (Stepper_flag) {
	  		  Stepper(Steps2Move);
	  		  Stepper_flag = 0;
	  	  }
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 9600-1;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM2) {
		//	float PID_Output;

			time_count++;

			// TEMP------------GET TEMPERATURE-------------

			switch (Tset) {
				case TempSET1:
					if (time_count < 100) { //60000
						// Holding the temperature in 10mins
		//				Get_Current_Temperature();
		//				PID_Output = PID_Controller();
		//				Steps2Move = PID2STEPS(Output);
						Stepper_flag = 1;
					}

					if (time_count == 99) Send_UART(); //100

					if (time_count == 100) { // 10min = 60000
						if (TempSET1_count == 3) { // 650/200 = 3
						 	Tset = TempSET2;
						 	TempSET1_count = 0;
						 	time_count = 0;
						 	break;
						}

						if (time_count == 600) TempSET1_count++; // 1h = 3600000s
						time_count = 0;

						// ----------Command to set PID---------
						// Increasing 33C after 10mins // 200C/6 (6 times 10mins) = 33
						Steps2Move += 50;
						Setpoint += 33;
						if (Setpoint > MAX_TEMP) Setpoint = MAX_TEMP;
					}
					break;

				case TempSET2:
					if (time_count < 300) { //180000
						// Holding the temperature at 650C in 30mins
		//				Get_Current_Temperature();
		//				PID_Output = PID_Controller();
		//				Steps2Move = PID2STEPS(PID_Output);
						Stepper_flag = 1;
					}
					else {
						Tset = TempSET3;
						time_count = 0;
					}

					if (time_count == 99) Send_UART(); //100

					break;

				case TempSET3:
					if (time_count < 600) { //360000
						// Holding the temperature in 1h
		//				Get_Current_Temperature();
		//				PID_Output = PID_Controller();
		//				Steps2Move = PID2STEPS(PID_Output);
						Stepper_flag = 1;
					}

					if (time_count == 99) Send_UART(); //100

					if (time_count == 600) { //360000
						if (TempSET3_count == 8) {
						 	Tset = TempSET4;
						 	TempSET3_count = 0;
						 	time_count = 0;
						 	break;
						}

						TempSET3_count++; // 1h = 360000
						time_count = 0;

						// ----------Command to set PID---------
						// Decreasing 12C after 1h
						Steps2Move = -50;
						Setpoint -= 12;
						if (Setpoint < MIN_TEMP) Setpoint = MIN_TEMP;
					}
					break;

				case TempSET4:
					if (time_count < 600) { //360000
						// Holding the temperature at 550C in 1h
		//				Get_Current_Temperature();
		//				PID_Output = PID_Controller();
		//				Steps2Move = PID2STEPS(Output);
						Stepper_flag = 1;
					}
					else {
						Tset = TempSET5;
						time_count = 0;
					}

					if (time_count == 99) Send_UART(); //100

					break;

				case TempSET5:
					if (time_count < 100) { //60000
						// Holding the temperature in 10mins
		//				Get_Current_Temperature();
		//				PID_Output = PID_Controller();
		//				Steps2Move = PID2STEPS(Output);
						Stepper_flag = 1;
					}

					if (time_count == 99) Send_UART(); // 1s = 100

					if (time_count == 100) { //60000
						if (TempSET5_count == 5) {
						 	Tset = TempSET1;
						 	TempSET5_count = 0;
						 	time_count = 0;
						 	HAL_TIM_Base_Stop_IT(&htim2);
						 	break;
						}

						if (time_count == 600) TempSET5_count++; // 1h = 360000
						time_count = 0;

						// ----------Command to set PID---------
						// Decreasing 17C after 10mins
						Steps2Move = -50;
						Setpoint -= 17;
						if (Setpoint < MIN_TEMP) Setpoint = MIN_TEMP;
					}
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
