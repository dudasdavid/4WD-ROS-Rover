/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include "pid.h"
#include "i2c_handler.h"
#include "lsm303dlhc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ABS(x)         (x < 0) ? (-x) : x
#define PI     3.14159f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId motorCtrlTaskHandle;
osThreadId commTaskHandle;
osThreadId servoTaskHandle;
osThreadId sensorTaskHandle;
/* USER CODE BEGIN PV */
static volatile uint32_t timeStamp =0;
static volatile uint32_t timeOutGuard =0;
static volatile uint32_t timeOutGuardMax = 0;
static volatile uint8_t timeOutGuardEna = 1;
static volatile float referenceSpeed = 0;
static float referenceSpeedRaw = 50;
static float referenceAngle = 50;
static float referenceDistance = 50;
volatile uint8_t servoEna = 0;
char txBuf[30];
char rxBuf[64];
uint8_t receiveState = 0;

static volatile float FL_min = 12.5;
static volatile float FR_min = 11;
static volatile float RL_min = 11.5;
static volatile float RR_min = 11.5;
static volatile float ST_min = 7;
                 
static volatile float FL_max = 16.5;
static volatile float FR_max = 15;
static volatile float RL_max = 15.5;
static volatile float RR_max = 15.5;
static volatile float ST_max = 9.4;
                 
static volatile float FL_val = 50;
static volatile float FR_val = 50;
static volatile float RL_val = 50;
static volatile float RR_val = 50;
static volatile float ST_val = 50;

static uint32_t motCntr = 0; //Motor encoder
static uint32_t motCntrPrev = 0;
static uint32_t motSpeedRaw = 0;
static volatile float motSpeed = 0;
static float raw2mps = 2000.f;
static volatile int16_t controlTaskCycle_ms = 20;

static volatile int16_t pwm1 = 0;
static volatile int16_t pwm2 = 0;

static volatile float motSpeedFiltered = 0;
static volatile float speedError = 0;
static volatile float speedErrorPrev = 0;
static volatile float speedIntError = 0;
static volatile float speedDerivatedError = 0;
static int16_t forceMotor = 0;
static volatile int16_t forceMotorBeforeSaturation = 0;
static volatile uint16_t absForceMotor = 0;

static volatile float antiWindup = 0;
static volatile float ctrlP = 300;
static volatile float ctrlI = 0.2;
static volatile float ctrlD = 0;
static volatile float feedForward = 15;
static int16_t forceSaturation = 90;

static float automaticSuspension = 1;
static float fixedDistance = 0;
static float referenceFL = 50;
static float referenceFR = 50;
static float referenceRL = 50;
static float referenceRR = 50;

static PID_t controller;

static volatile uint8_t whoamireg = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartMotorControlTask(void const * argument);
void StartCommTask(void const * argument);
void StartServoTask(void const * argument);
void StartSensorTask(void const * argument);

/* USER CODE BEGIN PFP */
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  
  TIM1->CCR1 = 0*3600/100; //DC motor +
  TIM1->CCR2 = 0*3600/100; //DC motor -
  

  //__HAL_RCC_I2C1_FORCE_RESET();
  //HAL_Delay(2);
  //__HAL_RCC_I2C1_RELEASE_RESET();
    
  
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of motorCtrlTask */
  osThreadDef(motorCtrlTask, StartMotorControlTask, osPriorityNormal, 0, 128);
  motorCtrlTaskHandle = osThreadCreate(osThread(motorCtrlTask), NULL);

  /* definition and creation of commTask */
  osThreadDef(commTask, StartCommTask, osPriorityNormal, 0, 128);
  commTaskHandle = osThreadCreate(osThread(commTask), NULL);

  /* definition and creation of servoTask */
  osThreadDef(servoTask, StartServoTask, osPriorityNormal, 0, 128);
  servoTaskHandle = osThreadCreate(osThread(servoTask), NULL);

  /* definition and creation of sensorTask */
  osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hi2c1.Init.OwnAddress1 = 0x08;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3600;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 36000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 40;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 36000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(PC13_LED_GPIO_Port, PC13_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENA_SERVO_GPIO_Port, ENA_SERVO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13_LED_Pin */
  GPIO_InitStruct.Pin = PC13_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PC13_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_LRDY_Pin IMU_LIN2_Pin IMU_LIN1_Pin IMU_GRDY_Pin 
                           IMU_GINT_Pin */
  GPIO_InitStruct.Pin = IMU_LRDY_Pin|IMU_LIN2_Pin|IMU_LIN1_Pin|IMU_GRDY_Pin 
                          |IMU_GINT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENA_SERVO_Pin */
  GPIO_InitStruct.Pin = ENA_SERVO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENA_SERVO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI5_ENC_Pin */
  GPIO_InitStruct.Pin = EXTI5_ENC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI5_ENC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  switch ( GPIO_Pin ) {
  case GPIO_PIN_5:
    motCntr++;
    break; 
  default:
    break;
  }
}

/**
  * @brief Count characters in char array
  * @param ptr: pointer to char array
  * @retval Number of characters in array
  */
uint16_t SizeofCharArray(char *ptr)
{
  /* Local variables */
  uint16_t len = 0;
  
  /* Search until end char */
  while (ptr[len] != '\0') {   
    len++;
  }	
  return len;
}

float filter2 (float avg, float input, float alpha) {
  avg = (alpha * input) + (1.0 - alpha) * avg;
  return avg;
}

void saturateInteger(int16_t* i, int16_t min, int16_t max) {
  int16_t val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}

void saturateFloat(float* i, float min, float max) {
  float val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  
  /* Infinite loop */
  for(;;)
  {
  
    osDelay(500);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartMotorControlTask */
/**
* @brief Function implementing the motorCtrlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorControlTask */
void StartMotorControlTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorControlTask */
  pid_initialize(&controller); 
  controller.config.P          = ctrlP;
  controller.config.I          = ctrlI;
  controller.config.D          = 0.f;
  controller.config.LowerLimit = 0.f;
  controller.config.UpperLimit = forceSaturation;
  
  /* Infinite loop */
  for(;;)
  {
    
    referenceSpeed = (referenceSpeedRaw - 50)/100.0;
    
    motSpeedRaw = motCntr - motCntrPrev;
    motSpeed = (motSpeedRaw * 1000.f) / (raw2mps * controlTaskCycle_ms);
    
    motCntrPrev = motCntr;
    
    motSpeedFiltered = filter2(motSpeedFiltered, motSpeed, 0.35);
    
#ifdef OLD
      speedErrorPrev = speedError;
      speedError = fabs(referenceSpeed) - motSpeedFiltered;
      speedIntError = speedIntError + speedError + antiWindup * (forceMotor - forceMotorBeforeSaturation);
      speedDerivatedError = speedError - speedErrorPrev;
      forceMotor = (int)(ctrlP * speedError + ctrlI * speedIntError + ctrlD * speedDerivatedError);
      
      if (fabs(referenceSpeed) < 0.01) {
        speedIntError -= speedIntError*0.03;
      }
      else {
        forceMotor += feedForward;
      }
      
      forceMotorBeforeSaturation = forceMotor;
      
      saturateInteger(&forceMotor, 0, forceSaturation);
      absForceMotor = ABS(forceMotor);
#else
      absForceMotor = (uint16_t) lroundf(pid_update(&controller, fabs(referenceSpeed), motSpeed));
      if (fabs(referenceSpeed) > 0.01) {
        absForceMotor += feedForward;
        saturateInteger((int16_t*)&absForceMotor, 0, forceSaturation);
      }
      
#endif
    
    if ((pwm1 != 0) || (pwm2 != 0)){
      TIM1->CCR1 = pwm1*3600/100; //DC motor +
      TIM1->CCR2 = pwm2*3600/100; //DC motor -
    }
    else {
      if (referenceSpeed > 0){
          TIM1->CCR1 = (100-absForceMotor)*3600/100; //DC motor +
          TIM1->CCR2 = 100*3600/100; //DC motor -
      }
      else {
          TIM1->CCR1 = 100*3600/100; //DC motor +
          TIM1->CCR2 = (100-absForceMotor)*3600/100; //DC motor -
      }
    }
    
    osDelay(controlTaskCycle_ms);
  }
  /* USER CODE END StartMotorControlTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void const * argument)
{
  /* USER CODE BEGIN StartCommTask */
  uint16_t Len = 0;
  /* Infinite loop */
  for(;;)
  {
    if (receiveState == 1){
      if ((rxBuf[0] == 'K') && (rxBuf[1] == 'A') && (rxBuf[2] == '\r')){ // KA = Keep alive (response echo)
        __ASM("NOP");
      }
      else if ((rxBuf[0] == 'S') && (rxBuf[1] == 'T') && (rxBuf[2] == 'R') && (rxBuf[6] == '\r')){
        referenceAngle = (int16_t)((rxBuf[3]  - '0')*100 + (rxBuf[4]  - '0')*10 + (rxBuf[5]  - '0')*1);
        saturateFloat(&referenceAngle,0,100);
        sprintf(txBuf, "OK: %s\r\n", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_FS((uint8_t*)txBuf, Len);
      }
      else if ((rxBuf[0] == 'S') && (rxBuf[1] == 'P') && (rxBuf[2] == 'D') && (rxBuf[6] == '\r')){
        referenceSpeedRaw = (int16_t)((rxBuf[3]  - '0')*100 + (rxBuf[4]  - '0')*10 + (rxBuf[5]  - '0')*1);
        saturateFloat(&referenceSpeedRaw,0,100);
        sprintf(txBuf, "OK: %s\r\n", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_FS((uint8_t*)txBuf, Len);
      }
      else if ((rxBuf[0] == 'D') && (rxBuf[1] == 'I') && (rxBuf[2] == 'S') && (rxBuf[6] == '\r')){
        automaticSuspension = 0;
        fixedDistance = 1;
        referenceDistance = (int16_t)((rxBuf[3]  - '0')*100 + (rxBuf[4]  - '0')*10 + (rxBuf[5]  - '0')*1);
        saturateFloat(&referenceDistance,0,100);
        sprintf(txBuf, "OK: %s\r\n", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_FS((uint8_t*)txBuf, Len);
      }
      else if ((rxBuf[0] == 'S') && (rxBuf[1] == 'U') && (rxBuf[2] == 'A') && (rxBuf[3] == '\r')){
        automaticSuspension = 1;
        fixedDistance = 0;
        sprintf(txBuf, "OK: %s\r\n", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_FS((uint8_t*)txBuf, Len);
      }
      else if ((rxBuf[0] == 'S') && (rxBuf[1] == 'U') && (rxBuf[2] == 'M') && (rxBuf[15] == '\r')){
        automaticSuspension = 0;
        fixedDistance = 0;
        referenceFL = (int16_t)((rxBuf[3]  - '0')*100 + (rxBuf[4]  - '0')*10 + (rxBuf[5]  - '0')*1);
        referenceFR = (int16_t)((rxBuf[6]  - '0')*100 + (rxBuf[7]  - '0')*10 + (rxBuf[8]  - '0')*1);
        referenceRL = (int16_t)((rxBuf[9]  - '0')*100 + (rxBuf[10]  - '0')*10 + (rxBuf[11]  - '0')*1);
        referenceRR = (int16_t)((rxBuf[12]  - '0')*100 + (rxBuf[13]  - '0')*10 + (rxBuf[14]  - '0')*1);
        saturateFloat(&referenceFL,0,100);
        saturateFloat(&referenceFR,0,100);
        saturateFloat(&referenceRL,0,100);
        saturateFloat(&referenceRR,0,100);
        sprintf(txBuf, "OK: %s\r\n", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_FS((uint8_t*)txBuf, Len);
      }
      else if ((rxBuf[0] == 'T') && (rxBuf[1] == 'G') && (rxBuf[2] == 'E') && (rxBuf[3] == '\r')){
        timeOutGuardEna = 1;
        sprintf(txBuf, "OK: %s\r\n", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_FS((uint8_t*)txBuf, Len);
      }
      else if ((rxBuf[0] == 'T') && (rxBuf[1] == 'G') && (rxBuf[2] == 'D') && (rxBuf[3] == '\r')){
        timeOutGuardEna = 0;
        sprintf(txBuf, "OK: %s\r\n", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_FS((uint8_t*)txBuf, Len);
      }
      else if ((rxBuf[0] == 'S') && (rxBuf[1] == 'R') && (rxBuf[2] == 'E') && (rxBuf[3] == '\r')){
        servoEna = 1;
        sprintf(txBuf, "OK: %s\r\n", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_FS((uint8_t*)txBuf, Len);
      }
      else if ((rxBuf[0] == 'S') && (rxBuf[1] == 'R') && (rxBuf[2] == 'D') && (rxBuf[3] == '\r')){
        servoEna = 0;
        sprintf(txBuf, "OK: %s\r\n", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_FS((uint8_t*)txBuf, Len);
      }
      else if ((rxBuf[0] == 'P') && (rxBuf[1] == 'O') && (rxBuf[2] == 'W') && (rxBuf[6] == '\r')){
        forceSaturation = (int16_t)((rxBuf[3]  - '0')*100 + (rxBuf[4]  - '0')*10 + (rxBuf[5]  - '0')*1);
        saturateInteger(&forceSaturation,0,100);
        sprintf(txBuf, "OK: %s\r\n", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_FS((uint8_t*)txBuf, Len);
      }
      
      else{
        sprintf(txBuf, "ERR: %s\r\n", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_FS((uint8_t*)txBuf, Len);
      }
      timeStamp = HAL_GetTick();
      receiveState = 0;
    }
      
    timeOutGuard = HAL_GetTick() - timeStamp;
    if (timeOutGuard > timeOutGuardMax){
      timeOutGuardMax = timeOutGuard;
    }
    
    if ((timeOutGuard > 1000) && timeOutGuardEna == 1){
      referenceSpeedRaw = 50;
      referenceAngle = 50;
    }
    osDelay(10);
  }
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Function implementing the servoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoTask */
void StartServoTask(void const * argument)
{
  /* USER CODE BEGIN StartServoTask */
  TIM2->CCR1 = (int)((RL_val / (100 / (RL_max - RL_min)) + RL_min) * 36000) / 100; //Servo RL
  TIM2->CCR2 = (int)(((100 - RR_val) / (100 / (RR_max - RR_min)) + RR_min) * 36000) / 100; //Servo RR
  TIM2->CCR3 = (int)(((100 - FL_val) / (100 / (FL_max - FL_min)) + FL_min) * 36000) / 100; //Servo FL
  TIM2->CCR4 = (int)((FR_val / (100 / (FR_max - FR_min)) + FR_min) * 36000) / 100; //Servo FR
  
  TIM3->CCR1 = (int)(((100 - ST_val) / (100 / (ST_max - ST_min)) + ST_min) * 36000) / 100; //Steering
  osDelay(100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); //SERVO DISABLED
  /* Infinite loop */
  for(;;)
  {
    
    ST_val = referenceAngle;
    
    if (automaticSuspension == 1) {
      __asm("NOP");
    }
    else {
      if (fixedDistance == 1) {
        FL_val = FR_val = RL_val = RR_val = referenceDistance;
      }
      else {
        FL_val = referenceFL;
        FR_val = referenceFR;
        RL_val = referenceRL;
        RR_val = referenceRR;
      }
    }
    
    
    TIM2->CCR1 = (int)((RL_val / (100 / (RL_max - RL_min)) + RL_min) * 36000) / 100; //Servo RL
    TIM2->CCR2 = (int)(((100 - RR_val) / (100 / (RR_max - RR_min)) + RR_min) * 36000) / 100; //Servo RR
    TIM2->CCR3 = (int)(((100 - FL_val) / (100 / (FL_max - FL_min)) + FL_min) * 36000) / 100; //Servo FL
    TIM2->CCR4 = (int)((FR_val / (100 / (FR_max - FR_min)) + FR_min) * 36000) / 100; //Servo FR
    
    TIM3->CCR1 = (int)(((100 - ST_val) / (100 / (ST_max - ST_min)) + ST_min) * 36000) / 100; //Steering
    
    if (servoEna == 1){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //SERVO ENABLED
    }
    else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); //SERVO DISABLED
    }
    
    osDelay(10);
  }
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  uint8_t a;
  //whoamireg = I2Cx_ReadData(&hi2c1, 0x19, LSM303DLHC_WHO_AM_I_ADDR);
  uint8_t ret;
  uint8_t buf[12];
  buf[0] = 0x0F;

  //ret = HAL_I2C_Master_Receive(&hi2c1, 0x32, buf, 1, I2Cx_TIMEOUT_MAX);
  //ret = HAL_I2C_Master_Transmit(&hi2c1, 0x32, buf, 1, I2Cx_TIMEOUT_MAX);
  //if ( ret != HAL_OK ) {
  //  __asm("NOP");
  //}
  //else {

    // Read 2 bytes from the temperature register
    //ret = HAL_I2C_Master_Receive(&hi2c1, 0x32, buf, 1, I2Cx_TIMEOUT_MAX);
    //if ( ret != HAL_OK ) {
    //  __asm("NOP");
    //}
    
  //}
  
  osDelay(2000);
  ret = HAL_I2C_Master_Receive(&hi2c1, 0x32, buf, 1, I2Cx_TIMEOUT_MAX);
  a = I2Cx_ReadData(&hi2c1, 0x32, 0x0F);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Infinite loop */
  for(;;)
  {
    a = I2Cx_ReadData(&hi2c1, 0x32, 0x0F);
    osDelay(100);
    
  }
  /* USER CODE END StartSensorTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
