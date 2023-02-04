/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055_stm32.h"
#include "MS5837.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_LED 50
#define USE_BRIGHTNESS 0
#define SIZE 20
#define TIME 10
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_ch2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t LED_Data[MAX_LED][4]; //Array to store the colour of LED
uint8_t LED_Mod[MAX_LED][4]; //Array to store the brightness of the LED
uint16_t pwmData[(24*MAX_LED)+50]; //Array to store colour in PWM format that LEDs expect
volatile int datasentflag=0; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/**
 * @brief prints up characters up to a 256 buffer
 * 
 * @param fmt 
 * @param ... 
 */
void serialprintf(const char *fmt, ...);
void Set_LED (int LEDnum, int Red, int Green, int Blue);
void LED_Sequence(int mode, int Red, int Green, int Blue);
void SetSome(int Red, int Green, int Blue, int interval, int start);
void SweepAll(int Red, int Green, int Blue);
void Christmas(int mode);
void LED_RaceStart(void);
void TestAll(int Red, int Green, int Blue);
double CalculateAccel (double velocity1, double velocity2, double time);
void ReflectData(double vector);
void ShowData(double angle, int interval, int start);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void serialprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}

//Function to set LED colours
void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Red;
	LED_Data[LEDnum][2] = Green;
	LED_Data[LEDnum][3] = Blue;
}

//Function to send data to LEDs
void WS2812_Send (void)
{
	uint32_t index=0; //Indexing varibale used to maintain its value through loops
	uint32_t colour;

  for (int i = 0; i < MAX_LED; i++){
    //Extract colour for current LED as a 24-bit string. Each element of LED_Data is 8 bits
    colour = ((LED_Data[i][1]<<16)|(LED_Data[i][2]<<8)|(LED_Data[i][3]));

    //Parse the extracted colour to see how long the PWM should be for each bit
    //Start j at 23 since MSB must be sent first(see datasheet)
    for (int j = 23; j >= 0; j--){
      
      if(colour & (1<<j)){
        pwmData[index] = 70*0.56667; //In order to send a 1, the duty cycle must be 0.56667
      }
      else pwmData[index] = 70*0.3; //In order to send a 0, the duty cycle must be 0.3
      index++;
    }    
    
  }

  //Add the reset string so that the LEDs know that the end of the string has been reached
  for(int i = 0; i<50; i++){
    pwmData[index] = 0;
    index++;
  }

  //Once the LED colours have been parsed, send the data!
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)pwmData, index);
  while(!datasentflag){}; //Wait until data has been sent(flag is set through an interrupt)
  datasentflag = 0; //Reset flag after data has beens sent so we can send more data
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
  datasentflag=1;
}

//Sets all LEDs to one colour
void SetAll(int Red, int Green, int Blue){
  for(int i = 0; i < 50; i++){
    Set_LED(i, Red, Green, Blue);
  }
}

//Sets some LEDs to a colour using a start number and intervals of incrementation
void SetSome(int Red, int Green, int Blue, int interval, int start){
  for(int i = start; i < 50; i += interval){
    Set_LED(i, Red, Green, Blue);
  }
}

//Function to set LEDs in certain ranges (0 through 10, 5 through 30, etc.)
void SetSection(int Red, int Green, int Blue, int start, int final)
{
  for(int i = start; i <= final; i++){
    Set_LED(i, Red, Green, Blue);
  }
}

//fades all the LEDs between some colour and off
void SweepAll(int Red, int Green, int Blue)
{
  for(double i = 0; i < 1; i += 0.01){
    SetAll(i*Red, i*Green, i*Blue);
    WS2812_Send();
    HAL_Delay(10);
  }
  for(double i = 1; i > 0; i -= 0.01){
    SetAll(i*Red, i*Green, i*Blue);
    WS2812_Send();
    HAL_Delay(10);
  }
  HAL_Delay(100);
}

//:)
void Christmas(int mode)
{
  //mode 0: static
  //mode 1: change lights up
  //mode 2: change lights down

  if(mode == 0){
   for(int i = 0; i < 50 ; i++){
      if(i % 3 == 0){
        Set_LED(i, 255, 0, 0); //red
      }
      else if(i % 3 == 1){
        Set_LED(i, 255, 255, 255); //white
      }
      else if(i % 3 == 2){
        Set_LED(i, 88, 139, 31); //green
      }
    }
  }
  else if(mode == 1){
    for(int i = 0; i < 50 ; i++){
      switch (LED_Data[i][2]) //if green is...
      {
      case 0: //in red mode
        Set_LED(i, 255, 255, 255); //make it white
        break;

      case 255: //in white mode
         Set_LED(i, 88, 139, 31); //to green
        break;
      
      case 139: //in green mode
        Set_LED(i, 255, 0, 0); //make it red
        break;
      
      default:
        break;
      }

      }
    }
    else if(mode == 2){
    for(int i = 0; i < 50 ; i++){
      switch (LED_Data[i][2])
      {
      case 0:
        Set_LED(i, 88, 139, 31);
        break;

      case 255:
         Set_LED(i, 255, 0, 0);
        break;
      
      case 139:
        Set_LED(i, 255, 255, 255);
        break;
      
      default:
        break;
      }

      }
    }
    
}


//sets all leds to certain colours for periods of time before setting them off
void LED_RaceStart(void){
  SetAll(255, 0, 0);
  WS2812_Send();
  HAL_Delay(2000);

  SetAll(150, 100, 0);
  WS2812_Send();
  HAL_Delay(2000);

  SetAll(0, 255, 0);
  WS2812_Send();
  HAL_Delay(2000);
  
  SetAll(0,0,0);
  WS2812_Send();
}

//if mode is 1, then the leds turn some colour, one by one
void LED_Sequence(int mode, int Red, int Green, int Blue){
  for(int i = 0; i < 50; i++){
    if(mode == 1){
       Set_LED(i, Red, Green, Blue);
    }
    else Set_LED(i, 0, 0, 0); 
    
    WS2812_Send();
    HAL_Delay(50);
  }
}

//test function
void TestAll(int Red, int Green, int Blue){
  Set_LED(0, Red, Green, Blue);

  for(int i = 1; i < 50; i++){
    SetAll(0,0,0);
    Set_LED(i, Red, Green, Blue);
    WS2812_Send();
    while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)){}
    while(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)){}
  }
}
void ReflectData(double vector) {
 if (vector > 0){
   SetAll(255, 0, 0);
 } else if (vector < 0) {
   SetAll(0, 255, 0);
 }
}
 
double CalculateAccel (double velocity1, double velocity2, double time) {
  double acceleration = (velocity1 - velocity2)/time;

  return acceleration;
}

void ShowData(double angle, int interval, int start) {
  int red, green, blue;
  if ((int) angle == 0) {
    red = 100;
    green = 100;
    blue = 100;
  } else if (angle < 0) {
    red = 0;
    green = 0;
    blue = 255;
    angle = -angle;
  } else if (angle > 0) {
    red = 255;
    green = 255;
    blue = 0;
  }

  if (angle >= interval) {
    Set_LED((start + interval), 255,0,0);
    angle = interval-1;
  }
  SetSection(red, green, blue, start, (start+angle));
  //SetSome(red, green, blue, angle, start);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  bno055_assignI2C(&hi2c1);
  bno055_setup();
  bno055_setOperationModeNDOF();

  HAL_Delay(1000); //a short delay is important to let the SD card settle

  bno055_vector_t p = {0,0,0,0};
  bno055_vector_t v = {0,0,0,0};
  double velAvg[SIZE] = {0};
  double velocitySum = 0;
  double velocity;
  double accelAvg[SIZE/10] = {0};
  double accelSum = 0;
  double acceleration;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(30);

    p = bno055_getVectorEuler();

    for (int i = 0; i < SIZE; i++) {
      velocitySum -= velAvg[i];

      p = bno055_getVectorEuler();
      double p1 = p.z;
      HAL_Delay(TIME);
      p = bno055_getVectorEuler();
      double p2 = p.z;
      velAvg[i] = (p2-p1)/TIME;
      velocitySum += velAvg[i];
      //loops around to the beginning
      if (i == SIZE) {
        i = 0;
      }

      velocity = velocitySum/SIZE;
      acceleration = accelSum/SIZE;

      //calculate future position

      //to display the data onto the LEDs
      SetAll(0,0,0);
      ShowData(200* velocity, 10, 0);
      ShowData(p.z, 10, 30);
      WS2812_Send();
      HAL_Delay(TIME);
    }

    serialprintf("Yaw: %d Roll: %d Pitch: %d\r\n", (int) p.x, (int) p.z, (int) p.y);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 70;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 71;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin SD_CS_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
