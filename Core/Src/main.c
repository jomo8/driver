/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

//not needed here
#define USE_HAL_UART_REGISTER_CALLBACKS 1

#include "ringbuffer.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>

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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

void GenerateSineWaveData(void);
void SendI2CMessage(uint16_t DevAddress, uint8_t * pData, uint16_t Size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//chip model: stm32c011f4p

TIM_HandleTypeDef htim1;
#define SINE_WAVE_RES 360 // Resolution of sine wave table
#define SINE_WAVE_FREQ 1 // Frequency of sine wave
uint32_t sine_wave_data[SINE_WAVE_RES];

void GenerateSineWaveData(void);
void SendI2CMessage(uint16_t DevAddress, uint8_t * pData, uint16_t Size);











/* USER CODE END PFP */
#define DATA_STORAGE_START_ADDRESS 0x08002000 // Adjust as needed

void SaveToFlash(uint16_t data) {
  uint32_t Address = DATA_STORAGE_START_ADDRESS;

  // Unlock the Flash
  HAL_FLASH_Unlock();

  // Define the type for erase (Page Erase)
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;

  // Fill EraseInitStruct structure
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page = 9;
  EraseInitStruct.NbPages = 1; // Number of pages to be erased

  // Erase the specified FLASH page
  if (HAL_FLASHEx_Erase( & EraseInitStruct, & PageError) != HAL_OK) {
    // Handle error
  }

  // Program the user Flash area half-word by half-word
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, Address, (uint64_t) data) != HAL_OK) {
    // Handle error

  }

  // Lock the Flash
  HAL_FLASH_Lock();
}



































/* Create and initialize ring buffer */
ring_buffer_t ring_buffer;
char buf_arr[128];





uint8_t rxBuffer[100];

uint8_t Rx_byte;
uint8_t Rx_data[10];
uint8_t index = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//  Rx_data[index] = Rx_byte;
//  index = (index + 1)%10;
  //char *textt = "HAL_UART_RxCpltCallback!!\n";

    //writeCyclicBuffer(&Rx_data, 1);
    ring_buffer_queue(&ring_buffer, Rx_data);
    ring_buffer_dequeue(&ring_buffer, &Rx_data);
    UART_SendString(Rx_data);

    HAL_UART_Receive_IT(&huart2, &Rx_data, 1); // Now receive the message

}

void UART_SendString(char * string) {
  HAL_UART_Transmit( & huart2, (uint8_t * ) string, strlen(string), 1000);
  //HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  //HAL_UART_Receive(&huart2, rxBuffer, Size, 1000);
  //HAL_UART_Transmit(&huart2, rxBuffer, 20, 1000);
  char *textt = "HAL_UARTEx_RxEventCallback!!\n";
  UART_SendString(textt);

}



/*
// ISR for brownout
void PWR_IRQHandler(void) {
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PWRRST))
    {
        // Clear the BOR flag
        __HAL_RCC_CLEAR_RESET_FLAGS();

        SaveToFlash(1234);

        // Optionally, reset the microcontroller
        NVIC_SystemReset();
    }
}


void EnableBORInterrupt(void) {
    // Enable the PWR clock
    __HAL_RCC_PWR_CLK_ENABLE();

    // Enable the BOR interrupt
    HAL_PWR_EnableBkUpAccess();
    __HAL_PWR_BOR_ENABLE_IT();
}

// Example function to set BOR level
void SetBORLevel(void) {
    HAL_FLASH_Unlock();
    FLASH_OBProgramInitTypeDef obConfig;

    HAL_FLASHEx_OBGetConfig(&obConfig);
    //obConfig.OptionType = OPTIONBYTE_BOR;

    // Set the BOR Level to a desired value (e.g., OB_BOR_LEVEL1, OB_BOR_LEVEL2, etc.)
    obConfig.USERConfig = OB_BOR_ENABLE | OB_BOR_LEVEL_FALLING_3 | OB_BOR_LEVEL_RISING_3;


    HAL_FLASHEx_OBProgram(&obConfig);
    HAL_FLASH_Lock();

    // Perform a system reset to apply the new BOR level
    NVIC_SystemReset();
}
*/

double abs_(double a) {
  if (a < 0.0)
    return -a;
  return a;
}

/*
float sin(float x) {
  const double pi = 3.14159265358;
  const float B = 4 / pi;
  const float C = -4 / (pi * pi);

  float y = B * x + C * x * abs_(x);

  #ifdef EXTRA_PRECISION
  //  const float Q = 0.775;
  const float P = 0.225;

  y = P * (y * abs_(y) - y) + y; // Q * y + P * y * abs_(y)
  #endif
}*/

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void GenerateSineWaveData(void) {
  for (int i = 0; i < SINE_WAVE_RES; i++) {
    // Generate sine wave data from 0 to TIM_Period
    sine_wave_data[i] = (htim1.Init.Period / 2) * (1 + sin(2 * (3.14159265358979323846) * i / SINE_WAVE_RES));
  }
}

void UpdatePWMDutyCycle(uint32_t channel, uint32_t value) {
  TIM_OC_InitTypeDef sConfigOC = {
    0
  };

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = value;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel( & htim1, & sConfigOC, channel);
  HAL_TIM_PWM_Start( & htim1, channel); // Start PWM
}

//void SendI2CMessage(uint16_t DevAddress, uint8_t *pData, uint16_t Size) {
//if (HAL_I2C_Master_Transmit(&hi2c1, DevAddress, pData, Size, 1000) != HAL_OK) {
// Handle transmission error
//Error_Handler();
//}
//}

/* USER CODE END 0 */

#include <stdarg.h>

// Variadic function to send formatted strings via UART
void UART_SendFormattedString(const char* format, ...) {
    char buffer[128]; // Buffer for the formatted string
    va_list args;     // Initialize the argument list

    va_start(args, format); // Start processing the arguments
    vsnprintf(buffer, sizeof(buffer), format, args); // Format the string
    va_end(args); // Clean up the argument list

    //UART_SendString("test:");
    UART_SendString(buffer);

}





void motorPIDA_update();
void motorPIDB_update();
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
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  //GenerateSineWaveData();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t index = 0;
  uint32_t pwm_value1 = htim1.Init.Period / 2 / 2; //*(sin(index/360.0f)+1)/2;//sine_wave_data[index];
  uint32_t pwm_value2 = 0;

  HAL_UART_Receive_IT(&huart2, Rx_data, 1); // Now receive the message

  ring_buffer_init(&ring_buffer, buf_arr, sizeof(buf_arr));


  while (1) {
  motorPIDA_update();
  motorPIDB_update();

  HAL_Delay(100); // Delay for sine wave frequency timing
  char *textt = "Second!!\n";
  UART_SendString(textt);

  //    uint8_t data;
//      UART_SendString("save received");



     // HAL_Delay(100);
      //UART_SendFormattedString("Sending a string: %s\n", "Hello, UART!");
  uint8_t data;
  //data = ring_buffer_num_items(&ring_buffer);
 // UART_SendFormattedString("received: %d\n", data);

  // while(ring_buffer_num_items(&ring_buffer) > 0)
//   {
      //readCyclicBuffer(&cb, &data);
  //while(ring_buffer_dequeue(&ring_buffer, &data))
	//{
	//UART_SendFormattedString("%d", data);
	UART_SendString(data);
      //}
   //}

    /*
    Motor A
    this is the max value of the -> htim1.Init.Period
    UpdatePWMDutyCycle(TIM_CHANNEL_1, pwm_value1);
    UpdatePWMDutyCycle(TIM_CHANNEL_2, pwm_value2);

    Motor B
    UpdatePWMDutyCycle(TIM_CHANNEL_3, pwm_value1);
    UpdatePWMDutyCycle(TIM_CHANNEL_4, pwm_value2);

    */

    // Generate sine wave on all available PWM channels

/*    UpdatePWMDutyCycle(TIM_CHANNEL_1, pwm_value1);
    UpdatePWMDutyCycle(TIM_CHANNEL_2, pwm_value2);
    UpdatePWMDutyCycle(TIM_CHANNEL_3, pwm_value1);
    UpdatePWMDutyCycle(TIM_CHANNEL_4, pwm_value2);
    UART_SendString("Merhabalar Hojam\r\n"); // Send a message over UART

    HAL_Delay(300); // Delay for sine wave frequency timing

    UpdatePWMDutyCycle(TIM_CHANNEL_1, pwm_value2);
    UpdatePWMDutyCycle(TIM_CHANNEL_2, pwm_value1);
    UpdatePWMDutyCycle(TIM_CHANNEL_3, pwm_value2);
    UpdatePWMDutyCycle(TIM_CHANNEL_4, pwm_value1);

    // Increment index for sine wave data array
    index = (index + 1) % (SINE_WAVE_RES * 20);
    HAL_Delay(300); // Delay for sine wave frequency timing

    uint8_t i2cReceiveBuffer[10]; // Buffer for receiving data
    HAL_I2C_Slave_Receive_IT( & hi2c1, i2cReceiveBuffer, sizeof(i2cReceiveBuffer));
*/
    //     uint8_t data[] = {0xAA, 0xBB, 0xCC}; // Example data
    //   SendI2CMessage(0x48 << 1, data, sizeof(data)); // Replace 0x48 with your device's address

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
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

volatile char *a = "1000000000000000000000000000000010000000000000000000000000000000100000000000000000000000000000001000000000000000000000000000000010000000000000000000000000000000100000000000000000000000000000001000000000000000000000000000000010000000000000000000000000000000100000000000000000000000000000001000000000000000000000000000000010000000000000000000000000000000100000000000000000000000000000001000000000000000000000000000000010000000000000000000000000000000";

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
a[0];
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40000A0B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  //if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    //{
      //Error_Handler();
    //}
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
      Error_Handler();
    }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA13 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// ISR Pseudocode

//  EXTI0_1_IRQn                = 5,      //!< EXTI 0 and 1 Interrupts                                           */
//  EXTI2_3_IRQn                = 6,      //!< EXTI Line 2 and 3 Interrupts                                      */
//  EXTI4_15_IRQn               = 7,      //!< EXTI Line 4 to 15 Interrupts

/*
void EXTI1_IRQHandler(void)
{
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
        // Handle the interrupt
        UART_SendString("INTERRRUPTA\r\n"); // Send a message over UART

    }

    UART_SendString("INTERRRUPTB\r\n"); // Send a message over UART

}
*/


/*

maxPWMValue = htim1.Init.Period;

//do drive the motor clockwise
UpdatePWMDutyCycle(TIM_CHANNEL_1, maxPWMValue);
UpdatePWMDutyCycle(TIM_CHANNEL_2, 0);

//drives the motor counter clockwise
UpdatePWMDutyCycle(TIM_CHANNEL_1, 0);
UpdatePWMDutyCycle(TIM_CHANNEL_2, maxPWMValue);


*/

//void printNumber(uint16_t GPIO_Pin)
//{
//  char st[6];
//  st[0] = (uint8_t)((uint16_t)(GPIO_Pin%1000000)/100000 + (uint16_t)('0'));
//  st[1] = (uint8_t)((uint16_t)(GPIO_Pin%100000)/10000 + (uint16_t)('0'));
//  st[2] = (uint8_t)((uint16_t)(GPIO_Pin%10000)/1000 + (uint16_t)('0'));
//  st[3] = (uint8_t)((uint16_t)((GPIO_Pin%1000)/100) + (uint16_t)('0'));
//  st[4] = (uint8_t)((uint16_t)((GPIO_Pin%100)/10) + (uint16_t)('0'));
//  st[5] = (uint8_t)((uint16_t)((GPIO_Pin%10)/1) + (uint16_t)('0'));
//  UART_SendString(st);
//}

/*
void print_signed_number(int64_t to_print)
{
  char st[20];
  uint_fast8_t isNeg = 0;

  if (to_print == 0) {
      UART_SendString("0");
  } else if (to_print < 0) {
      isNeg = 1;
      to_print *= -1;
  }

  uint8_t i = 0, cur = 0;

  while (to_print != 0) {
      cur = to_print %10;
      to_print /= 10;
      st[i++] = cur;
  }

  if (isNeg) {
      st[i++] = '-';
  }

  UART_SendString(st);
}
*/


int64_t motorA_pos = 0;
int64_t motorB_pos = 0;

GPIO_PinState pinA0_prev = GPIO_PIN_RESET;
GPIO_PinState pinA1_prev = GPIO_PIN_RESET;
GPIO_PinState pinA0_new = GPIO_PIN_RESET;
GPIO_PinState pinA1_new = GPIO_PIN_RESET;

GPIO_PinState pinB0_prev = GPIO_PIN_RESET;
GPIO_PinState pinB1_prev = GPIO_PIN_RESET;
GPIO_PinState pinB0_new = GPIO_PIN_RESET;
GPIO_PinState pinB1_new = GPIO_PIN_RESET;

//void motor_encoderUpdate(int64_t *pos, GPIO_PinState pin0_prev, GPIO_PinState pin1_prev, GPIO_PinState pin0_new, GPIO_PinState pin1_new,uint16_t GPIO_Pin)
//{
//
//  if(  // counterclockwise
//    (pin0_prev == 0 && pin1_prev == 0 && pin0_new == 0 && pin1_new == 1) ||
//    (pin0_prev == 0 && pin1_prev == 1 && pin0_new == 1 && pin1_new == 1) ||
//    (pin0_prev == 1 && pin1_prev == 0 && pin0_new == 0 && pin1_new == 0) ||
//    (pin0_prev == 1 && pin1_prev == 1 && pin0_new == 1 && pin1_new == 0)
//    )
//  {
//   	(*pos) --;
//  }
//  else if(  // clockwise
//    (pin0_prev == 0 && pin1_prev == 0 && pin0_new == 1 && pin1_new == 0) ||
//    (pin0_prev == 0 && pin1_prev == 1 && pin0_new == 0 && pin1_new == 0) ||
//    (pin0_prev == 1 && pin1_prev == 0 && pin0_new == 1 && pin1_new == 1) ||
//    (pin0_prev == 1 && pin1_prev == 1 && pin0_new == 0 && pin1_new == 1)
//    )
//  {
//	  (*pos) ++;
//  } else {
//    UART_SendString("ERROR FOUND\n");
//    UART_SendString("pin0_prev: ");
//    printNumber(pin0_prev);
//    UART_SendString("\npin0_new: ");
//    printNumber(pin0_new);
//    UART_SendString("\npin1_prev: ");
//    printNumber(pin1_prev);
//    UART_SendString("\npin1_new: ");
//    printNumber(pin1_new);
//    UART_SendString("\npin number: ");
//        printNumber(GPIO_Pin);
//  }
//
//
//}

/*************** motor_encoderUpdate **************
 *
 * Updates motor position based on current and previous encoder states
 *
 * Inputs:
 *       uint64_t *pos: encoder output of the motor
 *
 * Return: None
 *
 * Expects
 *      pos to be a binary representation of the pin states.
 ***********************************************/
void motor_encoderUpdate2(int64_t *pos, GPIO_PinState pin0_prev, GPIO_PinState pin1_prev, GPIO_PinState pin0_new, GPIO_PinState pin1_new, uint16_t GPIO_Pin)
{
    uint32_t state = (pin0_prev << 3) | (pin1_prev << 2) | (pin0_new << 1) | pin1_new;
//    UART_SendString("\n printing the state: ");
//    printNumber(state);

    switch (state) {
        // Counterclockwise rotation cases
        case 0b0001:
        case 0b0111:
        case 0b1000:
        case 0b1101:
            (*pos)--;
            break;

        // Clockwise rotation cases
        case 0b0010:
        case 0b0100:
        case 0b1011:
        case 0b1100:
            (*pos)++;
            break;


        default:
	    UART_SendString("ERROR FOUND\n");
//          UART_SendString("pin0_prev: ");
//	    print_signed_number(pin0_prev);
//          UART_SendString("\n pin0_new: ");
//          print_signed_number(pin0_new);
//          UART_SendString("\n pin1_prev: ");
//          print_signed_number(pin1_prev);
//          UART_SendString("\n pin1_new: ");
//          print_signed_number(pin1_new);
//          UART_SendString("\n pin number: ");
//          print_signed_number(GPIO_Pin);
    }

}

#define MOTORA_ENCODERPIN_0 4096
#define MOTORA_ENCODERPIN_1 8192
#define MOTORB_ENCODERPIN_0 16384
#define MOTORB_ENCODERPIN_1 64

//int64_t motorA_pos = 0;
//int64_t motorB_pos = 0;
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    // Update previous state
    pinA0_prev = pinA0_new;
    pinA1_prev = pinA1_new;
    pinB0_prev = pinB0_new;
    pinB1_prev = pinB1_new;

    // Determine motor and update position based on the encoder pin that triggered the interrupt
    switch (GPIO_Pin) {
        case MOTORA_ENCODERPIN_0:
        case MOTORA_ENCODERPIN_1:
            pinA0_new = (GPIO_Pin == MOTORA_ENCODERPIN_0) ? 1 : pinA0_new;
            pinA1_new = (GPIO_Pin == MOTORA_ENCODERPIN_1) ? 1 : pinA1_new;
            motor_encoderUpdate2(&motorA_pos, pinA0_prev, pinA1_prev, pinA0_new, pinA1_new, GPIO_Pin);
            break;
        case MOTORB_ENCODERPIN_0:
        case MOTORB_ENCODERPIN_1:
            pinB0_new = (GPIO_Pin == MOTORB_ENCODERPIN_0) ? 1 : pinB0_new;
            pinB1_new = (GPIO_Pin == MOTORB_ENCODERPIN_1) ? 1 : pinB1_new;
            motor_encoderUpdate2(&motorB_pos, pinB0_prev, pinB1_prev, pinB0_new, pinB1_new, GPIO_Pin);
            break;
        default:
            // Handle unexpected GPIO_Pin values, if necessary
            break;
    }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    // Update previous state
    pinA0_prev = pinA0_new;
    pinA1_prev = pinA1_new;
    pinB0_prev = pinB0_new;
    pinB1_prev = pinB1_new;

    // Determine motor and update position based on the encoder pin that triggered the interrupt
    switch (GPIO_Pin) {
        case MOTORA_ENCODERPIN_0:
        case MOTORA_ENCODERPIN_1:
            pinA0_new = (GPIO_Pin == MOTORA_ENCODERPIN_0) ? 0 : pinA0_new;
            pinA1_new = (GPIO_Pin == MOTORA_ENCODERPIN_1) ? 0 : pinA1_new;
            motor_encoderUpdate2(&motorA_pos, pinA0_prev, pinA1_prev, pinA0_new, pinA1_new, GPIO_Pin);
            break;
        case MOTORB_ENCODERPIN_0:
        case MOTORB_ENCODERPIN_1:
            pinB0_new = (GPIO_Pin == MOTORB_ENCODERPIN_0) ? 0 : pinB0_new;
            pinB1_new = (GPIO_Pin == MOTORB_ENCODERPIN_1) ? 0 : pinB1_new;
            motor_encoderUpdate2(&motorB_pos, pinB0_prev, pinB1_prev, pinB0_new, pinB1_new, GPIO_Pin);
            break;
        default:
            // Handle unexpected GPIO_Pin values, if necessary
            break;
    }
}

//int64_t motorA_pos = 0;
//int64_t motorB_pos = 0;


/*************** motorPID_update **************
 *
 * Updates motor PID values. Updates motor movement with calculated voltage
 *
 * Inputs:
 *       None
 *
 * Return: None
 *
 * Expects
 *      pos to be a binary representation of the pin states.
 ***********************************************/

void motorPIDA_update()
{
  float Kp, Ki = 0, Kd = 0;
  // Setting the proportional gain
  Kp = -0.01;

  // Setting the position variables.
  int64_t tar_pos = 0;
  int64_t curr_pos; // Use later to generalize motors

  // Calculating error
  double error = motorA_pos - tar_pos;
  float integral = 0; // TODO implement later
  float derivative = 0; // TODO implement later


  // PID formula
  double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  output = output*(htim1.Init.Period);

  uint32_t motor_volts = abs_(output);

  if (motor_volts > htim1.Init.Period) {
    motor_volts = htim1.Init.Period;
  }


  if (output > 0) {
    UpdatePWMDutyCycle(TIM_CHANNEL_3, motor_volts);
    UpdatePWMDutyCycle(TIM_CHANNEL_4, 0);

  } else if (output < 0) {
    UpdatePWMDutyCycle(TIM_CHANNEL_3, 0);
    UpdatePWMDutyCycle(TIM_CHANNEL_4, motor_volts);
  }
}


void motorPIDB_update()
{
  float Kp, Ki = 0, Kd = 0;
  // Setting the proportional gain
  Kp = 0.01;

  // Setting the position variables.
  int64_t tar_pos = 0;
  int64_t curr_pos; // Use later to generalize motors

  // Calculating error
  double error = motorB_pos - tar_pos;
  float integral = 0; // TODO implement later
  float derivative = 0; // TODO implement later


  // PID formula
  double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  output = output*(htim1.Init.Period);

  uint32_t motor_volts = abs_(output);

  if (motor_volts > htim1.Init.Period) {
    motor_volts = htim1.Init.Period;
  }


  if (output > 0) {
    UpdatePWMDutyCycle(TIM_CHANNEL_1, motor_volts);
    UpdatePWMDutyCycle(TIM_CHANNEL_2, 0);

  } else if (output < 0) {
    UpdatePWMDutyCycle(TIM_CHANNEL_1, 0);
    UpdatePWMDutyCycle(TIM_CHANNEL_2, motor_volts);
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
  while (1) {}
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
