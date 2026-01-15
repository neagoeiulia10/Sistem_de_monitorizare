/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float temperatura;
    float umiditate;
    float presiune;
} SensorData_t;

// BMP280 Calibration data
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} BMP280_Calib_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// BMP280 Definitions
#define BMP280_ADDR         (0x77 << 1)  // Using alternate address (shift for HAL)
#define BMP280_ID_REG       0xD0
#define BMP280_RESET_REG    0xE0
#define BMP280_CTRL_MEAS    0xF4
#define BMP280_CONFIG       0xF5
#define BMP280_PRESS_MSB    0xF7

// AHT20 Definitions
#define AHT20_ADDR          (0x38 << 1)  // Standard address (shift for HAL)
#define AHT20_CMD_INIT      0xBE
#define AHT20_CMD_TRIGGER   0xAC
#define AHT20_CMD_SOFTRESET 0xBA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for Task_Senzor */
osThreadId_t Task_SenzorHandle;
const osThreadAttr_t Task_Senzor_attributes = {
  .name = "Task_Senzor",
  .stack_size = 128 * 4,  // Increased for I2C operations
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_Comunicati */
osThreadId_t Task_ComunicatiHandle;
const osThreadAttr_t Task_Comunicati_attributes = {
  .name = "Task_Comunicati",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DataQueue */
osMessageQueueId_t DataQueueHandle;
const osMessageQueueAttr_t DataQueue_attributes = {
  .name = "DataQueue"
};
/* USER CODE BEGIN PV */
static BMP280_Calib_t bmp280_calib;
static int32_t t_fine;  // For temperature compensation

/* MUTEX */
osMutexId_t I2C_MutexHandle;
osMutexId_t UART_MutexHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */
// BMP280 Functions
uint8_t BMP280_Init(void);
void BMP280_ReadCalibration(void);
float BMP280_ReadTemperature(void);
float BMP280_ReadPressure(void);

// AHT20 Functions
uint8_t AHT20_Init(void);
uint8_t AHT20_Read(float *temperature, float *humidity);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ============================================================================
// BMP280 Driver Implementation
// ============================================================================

uint8_t BMP280_Init(void)
{
    uint8_t chip_id;
    uint8_t ctrl_meas = 0x27;
    uint8_t config = 0xA0;
    uint8_t reset_cmd = 0xB6;

    osMutexAcquire(I2C_MutexHandle, osWaitForever);

    if (HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, BMP280_ID_REG, 1, &chip_id, 1, 1000) != HAL_OK)
    {
        osMutexRelease(I2C_MutexHandle);
        return 0;
    }

    HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, BMP280_RESET_REG, 1, &reset_cmd, 1, 1000);
    HAL_Delay(10);
    HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, BMP280_CONFIG, 1, &config, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR, BMP280_CTRL_MEAS, 1, &ctrl_meas, 1, 1000);

    BMP280_ReadCalibration();

    osMutexRelease(I2C_MutexHandle);

    return (chip_id == 0x58);
}

void BMP280_ReadCalibration(void) {
    uint8_t calib[24];

    HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0x88, 1, calib, 24, 1000);

    bmp280_calib.dig_T1 = (calib[1] << 8) | calib[0];
    bmp280_calib.dig_T2 = (calib[3] << 8) | calib[2];
    bmp280_calib.dig_T3 = (calib[5] << 8) | calib[4];
    bmp280_calib.dig_P1 = (calib[7] << 8) | calib[6];
    bmp280_calib.dig_P2 = (calib[9] << 8) | calib[8];
    bmp280_calib.dig_P3 = (calib[11] << 8) | calib[10];
    bmp280_calib.dig_P4 = (calib[13] << 8) | calib[12];
    bmp280_calib.dig_P5 = (calib[15] << 8) | calib[14];
    bmp280_calib.dig_P6 = (calib[17] << 8) | calib[16];
    bmp280_calib.dig_P7 = (calib[19] << 8) | calib[18];
    bmp280_calib.dig_P8 = (calib[21] << 8) | calib[20];
    bmp280_calib.dig_P9 = (calib[23] << 8) | calib[22];
}

float BMP280_ReadTemperature(void) {
    uint8_t data[3];
    int32_t adc_T;
    int32_t var1, var2;

    HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0xFA, 1, data, 3, 1000);

    adc_T = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);

    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) *
            ((int32_t)bmp280_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) *
            ((int32_t)bmp280_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    return ((t_fine * 5 + 128) >> 8) / 100.0f;
}

float BMP280_ReadPressure(void) {
    uint8_t data[3];
    int32_t adc_P;
    int64_t var1, var2, p;
    osMutexAcquire(I2C_MutexHandle, osWaitForever);

    HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0xF7, 1, data, 3, 1000);
    osMutexRelease(I2C_MutexHandle);

    adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)bmp280_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_calib.dig_P1) >> 33;

    if (var1 == 0) {
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);

    return (float)p / 25600.0f;  // Return in hPa
}

// ============================================================================
// AHT20 Driver Implementation
// ============================================================================

uint8_t AHT20_Init(void) {
    uint8_t init_cmd[3] = {AHT20_CMD_INIT, 0x08, 0x00};

    HAL_Delay(40);  // Wait for sensor power-up

    // Send soft reset
    uint8_t reset = AHT20_CMD_SOFTRESET;
    osMutexAcquire(I2C_MutexHandle, osWaitForever);
    HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, &reset, 1, 1000);
    HAL_Delay(20);
    HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, init_cmd, 3, 1000);
    osMutexRelease(I2C_MutexHandle);

    HAL_Delay(10);
    return 1;
}

uint8_t AHT20_Read(float *temperature, float *humidity) {
    uint8_t trigger_cmd[3] = {AHT20_CMD_TRIGGER, 0x33, 0x00};
    uint8_t data[6];

    // Trigger measurement
    osMutexAcquire(I2C_MutexHandle, osWaitForever);
    HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, trigger_cmd, 3, 1000);
    HAL_Delay(80);
    HAL_I2C_Master_Receive(&hi2c1, AHT20_ADDR, data, 6, 1000);
    osMutexRelease(I2C_MutexHandle);

    // Check if busy
    if (data[0] & 0x80) {
        return 0;
    }

    // Calculate humidity
    uint32_t hum = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    *humidity = (hum * 100.0f) / 1048576.0f;

    // Calculate temperature
    uint32_t temp = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
    *temperature = (temp * 200.0f / 1048576.0f) - 50.0f;

    return 1;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);

  char msg[] = "Sistem pornit!\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);

  // Initialize sensors
  if (BMP280_Init()) {
      char bmp_ok[] = "BMP280 OK\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)bmp_ok, strlen(bmp_ok), 1000);
  } else {
      char bmp_err[] = "BMP280 EROARE!\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)bmp_err, strlen(bmp_err), 1000);
  }

  if (AHT20_Init()) {
      char aht_ok[] = "AHT20 OK\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)aht_ok, strlen(aht_ok), 1000);
  } else {
      char aht_err[] = "AHT20 EROARE!\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)aht_err, strlen(aht_err), 1000);
  }

  // LED ON = am ajuns aici
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  I2C_MutexHandle  = osMutexNew(NULL);
  UART_MutexHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of DataQueue */
  DataQueueHandle = osMessageQueueNew(5, sizeof(SensorData_t), &DataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_Senzor */
  Task_SenzorHandle = osThreadNew(StartDefaultTask, NULL, &Task_Senzor_attributes);

  /* creation of Task_Comunicati */
  Task_ComunicatiHandle = osThreadNew(StartTask02, NULL, &Task_Comunicati_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
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
  hi2c1.Init.Timing = 0x00000608;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the Task_Senzor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  static SensorData_t sensor_data;
  float temp_aht, hum_aht;

  osDelay(2000);

  for(;;)
  {
    // Read AHT20 (temperature + humidity)
    if (AHT20_Read(&temp_aht, &hum_aht)) {
      sensor_data.temperatura = temp_aht;
      sensor_data.umiditate = hum_aht;
    } else {
      // Error reading AHT20 - use fallback values
      sensor_data.temperatura = -999.0f;
      sensor_data.umiditate = -999.0f;
    }

    // Read BMP280 (pressure, we use AHT20 for temperature)
    sensor_data.presiune = BMP280_ReadPressure();

    // Send to queue
    osMessageQueuePut(DataQueueHandle, &sensor_data, 0, 100);

        // 4. Feedback vizual (Toggle LED)
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

        // 5. Pauză de 30 de secunde (30000 ms)
        osDelay(30000);
  }
}

void StartTask02(void *argument)
{
  static SensorData_t received_data;
  static char buf[100];


  for(;;)
  {
    if (osMessageQueueGet(DataQueueHandle, &received_data, NULL, osWaitForever) == osOK)
    {
      // Convert to integers to save RAM
      int temp = (int)(received_data.temperatura * 10);
      int umid = (int)(received_data.umiditate * 10);
      int pres = (int)(received_data.presiune * 10);

      int len = snprintf(buf, sizeof(buf),
                        "T:%d.%d C|U:%d.%d%%|P:%d.%d hPa\r\n",
                        temp/10, temp%10,
                        umid/10, umid%10,
                        pres/10, pres%10);

      osMutexAcquire(UART_MutexHandle, osWaitForever);
      HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 1000);
      osMutexRelease(UART_MutexHandle);
    }
  }
}

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

#ifdef USE_FULL_ASSERT
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
