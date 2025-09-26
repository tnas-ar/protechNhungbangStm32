/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CODE NHOM 11 (STM32F103C8T6 + FreeRTOS + OLED SSD1306 + LM75 + MQ2)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <math.h>
#include "mq2.h"

/* Private define ------------------------------------------------------------*/
#define LM75_DEFAULT_ADDR  (0x48 << 1)
#define MQ2_ADC_CHANNEL    ADC_CHANNEL_1  // MQ2 connected to PA1 -> ADC2_IN1
#define SMOKE_THRESHOLD    1500           // Alert threshold

#define QUEUE_LENGTH       16

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
ADC_HandleTypeDef hadc2;

/* FreeRTOS handles ----------------------------------------------------------*/
osThreadId_t LM75Handle, UARTHandle, OLEDHandle, MQ2Handle;
osMessageQueueId_t uartQueueHandle, oledQueueHandle;
osMutexId_t i2cMutexHandle, uartMutexHandle;

/* USER CODE BEGIN PV */
uint8_t lm75_addr = LM75_DEFAULT_ADDR;
uint8_t mq2_alert = 0;
MQ2_t mq2;
/* USER CODE END PV */

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* RTOS task functions */
void LM75_Task(void *argument);
void UART_Task(void *argument);
void OLED_Task(void *argument);
void MQ2_Task(void *argument);

/* USER CODE BEGIN 0 */
const osMutexAttr_t i2cMutex_attributes = { .name = "i2cMutex" };
const osMutexAttr_t uartMutex_attributes = { .name = "uartMutex" };

void UART_Print(const char *msg) {
    osMutexAcquire(uartMutexHandle, osWaitForever);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    osMutexRelease(uartMutexHandle);
}

float LM75_ReadTemp(uint8_t addr) {
    uint8_t buf[2];
    int16_t raw;

    osMutexAcquire(i2cMutexHandle, osWaitForever);
    if (HAL_I2C_Mem_Read(&hi2c1, addr, 0x00,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, 1000) != HAL_OK) {
        osMutexRelease(i2cMutexHandle);
        return -1000; // error
    }
    osMutexRelease(i2cMutexHandle);

    raw = ((int16_t)buf[0] << 8) | buf[1];
    raw >>= 7;
    if (raw & 0x0100) raw |= 0xFE00;
    return raw * 0.5f;
}
/* USER CODE END 0 */

/**
  * @brief  Application entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();

  /* Init MQ2 */
  MQ2_Init(&mq2, &hadc2, MQ2_ADC_CHANNEL);
  MQ2_Calibrate(&mq2, 50, 100);

  /* Init scheduler */
  osKernelInitialize();

  /* Create mutexes */
  i2cMutexHandle = osMutexNew(&i2cMutex_attributes);
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* Create queues */
  uartQueueHandle = osMessageQueueNew(QUEUE_LENGTH, sizeof(float), NULL);
  oledQueueHandle = osMessageQueueNew(QUEUE_LENGTH, sizeof(float), NULL);

  /* Create tasks */
  const osThreadAttr_t LM75_attributes = {
    .name = "LM75", .stack_size = 256 * 4, .priority = osPriorityNormal,
  };
  LM75Handle = osThreadNew(LM75_Task, NULL, &LM75_attributes);

  const osThreadAttr_t UART_attributes = {
    .name = "UART", .stack_size = 256 * 4, .priority = osPriorityNormal,
  };
  UARTHandle = osThreadNew(UART_Task, NULL, &UART_attributes);

  const osThreadAttr_t OLED_attributes = {
    .name = "OLED", .stack_size = 256 * 4, .priority = osPriorityNormal,
  };
  OLEDHandle = osThreadNew(OLED_Task, NULL, &OLED_attributes);

  const osThreadAttr_t MQ2_attributes = {
    .name = "MQ2", .stack_size = 256 * 4, .priority = osPriorityNormal,
  };
  MQ2Handle = osThreadNew(MQ2_Task, NULL, &MQ2_attributes);

  osKernelStart();

  while (1) { }
}

/* === Tasks === */
void LM75_Task(void *argument)
{
  float temp;
  for(;;)
  {
    temp = LM75_ReadTemp(lm75_addr);
    osMessageQueuePut(uartQueueHandle, &temp, 0, 0);
    osMessageQueuePut(oledQueueHandle, &temp, 0, 0);

    if (temp > 30.0) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    }

    osDelay(1000);
  }
}

void MQ2_Task(void *argument)
{
    uint32_t adcValue;
    for(;;)
    {
        osMutexAcquire(i2cMutexHandle, osWaitForever);
        MQ2_ReadSmoke(&mq2, &adcValue);
        osMutexRelease(i2cMutexHandle);

        if (adcValue > SMOKE_THRESHOLD) {
            mq2_alert = 1;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        } else {
            mq2_alert = 0;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        }

        osDelay(1000);
    }
}

void UART_Task(void *argument)
{
    char msg[64];
    float temp, lpg_ppm, co_ppm, smoke_ppm;
    uint32_t adcValue;

    for(;;)
    {
        temp = LM75_ReadTemp(lm75_addr);

        osMutexAcquire(i2cMutexHandle, osWaitForever);
        lpg_ppm   = MQ2_ReadLPG(&mq2, &adcValue);
        co_ppm    = MQ2_ReadCO(&mq2, &adcValue);
        smoke_ppm = MQ2_ReadSmoke(&mq2, &adcValue);
        osMutexRelease(i2cMutexHandle);

        // --- LM75 Temp ---
        if (temp > -200) {
            sprintf(msg, "Temp: %.2f\n", temp);
        } else {
            sprintf(msg, "Temp: Error\n");
        }
        UART_Print(msg);

        // --- MQ2 ADC ---
        sprintf(msg, "ADC: %lu\n", adcValue);
        UART_Print(msg);

        // --- MQ2 Gas Readings ---
        sprintf(msg, "LPG: %.1f ppm\n", lpg_ppm);
        UART_Print(msg);
        sprintf(msg, "CO: %.1f ppm\n", co_ppm);
        UART_Print(msg);
        sprintf(msg, "Smoke: %.1f ppm\n", smoke_ppm);
        UART_Print(msg);

        // --- Status ---
        if (adcValue > SMOKE_THRESHOLD) {
            UART_Print("Status: ALERT!\n");
        } else {
            UART_Print("Status: SAFE\n");
        }

        osDelay(1000);
    }
}

void OLED_Task(void *argument)
{
    char msg[64];
    float temp, lpg_ppm, co_ppm, smoke_ppm;
    uint32_t adcValue;

    osMutexAcquire(i2cMutexHandle, osWaitForever);
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
    osMutexRelease(i2cMutexHandle);

    for(;;)
    {
        temp = LM75_ReadTemp(lm75_addr);

        osMutexAcquire(i2cMutexHandle, osWaitForever);
        lpg_ppm    = MQ2_ReadLPG(&mq2, &adcValue);
        co_ppm     = MQ2_ReadCO(&mq2, &adcValue);
        smoke_ppm  = MQ2_ReadSmoke(&mq2, &adcValue);
        osMutexRelease(i2cMutexHandle);

        osMutexAcquire(i2cMutexHandle, osWaitForever);
        ssd1306_Fill(Black);

        // Line 1: Temperature
        ssd1306_SetCursor(0, 0);
        if (temp > -200) {
            sprintf(msg, "Temp: %.2fC", temp);
        } else {
            sprintf(msg, "Temp: Error");
        }
        ssd1306_WriteString(msg, Font_7x10, White);

        // Line 2: ADC Value
        ssd1306_SetCursor(0, 12);
        sprintf(msg, "ADC: %lu", adcValue);
        ssd1306_WriteString(msg, Font_7x10, White);

        // Line 3: Smoke
        ssd1306_SetCursor(0, 24);
        sprintf(msg, "Smoke: %.0f", smoke_ppm);
        ssd1306_WriteString(msg, Font_7x10, White);

        // Line 4: LPG + CO
        ssd1306_SetCursor(0, 36);
        sprintf(msg, "LPG: %.0f CO: %.0f", lpg_ppm, co_ppm);
        ssd1306_WriteString(msg, Font_7x10, White);

        // Line 5: Status
        ssd1306_SetCursor(0, 48);
        if (adcValue > SMOKE_THRESHOLD) {
            ssd1306_WriteString("Status: ALERT!", Font_7x10, White);
        } else {
            ssd1306_WriteString("Status: SAFE", Font_7x10, White);
        }

        ssd1306_UpdateScreen();
        osMutexRelease(i2cMutexHandle);

        osDelay(1000);
    }
}


/* System Clock Configuration */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
}

/* GPIO init function */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

/* Error Handler */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
