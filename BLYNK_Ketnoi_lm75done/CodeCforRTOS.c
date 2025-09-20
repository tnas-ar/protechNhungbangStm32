/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (FreeRTOS + LM75 + SSD1306)
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

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

/* FreeRTOS handles ----------------------------------------------------------*/
osThreadId_t LM75Handle, UARTHandle, OLEDHandle;
osMessageQueueId_t uartQueueHandle, oledQueueHandle;

/* Defines ----------------------------------------------------------*/
#define LM75_DEFAULT_ADDR  (0x48 << 1)

/* USER CODE BEGIN PV */
uint8_t lm75_addr = LM75_DEFAULT_ADDR;
/* USER CODE END PV */

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* RTOS task functions */
void LM75_Task(void *argument);
void UART_Task(void *argument);
void OLED_Task(void *argument);

/* USER CODE BEGIN 0 */
void UART_Print(const char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// -------- LM75 read temperature --------
float LM75_ReadTemp(uint8_t addr) {
    uint8_t buf[2];
    int16_t raw;

    if (HAL_I2C_Mem_Read(&hi2c1, addr, 0x00,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, 1000) != HAL_OK) {
        return -1000; // error
    }

    raw = ((int16_t)buf[0] << 8) | buf[1];
    raw >>= 7; // 9-bit data

    if (raw & 0x0100) { // sign extend if negative
        raw |= 0xFE00;
    }

    return raw * 0.5f;
}
/* USER CODE END 0 */

/**
  * @brief  Application entry point.
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* Init OLED */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_UpdateScreen();

  /* Init scheduler */
  osKernelInitialize();

  /* Create queues */
  uartQueueHandle = osMessageQueueNew(8, sizeof(float), NULL);
  oledQueueHandle = osMessageQueueNew(8, sizeof(float), NULL);

  /* Create tasks */
  const osThreadAttr_t LM75_attributes = {
    .name = "LM75", .stack_size = 256 * 4, .priority = osPriorityHigh,
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

  /* Start scheduler */
  osKernelStart();

  /* Infinite loop (should never reach here) */
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
    osDelay(1000); // 1s
  }
}

void UART_Task(void *argument)
{
  char msg[64];

  for(;;)
  {
	  float temp = LM75_ReadTemp(lm75_addr);
        if (temp > -200) {
            // For ESP32 → only send numeric value
            sprintf(msg, "%.2f\n", temp);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        } else {
            UART_Print("LM75 read error!\r\n");
        }
        osDelay(1000);
    }
}


void OLED_Task(void *argument)
{
  char msg[32];

  ssd1306_Init();
   ssd1306_Fill(Black);
   ssd1306_SetCursor(0, 0);
   ssd1306_UpdateScreen();
  for(;;)
  {
	  float temp = LM75_ReadTemp(lm75_addr);
	     ssd1306_Fill(0);
		                  ssd1306_SetCursor(0, 0);
		                  ssd1306_WriteString("LM75 Temp:",Font_7x10, White );

		                  ssd1306_SetCursor(0, 20);
		                  if (temp > -200) {
		                      sprintf(msg, "%.2f C", temp);
		                  } else {
		                      sprintf(msg, "Sensor Error!");
		                  }
		                  ssd1306_WriteString(msg, Font_7x10, White);
		                  ssd1306_UpdateScreen();
		                  osDelay(1000);
  }
}

/* HAL init functions --------------------------------------------------------*/

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

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add implementation to report error */
}
#endif
