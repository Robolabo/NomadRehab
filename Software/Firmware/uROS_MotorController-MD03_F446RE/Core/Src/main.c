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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

#define MD03_TEMPERATURE_RATIO  1.42

uint8_t MD03_isInitializaed = 0U;

typedef enum {
  MD03_STATUS_OK = 0,
  MD03_STATUS_ERROR = -1
}MD03_status_t;

typedef enum {
  MD03_ADDR_COMMAND       = 0x00,
  MD03_ADDR_STATUS        = 0x01,
  MD03_ADDR_SPEED         = 0x02,
  MD03_ADDR_ACCELERATION  = 0x03,
  MD03_ADDR_TEMPERATURE   = 0x04,
  MD03_ADDR_CURRENT       = 0x05,
  /* 0x06 unused */
  MD03_ADDR_VERSION       = 0x07,
}MD03_register_t;


MD03_status_t MD03_Init_port() {
  return MD03_STATUS_OK;
}

MD03_status_t MD03_RegisterRead_port(uint8_t dev_address, uint8_t reg_address, uint8_t* data) {
  MD03_status_t status = MD03_STATUS_OK;

  if (HAL_I2C_Mem_Read(&hi2c1, dev_address, reg_address, 1U, data, 1U, 100U) != HAL_OK) {
    status = MD03_STATUS_ERROR;
  }
  return status;
}

MD03_status_t MD03_RegisterWrite_port(uint8_t dev_address, uint8_t reg_address, uint8_t data) {
  MD03_status_t status = MD03_STATUS_OK;
  if (HAL_I2C_Mem_Write(&hi2c1, dev_address, reg_address, 1U, &data, 1U, 100U) != HAL_OK) {
    status = MD03_STATUS_ERROR;
  }
  return status;
}


MD03_status_t MD03_init() {
  MD03_status_t status = MD03_STATUS_ERROR;
  if (!MD03_isInitializaed) {
    if (MD03_Init_port() == MD03_STATUS_OK) {
      MD03_isInitializaed = 1U;
      status = MD03_STATUS_OK;
    }
  }
  return status;
}

MD03_status_t MD03_get_register(uint8_t dev_address, uint8_t reg_address, uint8_t* data) {
  MD03_status_t status = MD03_STATUS_ERROR;
  if (MD03_isInitializaed) {
    if (MD03_RegisterRead_port(dev_address, reg_address, data) == MD03_STATUS_OK) {
      status = MD03_STATUS_OK;
    }
  }
  return status;
}

MD03_status_t MD03_set_register(uint8_t dev_address, uint8_t reg_address, uint8_t data) {
  MD03_status_t status = MD03_STATUS_ERROR;
  if (MD03_isInitializaed) {
    if (MD03_RegisterWrite_port(dev_address, reg_address, data) == MD03_STATUS_OK) {
      status = MD03_STATUS_OK;
    }
  }
  return status;
}


uint8_t MD03_get_version(uint8_t address) {
  uint8_t buffer;
  if (MD03_get_register(address, MD03_ADDR_VERSION, &buffer) != MD03_STATUS_OK) {
    buffer = 0xFFU;
  }
  return buffer;
}

float MD03_get_temp(uint8_t address) {
  uint8_t buffer;

  if (MD03_get_register(address, MD03_ADDR_TEMPERATURE, &buffer) != MD03_STATUS_OK) {
    buffer = 0x00U;
  }
  return (float)(buffer*MD03_TEMPERATURE_RATIO);
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADDRESS_M1  0xB0
#define ADDRESS_M2  0xB2
#define ADDRESS_M3  0xB4
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
  /* USER CODE BEGIN 2 */
  uint8_t version;
  float temp;
  uint8_t address, ret;
  MD03_isInitializaed = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    version = MD03_get_version(ADDRESS_M1);
    temp = MD03_get_temp(ADDRESS_M1);
    version = MD03_get_version(ADDRESS_M2);
    temp = MD03_get_temp(ADDRESS_M2);
    version = MD03_get_version(ADDRESS_M3);
    temp = MD03_get_temp(ADDRESS_M3);


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
