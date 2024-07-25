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
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "dht11.h"
#include "sht20.h"
#include "core_json.h"
#include "stm32l4xx_hal.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int report_tempRH_json(void);

//static int parser_led_json(char *json_string, int bytes);

static void proc_uart1_recv(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  //uint32_t		lux, noisy;

  float		temperature, humidity;

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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //printf("temperature: %.3f\n", 30.33);

  while (1)
  {

	  proc_uart1_recv();

	  report_tempRH_json();
	  HAL_Delay(3000);

	  //adc_sample_lux_noisy(&lux, &noisy);
	  //printf("Lux[%lu] Noisy[%lu]\r\n", lux, noisy);
	  //HAL_Delay(5000);

	  if(SHT20_SampleData(&temperature, &humidity) <0 )
	  {
		  printf("error: SHT20 Sample Data failure\r\n");
	  }
	  else
	  {
		  printf("SHT20 Sample Temperature: %.3f  Relative Humidity: %.3f\r\n", temperature, humidity);

	  }

	  HAL_Delay(3000);

	  if( report_tempRH_json() < 0 )
	  {
		  printf("error: UART report temperature and relative humidity failure\r\n");

	  }
	  HAL_Delay(3000);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int report_tempRH_json(void)
  {
  	char		buf[128];
  	float		temperature, humidity;

  	if( SHT20_SampleData(&temperature ,&humidity) < 0 )
  	{
  		printf("ERROR: SHT20 sample data failure\n");

  		return -1;
  	}

  	memset(buf, 0, sizeof(buf));
  	snprintf(buf, sizeof(buf), "{\"Temperature\":\"%.2f\",\"Humidity\":\"%.2f\"}",temperature, humidity);
  	HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 0xFFFF);

  	return 0;

  }

int	parser_led_json(char *json_string, int bytes)
{
		JSONStatus_t		result;
		char				save;
		char				*value;
		size_t				valen;
		int					i;

		printf("DEBUG: start parser JSON string: %s\r\n", json_string);

		result = JSON_Validate(json_string, bytes);

		if(JSONPartial == result)
		{
			printf("WARN: JSON document is valid so far but incomlete!\r\n");
			return 0;
		}

		if(JSONSuccess != result)
		{
			printf("ERROR: JSON document is not valid JSON!\r\n");
			return -1;
		}

		for(i=0; i<LedMax; i++)
		{
			result = JSON_Search( json_string, bytes, leds[i].name, strlen(leds[i].name), &value, &valen);
			if( JSONSuccess == result )
			{
				save = value[valen];
				value[valen] = '\0';

				if( !strncasecmp(value, "on", 2) )
				{
					printf("DEBUG: turn %s on\r\n", leds[i].name);
					turn_led(i, ON);
				}

				else if( !strncasecmp(value, "off", 3) )
				{
					printf("DEBUG: turn %s off\r\n", leds[i].name);
					turn_led(i, OFF);
				}

				value[valen] = save;
			}
		}
	return 1;

}

void proc_uart1_recv(void)
{
	if( g_uart1_bytes >0 )
	{
		HAL_Delay(200);
		if( 0 != parser_led_json(g_uart1_rxbuf, g_uart1_bytes) )
		{
			clear_uart1_rxbuf();

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
