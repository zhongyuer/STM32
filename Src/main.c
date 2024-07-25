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
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
//#include "event_groups.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define pdMS_TO_TICKS( xTimeInMs ) ( ( TickType_t )( ( ( TickType_t )( xTimeInMs ) * ( TickType_t ) configTICK_RATE_HZ ) / ( TickType_t ) 1000 ) )
#define KEY1_EVENT (0x01 << 0)
#define KEY2_EVENT (0x01 << 1)
#define KEY3_EVENT (0X01 << 2)

#define KEY_ON 1
#define KEY_OFF 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static TaskHandle_t LED_Task_Handle = NULL;
static TaskHandle_t KEY_Task_Handle = NULL;
static EventGroupHandle_t Event_Handle = NULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
static void AppTaskCreate(void* parameter);
static void LED_Task(void* parameters);
//static void Hello_Task(void *parameters);
static void KEY_Task(void* parameters);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 static void AppTaskCreate(void* parameter)
 {
	  BaseType_t xReturn = pdPASS;

	  taskENTER_CRITICAL();

	  Event_Handle = xEventGroupCreate();
	  if(NULL != Event_Handle)
		  printf("Event_Handle Create Successfully\r\n");

	  //printf("Free heap size before creating LED_Task: %u\n", (unsigned int)xPortGetFreeHeapSize());
	  xReturn = xTaskCreate(LED_Task, "LED_Task", 128, NULL, 3, &LED_Task_Handle);
	  if(pdPASS != xReturn)
	  {
		  printf("LED_Task Create Error\r\n");
	  }

	  //printf("Free heap size before creating KEY_Task: %u\n", (unsigned int)xPortGetFreeHeapSize());
	  xReturn = xTaskCreate(KEY_Task, "KEY_Task", 128, NULL, 2, &KEY_Task_Handle);
	  if(pdPASS != xReturn)
	  {
		  //printf("x: %ld\r\n", xReturn);
		  printf("KET_Task Create Error\r\n");
	  }

	  vTaskDelete(NULL);

	  taskEXIT_CRITICAL();
 }

  static void LED_Task(void* parameter)
  {
	  EventBits_t r_event;

	  while(1)
	  {
		  r_event = xEventGroupWaitBits(Event_Handle, KEY1_EVENT|KEY2_EVENT|KEY3_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);
		  if((r_event & KEY1_EVENT) == KEY1_EVENT)
		  {
			  blink_led(RedLed, ON);
			  printf("RED LED ON\r\n");
		  }
		  else
		  {
			  blink_led(RedLed, OFF);
		  }

		  if((r_event & KEY2_EVENT) == KEY2_EVENT)
		  {
			  blink_led(GreenLed, ON);
			  printf("GREEN LED ON\r\n");
		  }
		  else
		  {
			  blink_led(GreenLed, OFF);
		  }

		  if((r_event & KEY3_EVENT) == KEY3_EVENT)
		  {
			  blink_led(BlueLed, ON);
			  printf("BLUE LED ON\r\n");
		  }
		  else
		  {
			  blink_led(BlueLed, OFF);
		  }

	  }
  }

  static void KEY_Task(void* parameter)
  {
	  while(1)
	  {
		  if(Key_Scan(Key1_GPIO_Port, Key1_Pin) == KEY_ON)
		  {
			  xEventGroupSetBits(Event_Handle, KEY1_EVENT);
			  printf("KEY1 ON\r\n");
		  }

		  if(Key_Scan(Key2_GPIO_Port, Key2_Pin) == KEY_ON)
		  {
			  xEventGroupSetBits(Event_Handle, KEY2_EVENT);
			  printf("KEY2 ON\r\n");
		  }

		  if(Key_Scan(Key3_GPIO_Port, Key3_Pin) == KEY_ON)
		  {
			  xEventGroupSetBits(Event_Handle, KEY3_EVENT);
			  printf("KEY3 ON\r\n");
		  }
	  }
  }

#if 0
void LED_Task(void* parameters)
{
	while(1)
	{
		blink_led(RedLed);
		blink_led(GreenLed);
		blink_led(BlueLed);
	}
}

void Hello_Task(void *parameters)
{
	while(1)
	{
		printf("Hello World\r\n");
		vTaskDelay(pdMS_TO_TICKS(3000));
	}

}
#endif
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
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();

  //printf("Free heap size before creating App_Task: %u\n", (unsigned int)xPortGetFreeHeapSize());
  BaseType_t xReturn = pdPASS;
  xReturn = xTaskCreate(AppTaskCreate, "AppTaskCreate", 128, NULL, 1, NULL);
  if(xReturn ==pdPASS)
  {
	  vTaskStartScheduler();//启动调度器
  }

  //xTaskCreate(Hello_Task, "Hello_Task", 128, NULL,1, NULL);


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
uint8_t Key_Scan(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET)
    {
        // 按键按下，等待按键释放
        while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET);
        return KEY_ON;
    }
    else
    {
        return KEY_OFF;
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
