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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include "sht20.h"
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
//#include "sht20.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	IDLE,
	SAMPLING,
	REPORTING,
	STOPPED
}SHT20_Task_State_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LED_ON_EVENT 		(0x01 << 0)
#define LED_OFF_EVENT 		(0x01 << 1)
#define SHT20_ON_EVENT 		(0X01 << 2)
#define SHT20_OFF_EVENT 	(0x01 << 3)

#define CMD_COUNT	4
const uint64_t	CMD_LIST[CMD_COUNT] = {
		0x1122334455667788,		//CMD_LED_ON
		0x1022334455667788,		//CMD_LED_OFF
		0x1020334455667788,		//CMD_SHT20_ON
		0x1020304455667788		//CMD_SHT2-_OFF
};


#if 0
#define CMD_LED_ON			0x1122334455667788
#define CMD_LED_OFF 		0x1022334455667788
#define CMD_SHT20_ON 		0x1020334455667788
#define CMD_SHT20_OFF  		0x1020304455667788
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static EventGroupHandle_t Event_Handle = NULL;
static TaskHandle_t CMD_Parser_Task_Handle = NULL;
static TaskHandle_t LED_Task_Handle = NULL;
static TaskHandle_t SHT20_Report_Task_Handle = NULL;
StreamBufferHandle_t CAN_Buffer_Handle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void AppTaskCreate(void* parameter);
static void CMD_Parser_Task(void* parameter);
static void LED_Task(void* parameter);
static void SHT20_Report_Task(void* parameter);

void FloatToByte(uint8_t *buf, size_t buf_size);
extern int CAN_TX_Message(uint8_t *TxData, int length);
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
  MX_USART1_UART_Init();
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();

  BaseType_t xReturn = pdPASS;
  xReturn = xTaskCreate(AppTaskCreate, "AppTaskCreate", 128, NULL, 1, NULL);
  if(xReturn ==pdPASS)
  {
	  printf("333\r\n");
  	  vTaskStartScheduler();//启动调度器
  }

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void AppTaskCreate(void* parameter)
{

	BaseType_t xReturn = pdPASS;

	taskENTER_CRITICAL();

	Event_Handle = xEventGroupCreate();
	if(NULL == Event_Handle)
	{
		printf("Event_Handle Create Failure\r\n");
	}

	CAN_Buffer_Handle = xStreamBufferCreate(128, sizeof(uint8_t));
	if(NULL == CAN_Buffer_Handle)
	{
		printf("CAN StreamBuffer Create Failure\r\n");
	}

	xReturn = xTaskCreate(CMD_Parser_Task, "CMD_Parser_Task", 256, NULL, 3, &CMD_Parser_Task_Handle);
	if(pdPASS != xReturn)
	{
		printf("CMD_Parser_Task Create Error\r\n");
	}

	//printf("Free heap size before creating LED_Task: %u\n", (unsigned int)xPortGetFreeHeapSize());
	xReturn = xTaskCreate(LED_Task, "LED_Task", 128, NULL, 2, &LED_Task_Handle);
	if(pdPASS != xReturn)
	{
		printf("LED_Task Create Error\r\n");
	}

	//printf("Free heap size before creating SHT20_Task: %u\n", (unsigned int)xPortGetFreeHeapSize());
	xReturn = xTaskCreate(SHT20_Report_Task, "SHT20_Report_Task", 256, NULL, 2, &SHT20_Report_Task_Handle);
	if(pdPASS != xReturn)
	{
		printf("SHT20_Report_Task Create Error\r\n");
	}

	//printf("create task over\r\n");

	vTaskDelete(NULL);

	taskEXIT_CRITICAL();
}

static void CMD_Parser_Task(void* parameter)
{
	uint8_t		Get_Data[8];
	size_t		xReceivedBytes;
	uint64_t	rec_command = 0;

	while(1)
	{
		printf("999\r\n");
		//从流缓冲区中获取CAN消息
		xReceivedBytes = xStreamBufferReceive(CAN_Buffer_Handle, Get_Data, sizeof(Get_Data), portMAX_DELAY);
		if(xReceivedBytes == sizeof(Get_Data))
		{
			printf("Data Receive to Stream Buffer successfully.\r\n");
			for(int i=0; i<8; ++i)
			{
				printf("Grt_Data: %02x\n", Get_Data[i]);
				//将接收到的字节转换为64位整数(移位与运算)
				rec_command = (rec_command << 8) | Get_Data[i];
			}

			//rec_command = *((uint64_t *)Get_Data);
			//memcpy(&rec_command, Get_Data, sizeof(rec_command));

			//解析命令，根据命令类型设置相应的事件位
			for(int i = 0; i < CMD_COUNT; i++)
			{
				if(rec_command == CMD_LIST[i])
				{
					switch(i)
					{
						case 0:
							xEventGroupSetBits(Event_Handle, LED_ON_EVENT);
							printf("LED_ON_EVENT\r\n");
							break;
						case 1:
							xEventGroupSetBits(Event_Handle, LED_OFF_EVENT);
							printf("LED_OFF_EVENT\r\n");
							break;
						case 2:
							xEventGroupSetBits(Event_Handle, SHT20_ON_EVENT);
							printf("SHT2_ON_EVENT\r\n");
							break;
						case 3:
							xEventGroupSetBits(Event_Handle, SHT20_OFF_EVENT);
							printf("SHT20_OFF_EVENT\r\n");
							break;
						default:
						    printf("Unknown command\r\n");
						    break;
					 }
				break;
				}
			}
		}
	}
}

static void LED_Task(void* parameter)
{
	EventBits_t r_event;

	while(1)
	{
		printf("888\r\n");
		r_event = xEventGroupWaitBits(Event_Handle, LED_ON_EVENT|LED_OFF_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);
		if((r_event & LED_ON_EVENT) == LED_ON_EVENT)
		{
			turn_led(RedLed, ON);
			vTaskDelay(pdMS_TO_TICKS(2000));
			turn_led(GreenLed, ON);
			vTaskDelay(pdMS_TO_TICKS(2000));
			turn_led(BlueLed, ON);
		}

		if((r_event & LED_OFF_EVENT) == LED_OFF_EVENT)
		{
			printf("555\r\n");
			turn_led(RedLed, OFF);
			vTaskDelay(pdMS_TO_TICKS(2000));
			turn_led(GreenLed, OFF);
			vTaskDelay(pdMS_TO_TICKS(2000));
			turn_led(BlueLed, OFF);
		}
	}
}

static void SHT20_Report_Task(void* parameter)
{
	EventBits_t 		r_event;
	uint8_t				length = 8;
	uint8_t				res;
	uint8_t				buf[64];
	size_t				buf_size = sizeof(buf);
	SHT20_Task_State_t	task_state = IDLE;

	while(1)
	{

#if 1
		switch(task_state)
		{
			case IDLE:
				r_event = xEventGroupWaitBits(Event_Handle, SHT20_ON_EVENT|SHT20_OFF_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);
				if((r_event & SHT20_ON_EVENT) == SHT20_ON_EVENT)
				//if(xEventGroupWaitBits(Event_Handle, SHT20_ON_EVENT, pdTRUE, pdTRUE, portMAX_DELAY))
				{
					task_state = SAMPLING;
				}
				break;

			case SAMPLING:
				FloatToByte(buf, buf_size);
				task_state = REPORTING;
				break;

			case REPORTING:
				res=CAN_TX_Message(buf, (uint8_t)length);
				if( res < 0 )
				{
					printf("CAN report error\r\n");
				}

				vTaskDelay(pdMS_TO_TICKS(3000));

				r_event = (xEventGroupGetBits(Event_Handle));
				//r_event = xEventGroupWaitBits(Event_Handle, SHT20_OFF_EVENT, pdTRUE, pdTRUE, portMAX_DELAY);
				if((r_event & SHT20_OFF_EVENT) == SHT20_OFF_EVENT)
				{
					task_state = STOPPED;
				}
				else
				{
					task_state = SAMPLING;
				}

				break;

			case STOPPED:
				printf("SHT20 Report Stop\r\n");
				task_state = IDLE;
				break;

			default:
				break;
		}
#endif

#if 0
		printf("777\r\n");
		r_event = xEventGroupWaitBits(Event_Handle, SHT20_ON_EVENT|SHT20_OFF_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);
		if((r_event & SHT20_ON_EVENT) == SHT20_ON_EVENT)
		{
			while(1)
			{
				FloatToByte(buf, buf_size);
				res=CAN_TX_Message(buf, (uint8_t)length);
				if( res < 0 )
				{
					printf("CAN report error\r\n");
				}
				vTaskDelay(pdMS_TO_TICKS(3000));

				//if((r_event & SHT20_OFF_EVENT) == SHT20_OFF_EVENT)
				if (xEventGroupGetBits(Event_Handle) & SHT20_OFF_EVENT)
				{
					printf("SHT20 Report Stop\r\n");
					break;
				}
			}
		}

#endif
	}

}

void FloatToByte(uint8_t *buf, size_t buf_size)
{
	float	temperature;
	float	humidity;

	if( SHT20_SampleData(&temperature, &humidity) <0 )
	{
		printf("ERROR: SHT20 Sample Data failure\r\n");
		return ;
	}

	int temp_int = (int)(temperature * 100)/100;
	int temp_dec = (int)(temperature * 100)%100;
	int humd_int = (int)(humidity * 100)/100;
	int humd_dec = (int)(humidity * 100)%100;

	buf[0] = 0x00;
	buf[1] = 0x01;
	buf[2] = 0x00;
	buf[3] = 0x04;
	buf[4] = (uint8_t)temp_int;
	buf[5] = (uint8_t)temp_dec;
	buf[6] = (uint8_t)humd_int;
	buf[7] = (uint8_t)humd_dec;

	printf("temperature: %.2f  humidity: %.2f\r\n", temperature, humidity);
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
