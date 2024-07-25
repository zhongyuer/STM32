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
#include <ring_buffer.h>
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
int CAN_TX_Message(uint8_t *TxData, int length);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
int test_candata(void);
void process_led(uint8_t led_state);
int Upload_Data(uint8_t *buf, size_t buf_size);
//int Report_tempRH(char *buf, size_t buf_size);
void FloatToByte(uint8_t *buf, size_t buf_size);
int SHT20_SampleData(float *temperature, float *humidity);
int RingBuffer_Read(RingBuffer *cb, uint8_t *data);
void RxData_Analyze(void);

uint8_t					led_state = 0;
uint8_t					temp_humd_state = 0;
uint8_t					read_data[8] = {0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint32_t		last_tick = 0;
	uint8_t			buf[32];
	size_t			buf_size = sizeof(buf);
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  RingBuffer_Init(&canRxBuffer);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /*启动CAN并启用中断*/
  if(HAL_CAN_Start(&hcan1) != HAL_OK)
    {
  	  Error_Handler();
    }

  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
     {
     	Error_Handler();
     }


   /* USER CODE END CAN1_Init 2 */

  while (1)
  {

	#if 0
	  //CAN_TX_Message("hello", 5);
	  if( test_candata() < 0)
	  {
		  printf("TX ERROE\r\n");
	  }
	  HAL_Delay(1000);
	#endif

	  /*从环形缓冲区里读取数据*/
	  for(int i = 0; i<8; i++)
	  {
		  RingBuffer_Read(&canRxBuffer, &read_data[i]);
	  }

#if 0
	  printf("main buffer: ");
	  for (int i = 0; i < BUFFER_SIZE; i++)
	  {
		  printf("%02X ", canRxBuffer.buffer[i]); // 打印为十六进制格式
	  }
	  printf("\r\n");
#endif

	  /*解析数据*/
	  RxData_Analyze();
	  //printf("after ananlyzing data led_state: %d, temp_humd_state: %d\r\n", led_state, temp_humd_state);

	  /*根据解析结果处理LED状态*/
	  if(led_state != 0)
	  {
		if(HAL_GetTick() - last_tick >= 2000)
		{
			last_tick = HAL_GetTick();
			process_led(led_state == 1 ? 1 : 0);
		}
	  }

	  /*根据解析结果处理温湿度上报*/
	  if( temp_humd_state ==1 ? 1 : 0 )
	  {
		  if(HAL_GetTick() - last_tick >= 2000)
		  {
			  last_tick = HAL_GetTick();
			  Upload_Data(buf, buf_size);

#if 0
			  for(int i = 0; i < 8; i++)
			  {
				  printf("TX data: %u\r\n", buf[i]);
			  }
#endif
		  }
	  }
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

#if 0
uint8_t TxData[8];
uint8_t length = 8;
uint8_t res1;
uint8_t count=0;
uint8_t i=0;

int test_candata()
{
	printf("send data: ");
	for(i=0; i<8; i++)
	{
		TxData[i]=count+1;
		count++;
		printf("%d\t", TxData[i]);
	}
	printf("\r\n");
	res1=CAN_TX_Message(TxData, length);
	if( res1 < 0 )
	{
		printf("CAN TX error\r\n");
		return -1;
	}
	//printf("999\r\n");
    return 0;
}
#endif

/*上报温湿度*/
int Upload_Data(uint8_t *buf, size_t buf_size)
{
	uint8_t		length = 8;
	uint8_t		res;

	FloatToByte(buf, buf_size);

	res=CAN_TX_Message(buf, (uint8_t)length);
	if( res < 0 )
	{
		printf("CAN report error\r\n");
		return -1;
	}
	return 0;
}

/*解析接收到的报文命令*/
void RxData_Analyze(void)
{

#if 0
	  printf("read_data: ");
	  for (int i = 0; i < 8; i++)
	  {
		  printf("%02X ",read_data[i]); // 打印为十六进制格式
	  }
	  printf("\r\n");
#endif

	uint8_t data_led_on[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};//打开LED灯
	uint8_t data_led_off[] = {0x10, 0x22, 0x33, 0x44, 0x55, 0x66,0x77, 0x88};//关闭LED灯

	uint8_t data_temp_humd_on[] = {0x10, 0x20, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};//打开温湿度上报
	uint8_t data_temp_humd_off[] = {0x10, 0x20, 0x30, 0x44, 0x55, 0x66, 0x77, 0x88};//关闭温度上报

	if(memcmp(read_data, data_led_on, 8) == 0)
	{
		led_state = 1;
	}
	else if(memcmp(read_data, data_led_off, 8) == 0)
	{
		led_state = 2;
	}
	else if(memcmp(read_data, data_temp_humd_on, 8) == 0)
	{
		temp_humd_state = 1;
	}
	else if(memcmp(read_data, data_temp_humd_off, 8) == 0)
	{
		temp_humd_state = 2;
	}
	else
	{
		led_state = 0;
		temp_humd_state = 0;
	}
}

/*以TLV格式上报数据*/
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
