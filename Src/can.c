/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END 0 */

CAN_HandleTypeDef 	hcan1;
extern uint8_t		led_state;

int RingBuffer_Write(RingBuffer *cb, uint8_t data);

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterActivation = ENABLE;//打开过滤�?
  sFilterConfig.FilterBank = 0;//过滤器组，可设置�?0-13
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//采用掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//采用32位掩�?
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;//采用邮箱FIFO0

  sFilterConfig.FilterIdHigh = 0x0000;//设置过滤器高16�?
  sFilterConfig.FilterIdLow = 0x0000;//设置过滤器低16�?
  sFilterConfig.FilterMaskIdHigh = 0x0000;//设置过滤器掩码高16�?
  sFilterConfig.FilterMaskIdLow = 0x0000;//设置掩码�?16�?

  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)//调用接口，初始化过滤�?
  {
	  Error_Handler();
  }

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

int CAN_TX_Message(uint8_t *TxData, int length)
{
	CAN_TxHeaderTypeDef 		Tx_pHeader;
	uint32_t 					TxMailboxNumber = 0x00000000U;

	Tx_pHeader.StdId = 0x122;//标准帧ID
	Tx_pHeader.ExtId = 0x0000;//扩展帧ID（此处无用）
	Tx_pHeader.IDE = CAN_ID_STD;
	Tx_pHeader.RTR = CAN_RTR_DATA;//数据�?
	Tx_pHeader.DLC = length;//发�?�报文的长度
	Tx_pHeader.TransmitGlobalTime = DISABLE;//时间戳不使能

	if(HAL_CAN_AddTxMessage(&hcan1, &Tx_pHeader, TxData, &TxMailboxNumber) != HAL_OK)
	{
		printf("5\r\n");
	}
	/*else
	{
		printf("55\r\n");
		printf("%lu\r\n", TxMailboxNumber);
	}
	*/
	return TxMailboxNumber;
}


#if 1

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef 	RxHeader;
	uint8_t					RxData[8];

	/*从FIFO0队列中接收到报文，并存储在'RxData'数组中*/
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
	{
		printf("RxData: ");
		for (int i = 0; i < 8; i++)
		{
		   printf("%02X ", RxData[i]); // 打印为十六进制格式
		}
		printf("\r\n");

		for(int i = 0; i<8; i++)
		{
			RingBuffer_Write(&canRxBuffer, RxData[i]);//一次只能写入一个字节到循环缓冲区
		}
#if 0
		printf("Buffer: ");
		for (int i = 0; i < BUFFER_SIZE; i++)
		{
		   printf("%02X ", canRxBuffer.buffer[i]); // 打印为十六进制格式
		}
		printf("\r\n");
#endif

	}
}

void process_led(uint8_t led_state)
{
	static	uint8_t	current_led =0;

	switch(current_led)
	{
		case 0:
			turn_led(RedLed, led_state);
			current_led = 1;
			break;
		case 1:
			turn_led(GreenLed, led_state);
			current_led = 2;
			break;
		case 2:
			turn_led(BlueLed, led_state);
			current_led = 0;
			break;
		default:
			current_led = 0;
			break;
	}

}
#endif

#if 0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	CAN_RxHeaderTypeDef RxHeader;
	uint8_t 			RxData[8];
	uint8_t 			i;

	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
	{
		printf("\r\nGet Rx Message Success!!\r\n");
		printf("[ID]:%lx [DLC]:%ld [Data]: ", RxHeader.StdId, RxHeader.DLC);
		for(i=0; i<RxHeader.DLC; i++)
		{
			printf("%x", RxData[i]);
		}
		printf("\r\n");
	}
}
#endif

/* USER CODE END 1 */
