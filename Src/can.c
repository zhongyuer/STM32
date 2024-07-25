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
#include "can.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include "gpio.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

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

  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;

  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;

  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
	  Error_Handler();
  }

  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
      Error_Handler();
  }

  if(HAL_CAN_Start(&hcan1) != HAL_OK)
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
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
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
	CAN_TxHeaderTypeDef		Tx_pHeader;
	uint32_t				TxMailboxNumber = 0x00000000U;

	Tx_pHeader.StdId = 0x122;
	Tx_pHeader.ExtId = 0x0000;
	Tx_pHeader.IDE = CAN_ID_STD;
	Tx_pHeader.RTR = CAN_RTR_DATA;
	Tx_pHeader.DLC = length;
	Tx_pHeader.TransmitGlobalTime = DISABLE;

	if(HAL_CAN_AddTxMessage(&hcan1, &Tx_pHeader, TxData, &TxMailboxNumber) != HAL_OK)
	{
		printf("TX error\r\n");
	}
	return TxMailboxNumber;

}

extern StreamBufferHandle_t CAN_Buffer_Handle;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	printf("CAN message received\r\n");

	CAN_RxHeaderTypeDef		RxHeader;
	uint8_t					RxData[8];
	BaseType_t 				xHigherPriorityTaskWoken = pdFALSE;
	size_t 					xSentBytes;

	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
	{
		printf("RxData: ");
		for (int i = 0; i < 8; i++)
		{
			printf("%02X ", RxData[i]); // 打印为十六进制格式
		}
		printf("\r\n");

		xSentBytes = xStreamBufferSendFromISR(CAN_Buffer_Handle, RxData, sizeof(RxData), &xHigherPriorityTaskWoken);
		if (xSentBytes == sizeof(RxData))
		{
			printf("Data sent to Stream Buffer successfully.\r\n");
		}

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

}
/* USER CODE END 1 */
