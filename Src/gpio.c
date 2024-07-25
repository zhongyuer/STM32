/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BlueLed_GPIO_Port, BlueLed_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GreenLed_Pin|RedLed_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BlueLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BlueLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = Key1_Pin|Key2_Pin|Key3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = GreenLed_Pin|RedLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */



gpio_t		leds[LedMax] =
{
	{ "RedLed", RedLed_GPIO_Port, RedLed_Pin},
	{ "GreenLed", GreenLed_GPIO_Port, GreenLed_Pin},
	{ "BlueLed", BlueLed_GPIO_Port, BlueLed_Pin},
};

gpio_t	relays[RelayMax] =
{
		{ "Relay1", Key1_GPIO_Port, Key1_Pin},
		{ "Relay2", Key2_GPIO_Port, Key2_Pin},
};

void turn_led(int which, int status)
{
	GPIO_PinState		level;

	if( which >= LedMax )
		return;
	level = status==OFF ? GPIO_PIN_SET : GPIO_PIN_RESET;

	HAL_GPIO_WritePin(leds[which].group, leds[which].pin, level);
}

void blink_led(int which, uint32_t interval)
{
	turn_led(which, ON);
	HAL_Delay(interval);

	turn_led(which, OFF);
	HAL_Delay(interval);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if( Key1_Pin == GPIO_Pin )
	{
		blink_led(BlueLed, 500);
	}

	else if( Key2_Pin == GPIO_Pin )
	{
		blink_led(RedLed, 500);
	}

	else if( Key3_Pin == GPIO_Pin )
	{
		blink_led(GreenLed, 500);
	}
}

/* USER CODE END 2 */
