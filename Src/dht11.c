/*
 * dht11.c
 *
 *  Created on: Jun 12, 2024
 *      Author: ASUS
 */

#include "tim.h"
#include "gpio.h"
#include "main.h"

typedef struct w1_gpio_s
{
	GPIO_TypeDef		*group;
	uint16_t			pin;
}w1_gpio_t;

static w1_gpio_t W1Dat =
{
		.group = GPIOB,
		.pin   = GPIO_PIN_4,
};

#define W1DQ_Input()  \
{  \
	GPIO_InitTypeDef GPIO_InitStruct = {0};\
	GPIO_InitStruct.Pin = W1Dat.pin;\
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;\
	GPIO_InitStruct.Pull = GPIO_PULLUP;\
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;\
	HAL_GPIO_Init(W1Dat.group, &GPIO_InitStruct);\
}

#define W1DQ_Output()  \
{  \
	GPIO_InitTypeDef GPIO_InitStruct = {0};\
	GPIO_InitStruct.Pin = W1Dat.pin;\
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;\
	GPIO_InitStruct.Pull = GPIO_NOPULL;\
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;\
	HAL_GPIO_Init(W1Dat.group, &GPIO_InitStruct);\
}

#define W1DQ_Write(x)	HAL_GPIO_WritePin(W1Dat.group, W1Dat.pin, \
							(x==1)?GPIO_PIN_SET:GPIO_PIN_RESET)

#define W1DQ_Read()		HAL_GPIO_ReadPin(W1Dat.group, W1Dat.pin)

static void DHT11_StartSignal(void)
{
	W1DQ_Output();

	W1DQ_Write(0);
	HAL_Delay(20);

	W1DQ_Write(1);
	delay_us(30);

	W1DQ_Input();
}

uint8_t DHT11_RespondSignal(void)
{
	uint8_t	retry = 0;

	while( W1DQ_Read() && retry < 100 )
	{
		retry++;
		delay_us(1);
	}

	if(retry >= 100)
		return 1;

	retry = 0;
	while( !W1DQ_Read() && retry <100 )
	{
		retry++;
		delay_us(1);
	}

	if(retry >= 100)
		return 1;

	return 0;
}

uint8_t DHT11_ReadBit(void)
{
	uint8_t retry = 0;

	while( W1DQ_Read() && retry<100 )
	{
		retry++;
		delay_us(1);
	}

	retry = 0;
	while( !W1DQ_Read() && retry<100 )
	{
		retry++;
		delay_us(1);
	}

	delay_us(40);
	if( W1DQ_Read() )
		return 1;
	else
		return 0;
}

uint8_t DHT11_ReadByte(void)
{
	uint8_t		i,dat;

	dat = 0;
	for(i=0; i<8; i++)
	{
		dat <<= 1;
		dat |= DHT11_ReadBit();
	}

	return dat;
}

int DHT11_SampleData(float *temperature, float *humidity)
{
	uint8_t		humi_H8bit;
	uint8_t		humi_L8bit;
	uint8_t		temp_H8bit;
	uint8_t		temp_L8bit;
	uint8_t		check_sum;

	if( !temperature || !humidity )
		return -1;

	DHT11_StartSignal();
	if( 0 != DHT11_RespondSignal() )
		return -2;

	humi_H8bit = DHT11_ReadByte();
	humi_L8bit = DHT11_ReadByte();
	temp_H8bit = DHT11_ReadByte();
	temp_L8bit = DHT11_ReadByte();
	check_sum = DHT11_ReadByte();

	if( ( humi_H8bit+humi_L8bit+temp_H8bit+temp_L8bit) != check_sum )
		return -3;

	*humidity = (humi_H8bit*100 + humi_L8bit)/100.00;
	*temperature = (temp_H8bit*100 + temp_L8bit)/100.00;

	return 0;
}
