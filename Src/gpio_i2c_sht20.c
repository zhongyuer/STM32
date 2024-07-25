/*
 * gpio_i2c_sht20.c
 *
 *  Created on: Jun 18, 2024
 *      Author: ASUS
 */

#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "gpio_i2c_sht20.h"

#define I2C_CLK_STRETCH_TIMEOUT		50

//#define CONFIG_GPIO_I2C_DEBUG

#ifdef CONFIG_GPIO_I2C_DEBUG
#define i2c_print(format, args...) printf(format, ##args)
#else
#define i2c_print(format, args...) do{} while(0)
#endif

typedef struct i2c_gpio_s
{
	GPIO_TypeDef		*group;
	uint16_t			scl;
	uint16_t			sda;
} i2c_gpio_t;

static i2c_gpio_t	i2c_pins = {GPIOB, GPIO_PIN_6, GPIO_PIN_7};

#define SDA_IN()	do{ GPIO_InitTypeDef GPIO_InitStruct = {0};\
							GPIO_InitStruct.Pin = i2c_pins.sda;\
							GPIO_InitStruct.Mode = GPIO_MODE_INPUT;\
							GPIO_InitStruct.Pull = GPIO_PULLUP;\
							GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;\
							HAL_GPIO_Init(i2c_pins.group, &GPIO_InitStruct);\
						}while(0)

#define SDA_OUT()	do{ GPIO_InitTypeDef GPIO_InitStruct = {0};\
							GPIO_InitStruct.Pin = i2c_pins.sda;\
							GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;\
							GPIO_InitStruct.Pull = GPIO_PULLUP;\
							GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;\
							HAL_GPIO_Init(i2c_pins.group, &GPIO_InitStruct);\
						}while(0)

#define SCL_OUT()		do{ GPIO_InitTypeDef GPIO_InitStruct = {0};\
							GPIO_InitStruct.Pin = i2c_pins.scl;\
							GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;\
							GPIO_InitStruct.Pull = GPIO_PULLUP;\
							GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;\
							HAL_GPIO_Init(i2c_pins.group, &GPIO_InitStruct);\
						}while(0)

#define SCL_H()		HAL_GPIO_WritePin(i2c_pins.group, i2c_pins.scl, GPIO_PIN_SET)
#define SCL_L()		HAL_GPIO_WritePin(i2c_pins.group, i2c_pins.scl, GPIO_PIN_RESET)
#define SDA_H()		HAL_GPIO_WritePin(i2c_pins.group, i2c_pins.sda, GPIO_PIN_SET)
#define SDA_L()		HAL_GPIO_WritePin(i2c_pins.group, i2c_pins.sda, GPIO_PIN_RESET)

#define READ_SDA()	HAL_GPIO_ReadPin(i2c_pins.group, i2c_pins.sda)
#define READ_SCL()	HAL_GPIO_ReadPin(i2c_pins.group, i2c_pins.scl)

static inline uint8_t I2c_WaitWhileClockStretching(uint16_t timeout)
{
	while( timeout-- > 0 )
	{
		if( READ_SCL() )
			break;
		delay_us(1);
	}
	return timeout ? NO_ERROR : BUS_ERROR;
}

uint8_t I2c_StartCondition()
{
	uint8_t		rv = NO_ERROR;

	SDA_OUT();
	SCL_OUT();

	SDA_H();
	delay_us(1);
	SCL_H();
	delay_us(1);

#ifdef I2C_CLK_STRETCH_TIMEOUT
		rv = I2c_WaitWhileClockStretching(I2C_CLK_STRETCH_TIMEOUT);
		if( rv )
		{
			i2c_print("ERROR: %s() I2C bus busy\n", __func__);
			return rv;
		}
#endif
		SDA_L();
		delay_us(2);

		SCL_L();
		delay_us(2);

		return rv;
}

uint8_t I2c_StopCondition(void)
{
	uint8_t		rv = NO_ERROR;
	SDA_OUT();

	SCL_L();
	SDA_L();
	delay_us(2);

	SCL_H();
	delay_us(2);

	return rv;
}

uint8_t I2c_WriteByte(uint8_t byte)
{
	uint8_t rv = NO_ERROR;
	uint8_t mask;

	SDA_OUT();
	SCL_L();

	for(mask=0x80; mask>0; mask>>=1)
	{
		if((mask & byte) == 0)
			SDA_L();
		else
			SDA_H();

		delay_us(1);

		SCL_H();
		delay_us(5);

#ifdef I2C_CLK_STRETCH_TIMEOUT
		rv = I2c_WaitWhileClockStretching(I2C_CLK_STRETCH_TIMEOUT);
		if( rv )
		{
			i2c_print("ERROR: %s() I2C bus busy\n", __func__);
			goto OUT;
		}
#endif
		SCL_L();
		delay_us(1);
	}

	SDA_IN();
	SCL_H();
	delay_us(1);

#ifdef I2C_CLK_STRETCH_TIMEOUT
		rv = I2c_WaitWhileClockStretching(I2C_CLK_STRETCH_TIMEOUT);
		if( rv )
		{
			i2c_print("ERROR: %s() I2C bus busy\n", __func__);
			goto OUT;
		}
#endif

		if( READ_SDA() )
			rv = ACK_ERROR;
OUT:
		SCL_L();
		delay_us(20);

		return rv;
}

uint8_t I2c_ReadByte(uint8_t *byte, uint8_t ack)
{
	uint8_t rv = NO_ERROR;
	uint8_t mask;

	*byte = 0x00;

	SDA_IN();

	for(mask = 0x80; mask > 0; mask>>= 1)
	{
		SCL_H();
		delay_us(1);

#ifdef I2C_CLK_STRETCH_TIMEOUT
			rv = I2c_WaitWhileClockStretching(I2C_CLK_STRETCH_TIMEOUT);
			if( rv )
			{
				i2c_print("ERROR: %s() I2C bus busy\n", __func__);
				goto OUT;
			}
#endif

			if( READ_SDA() )
				*byte |= mask;

			SCL_L();
			delay_us(1);
	}

	if(ack == ACK)
	{
		SDA_OUT();
		SDA_L();
	}
	else if( ack == NAK )
	{
		SDA_OUT();
		SDA_H();
	}

	delay_us(1);
	SCL_H();
	delay_us(2);

#ifdef I2C_CLK_STRETCH_TIMEOUT
	rv = I2c_WaitWhileClockStretching(I2C_CLK_STRETCH_TIMEOUT);
	if( rv )
	{
		i2c_print("ERROR: %s() I2C bus busy\n", __func__);
	}
#endif

OUT:
		SCL_L();
		delay_us(2);

	return rv;
}

uint8_t I2c_SendAddress(uint8_t addr)
{
	return I2c_WriteByte(addr);
}

int I2C_Master_Receive(uint8_t addr, uint8_t *buf, int len)
{
	int			i;
	int			rv = NO_ERROR;
	uint8_t 	byte;

	I2c_StartCondition();

	rv = I2c_SendAddress(addr);
	if( rv )
	{
		i2c_print("Send I2C read address failure, rv=%d\n", rv);
		goto OUT;
	}

#ifdef I2C_CLK_STRETCH_TIMEOUT
	rv = I2c_WaitWhileClockStretching(I2C_CLK_STRETCH_TIMEOUT);
	if( rv )
	{
		i2c_print("ERROR: %s() I2C wait clock stretching failure, rv=%d\n", __func__, rv);
		return rv;
	}
#endif

	for ( i=0; i<len; i++)
	{
		if( !I2c_ReadByte(&byte, ACK) )
			buf[i] = byte;
		else
			goto OUT;
	}

OUT:
	I2c_StopCondition();
	return rv;
}

int I2C_Master_Transmit(uint8_t addr, uint8_t *data, int bytes)
{
	int		i;
	int 	rv = NO_ERROR;

	if(!data)
	{
		return FARM_ERROR;
	}

	i2c_print("I2C Master start transmit [%d] bytes data to addr[0x%02x]\n", bytes, addr);
	I2c_StartCondition();

	rv = I2c_SendAddress(addr);
	if( rv )
	{
		goto OUT;
	}

	for(i=0; i<bytes; i++)
	{
		if( NO_ERROR != (rv=I2c_WriteByte(data[i])) )
		{
			break;
		}
	}

OUT:
	I2c_StopCondition();
	return rv;
}



