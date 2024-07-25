/*
 * sht20.c
 *
 *  Created on: Jun 16, 2024
 *      Author: ASUS
 */
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "sht20.h"
#include "tim.h"

//#define CONFIG_GPIO_I2C

#ifdef CONFIG_GPIO_I2C
#include "gpio_i2c_sht20.h"
#else
#include "i2c.h"
#endif

#define CONFIG_SHT20_DEBUG

#ifdef CONFIG_SHT20_DEBUG
#define sht20_print(format,args...) printf(format,##args)
#else
#define sht20_print(format,args...) do{} while(0)
#endif

static int sht20_send_cmd(SHT20_CMD cmd)
{
	uint8_t buf[1];

	buf[0] = cmd;

#ifdef CONFIG_GPIO_I2C
	return I2C_Master_Transmit(SHT20_ADDR_WR, (uint8_t*)buf, 1);
#else
	return HAL_I2C_Master_Transmit(&hi2c1, SHT20_ADDR_WR, (uint8_t*)buf, 1, 0xFFFF);
#endif

}

static void sht20_soft_reset(void)
{
	sht20_send_cmd(SOFT_RESET_CMD);
	HAL_Delay(1);
}

static int sht20_single_shot_measurement(uint8_t *humd_buf, uint8_t humd_buf_size,uint8_t *temp_buf, uint8_t temp_buf_size)
{
	uint8_t		rv;

	if( !humd_buf || !temp_buf || humd_buf_size<SHT20_DATA_SIZE || temp_buf_size<SHT20_DATA_SIZE )
	{
		sht20_print("%s(): Invalid input arguments\n", __func__);
		return -1;
	}

	rv = sht20_send_cmd(MEASURE_HUMIDITY_CMD);
	if( rv )
	{
		sht20_print("error: SHT20 send humidity measurement command failure, rv=%d\n", rv);
		sht20_soft_reset();
		return -2;
	}

#ifdef CONFIG_GPIO_I2C
	rv = I2C_Master_Receive(SHT20_ADDR_RD, humd_buf, SHT20_DATA_SIZE);
#else
	rv = HAL_I2C_Master_Receive(&hi2c1, SHT20_ADDR_RD, humd_buf, SHT20_DATA_SIZE, 0xFFFF);
#endif

	if(rv)
	{
		sht20_print("error: SHT20 read humidity measurement result failure, rv=%d\n", rv);
		return -3;
	}

	HAL_Delay(100);

	rv = sht20_send_cmd(MEASURE_TEMPERATURE_CMD);
		if( rv )
		{
			sht20_print("error: SHT30 send temperature measurement command failure, rv=%d\n", rv);
			sht20_soft_reset();
			return -2;
		}

#ifdef CONFIG_GPIO_I2C
	rv = I2C_Master_Receive(SHT20_ADDR_RD, temp_buf, SHT20_DATA_SIZE);
#else
	rv = HAL_I2C_Master_Receive(&hi2c1, SHT20_ADDR_RD, temp_buf, SHT20_DATA_SIZE, 0xFFFF);
#endif

		if(rv)
		{
			sht20_print("error: SHT20 read temperature measurement result failure, rv=%d\n", rv);
			return -3;
		}

	return 0;
}

static uint8_t sht20_crc8(const uint8_t *data, int len)
{
	uint8_t			crc = 0x00;
	int				i, j;

	for(i=0; i<len; ++i)
	{
		crc ^=  *data++;

		for(j=0; j<8; ++j)
		{
			crc = ( crc & 0x80 ) ? (crc<<1) ^ SHT20_POLYNOMIAL: (crc << 1);
		}
	}

	return crc;
}

int SHT20_SampleData(float *temperature, float *humidity)
{
	uint8_t		humd_buf[SHT20_DATA_SIZE];
	uint8_t		temp_buf[SHT20_DATA_SIZE];
	int			rv;
	uint16_t	humd;
	uint16_t	temp;
	uint8_t		humd_crc;
	uint8_t		temp_crc;
	if( !temperature || !humidity)
	{
		sht20_print("%s(): Invalid input arguments\n", __func__);
		return -1;
	}

	rv = sht20_single_shot_measurement(humd_buf, SHT20_DATA_SIZE, temp_buf, SHT20_DATA_SIZE);
	if( rv )
	{
		sht20_print("SHT20 single short measurement failure, rv=%d\n", rv);
		return -2;
	}
#ifdef CONFIG_SHT30_DEBUG
	{
		int		i;

		sht20_print("SHT20 get %d bytes sample data: \n", SHT20_DATA_SIZE);
		for(i=0; i<SHT20_DATA_SIZE; i++)
		{
			sht20_print("0x%02x", buf[i]);
		}
		sht20_print("\n");
	}
#endif

	humd_crc = sht20_crc8(humd_buf, 2);
	//sht20_print("SHT20 humidity Cal_CRC: [%02x] EXP_CRC: [%02x]\n", humd_crc, humd_buf[2]);
	if( humd_crc != humd_buf[2])
	{
		sht20_print("SHT20 measurement humidity got CRC error\n");
		return -3;
	}

	temp_crc = sht20_crc8(temp_buf, 2);
	//sht20_print("SHT30 temperature Cal_CRC: [%02x] EXP_CRC: [%02x]\n", temp_crc, temp_buf[2]);
	if( temp_crc != temp_buf[2])
	{
		sht20_print("SHT20 measurement temperature got CRC error\n");
		return -4;
	}

	humd = (humd_buf[0]<<8) | humd_buf[1];
	temp = (temp_buf[0]<<8) | temp_buf[1];

	*temperature = -46.85 + 175.72*((float)temp/65536);
	*humidity = -6 + 125 * ((float)humd / 65536);

	return 0;
}

