/*
 * sht20.h
 *
 *  Created on: Jun 17, 2024
 *      Author: ASUS
 */

#ifndef INC_SHT20_H_
#define INC_SHT20_H_

#include "stm32l4xx_hal.h"

#define SHT20_ADDR		0x40

#define SHT20_ADDR_WR	(SHT20_ADDR<<1)
#define SHT20_ADDR_RD	((SHT20_ADDR<<1) | 0x01)

#define SHT20_DATA_SIZE		3
#define SHT20_POLYNOMIAL 	0x31

typedef enum
{
	MEASURE_HUMIDITY_CMD =0xE5,
	MEASURE_TEMPERATURE_CMD =0xE3,
	SOFT_RESET_CMD =0xFE,
	READ_USER_REG_CMD =0xE7,
	WRITE_USER_REG_CMD =0xE6,

}SHT20_CMD;

extern int SHT20_SampleData(float *temperature, float *humidity);


#endif /* INC_SHT20_H_ */
