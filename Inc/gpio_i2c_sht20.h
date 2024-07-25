/*
 * gpio_i2c_sht20.h
 *
 *  Created on: Jun 19, 2024
 *      Author: ASUS
 */

#ifndef INC_GPIO_I2C_SHT20_H_
#define INC_GPIO_I2C_SHT20_H_

enum
{
	NO_ERROR		=0x00,
	FARM_ERROR		=0x01,
	ACK_ERROR		=0x02,
	CHECKSUM_ERROR	=0x04,
	TIMEOUT_ERROR	=0x08,
	BUS_ERROR		=0010,
};

enum
{
	ACK_NUNE,
	ACK,
	NAK,
};

extern int I2C_Master_Receive(uint8_t addr, uint8_t *buf, int len);

extern int I2C_Master_Transmit(uint8_t addr, uint8_t *data, int bytes);


#endif /* INC_GPIO_I2C_SHT20_H_ */
