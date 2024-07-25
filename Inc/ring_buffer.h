/*
 * circular_buffer.h
 *
 *  Created on: Jul 2, 2024
 *      Author: ASUS
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include <stdint.h>

#define BUFFER_SIZE 128

typedef struct
{
	uint8_t 	buffer[BUFFER_SIZE];
	uint16_t	head;
	uint16_t	tail;
	uint16_t	count;
}RingBuffer;

extern RingBuffer canRxBuffer;

void RingBuffer_Init(RingBuffer *cb);
int RingBuffer_write(RingBuffer *cb, uint8_t data);
int RingBuffer_Read(RingBuffer *cb, uint8_t *data);


#endif /* INC_RING_BUFFER_H_ */
