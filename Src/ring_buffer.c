/*
 * circular_buffer.c
 *
 *  Created on: Jul 2, 2024
 *      Author: ASUS
 */

#include <ring_buffer.h>

RingBuffer canRxBuffer;

void RingBuffer_Init(RingBuffer *cb)
{
	cb->head = 0;
	cb->tail = 0;
	cb->count = 0;
}

int RingBuffer_Write(RingBuffer *cb, uint8_t data)
{
	if(cb->count >= BUFFER_SIZE)
	{
		return -1;//缓存区已满
	}

	cb->buffer[cb->head] = data;
	cb->head = (cb->head + 1) % BUFFER_SIZE;
	cb->count++;

	return 0;
}

int RingBuffer_Read(RingBuffer *cb, uint8_t *data)
{

	if(cb->count == 0)
	{
		return -1;//缓冲区为空
	}

	*data = cb->buffer[cb->tail];
	//printf("Read data from buffer: 0x%02X at position %d\n", *data, cb->tail);
	cb->buffer[cb->tail] = 0;//清空已读的的数据
	cb->tail = (cb->tail + 1) % BUFFER_SIZE;
	cb->count--;

	return 0;
}
