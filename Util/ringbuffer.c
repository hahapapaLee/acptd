/*
 * ringbuffer.c
 *
 *  Created on: 2016. 12. 1.
 *      Author: Jaekwan
 */

#include <ringbuffer.h>

#define BUFFER_SIZE			100

typedef struct 
{
	uint32_t 	phead;
	uint32_t 	ptail;
	uint8_t 	buffer[BUFFER_SIZE];
}FIFOTypeDef;

FIFOTypeDef		fifo;

__STATIC_INLINE void Inc_pValue(uint32_t *pdata)
{
	(*pdata)++;
	if((*pdata) == BUFFER_SIZE)
	{
		(*pdata) = 0;
	}
}

void Fifo_EnQueue(uint8_t data)
{
	fifo.buffer[fifo.phead] = data;
	Inc_pValue(&fifo.phead);
}

uint8_t Fifo_DeQueue(void)
{
	uint8_t retVal = fifo.buffer[fifo.ptail];
	Inc_pValue(&fifo.ptail);
	return retVal;
}

uint32_t FifoIsEmpty(void)
{
	return (fifo.phead == fifo.ptail)? 1 : 0;
}



