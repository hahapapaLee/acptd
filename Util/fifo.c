/**
  ******************************************************************************
  * File Name          : fifo.c
  * Description        : FIFO functions
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fifo.h"

//FIFOTypeDef fifo;

static void Inc_pValue(uint32_t *pdata)
{
	(*pdata)++;
	if((*pdata) == FIFO_BUFF_SIZE)
	{
		(*pdata) = 0;
	}
}

void Fifo_EnQueue(FIFOTypeDef* fifo, uint8_t data)
{
	fifo->buffer[fifo->phead] = data;
	Inc_pValue(&fifo->phead);
}

uint8_t Fifo_DeQueue(FIFOTypeDef* fifo)
{
	uint8_t retVal = fifo->buffer[fifo->ptail];
	Inc_pValue(&fifo->ptail);
	return retVal;
}

uint32_t FifoIsEmpty(FIFOTypeDef* fifo)
{
	return (fifo->phead == fifo->ptail)? 1 : 0;
}
