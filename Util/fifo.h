/**
  ******************************************************************************
  * File Name          : fifo.h
  * Description        :
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FIFO_H
#define FIFO_H

#include "main.h"

#if defined(__SLC_PRJ__)
	#include "stm32f1xx_hal.h"
#else

#endif

#define FIFO_BUFF_SIZE							1000

typedef struct
{
	uint32_t	phead;
	uint32_t	ptail;
	uint8_t		buffer[FIFO_BUFF_SIZE];
}FIFOTypeDef;

void Fifo_EnQueue(FIFOTypeDef* fifo, uint8_t data);
uint8_t Fifo_DeQueue(FIFOTypeDef* fifo);
uint32_t FifoIsEmpty(FIFOTypeDef* fifo);

#endif /* FIFO_H */
