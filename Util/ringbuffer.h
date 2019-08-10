/*
 * ringbuffer.h
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include "stm32f1xx_hal.h"
#include "main.h"

__STATIC_INLINE void Inc_pValue(uint32_t *pdata);
void Fifo_EnQueue(uint8_t data);
uint8_t Fifo_DeQueue(void);
uint32_t FifoIsEmpty(void);

#endif /* RINGBUFFER_H_ */

