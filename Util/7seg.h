/*
 * fnd.h
 *
 * Created: 2016-05-03 오후 7:00:18
 *  Author: Jaekwan
 */ 

#include "stm32f1xx_hal.h"
#include "main.h"

#ifndef FND_H_
#define FND_H_

#define SEG_a			0x40
#define SEG_b			0x20
#define SEG_c			0x10
#define SEG_d			0x08
#define SEG_e			0x04
#define SEG_f			0x02
#define SEG_g			0x01

#define SEG_100			0x8000

#define SEG_10_OFF		0x00FF

#define SEG_0			( SEG_g )
#define SEG_1			( SEG_a | SEG_d | SEG_e | SEG_f | SEG_g )
#define SEG_2			( SEG_c | SEG_f )
#define SEG_3			( SEG_e | SEG_f )
#define SEG_4			( SEG_a | SEG_d | SEG_e)
#define SEG_5			( SEG_b | SEG_e )
#define SEG_6			( SEG_b )
#define SEG_7			( SEG_d | SEG_e | SEG_f | SEG_g )
#define SEG_8			( 0x00  )
#define SEG_9			( SEG_e )
#define SEG_A			( SEG_d )
#define SEG_B			( SEG_a | SEG_b )
#define SEG_C			( SEG_a | SEG_b | SEG_c | SEG_f )
#define SEG_D			( SEG_a | SEG_f )
#define SEG_E			( SEG_b | SEG_c )
#define SEG_F			( SEG_b | SEG_c | SEG_d )

/*
#define SEG_0			0x7E
#define SEG_1			0x30
#define SEG_2			0x6D
#define SEG_3			0x79
#define SEG_4			0x33
#define SEG_5			0x5B
#define SEG_6			0x5F
#define SEG_7			0x70
#define SEG_8			0x7F
#define SEG_9			0x7B
#define SEG_A			0x77
#define SEG_B			0x1F
#define SEG_C			0x4E
#define SEG_D			0x3D
#define SEG_E			0x4F
#define SEG_F			0x47
*/
/*
#define SEG_0			( SEG_a | SEG_b | SEG_c | SEG_d | SEG_e | SEG_f )
#define SEG_1			( SEG_b | SEG_c )
#define SEG_2			( SEG_a | SEG_b | SEG_d | SEG_e | SEG_g )
#define SEG_3			( SEG_a | SEG_b | SEG_c | SEG_d | SEG_g )
#define SEG_4			( SEG_b | SEG_c | SEG_f | SEG_g )
#define SEG_5			( SEG_a | SEG_c | SEG_d | SEG_f | SEG_g )
#define SEG_6			( SEG_a | SEG_c | SEG_d | SEG_e | SEG_f | SEG_g )
#define SEG_7			( SEG_a | SEG_b | SEG_c )
#define SEG_8			( SEG_a | SEG_b | SEG_c | SEG_d | SEG_e | SEG_f | SEG_g )
#define SEG_9			( SEG_a | SEG_b | SEG_c | SEG_d | SEG_f | SEG_g )
#define SEG_A			( SEG_a | SEG_b | SEG_c | SEG_e | SEG_f | SEG_g )
#define SEG_B			( SEG_c | SEG_d | SEG_e | SEG_f | SEG_g )
#define SEG_C			( SEG_d | SEG_e | SEG_g )
#define SEG_D			( SEG_b | SEG_c | SEG_d | SEG_e | SEG_g )
#define SEG_E			( SEG_a | SEG_d | SEG_e | SEG_f | SEG_g )
#define SEG_F			( SEG_a | SEG_e | SEG_f | SEG_g )
#define SEG_O			( SEG_c | SEG_d | SEG_e | SEG_g )
#define SEG_R			( SEG_e | SEG_g )
*/

uint16_t int2fnd(int data);

uint16_t ascii2fnd (uint8_t ascii1, uint8_t ascii2);


#endif /* FND_H_ */
