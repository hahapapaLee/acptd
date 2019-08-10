/*
 * 7seg.c
 *
 * Created: 2016-09-26 오후 4:03:31
 *  Author: Jaekwan
 */ 

#include "7seg.h"

const uint8_t seg[] = {SEG_0, SEG_1, SEG_2, SEG_3, SEG_4, SEG_5, SEG_6, SEG_7, SEG_8, SEG_9, 0x00};
const uint8_t seg2[] = {SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F};

/**
 * \brief	Function to write to the 7-Segment
 *
 * \param	integer data for 7-Segment Display
 * \returns	convert integer to 7-Segment data
 */
uint16_t int2fnd(int data)
{
	int high,low;
	uint16_t fnd_data;
	
	if(data < 100) high = data / 10;
	else high = (data - 100) / 10;
	
	low = data % 10;

	fnd_data = (seg[high]<<8) | seg[low];
	
	if(data < 10) fnd_data &= SEG_10_OFF;			// 0 ~ 9인 경우 십자리의 수는 표시하지 않음.

	if(data > 99) fnd_data |= SEG_100;
	 
	return fnd_data;
}

/**
 * \brief	Function to write to the 7-Segment
 *
 * \param	ascii1, ascii2	ASCII Input
 * \returns	convert input data to 7-Segment data
 */
uint16_t ascii2fnd (uint8_t ascii1, uint8_t ascii2)
{
	uint8_t high = 0;
	uint8_t low = 0;
	uint16_t fnd = 0;
	
	// high digit
	if(ascii1 >= 'A' && ascii1 <= 'F')			// A~F
	{
		high = seg2[ascii1 - 0x41];
	}
	else if(ascii1 >= '0' && ascii1 <= '9')		// 0~9
	{
		high = seg[ascii1 - 0x30];	
	}
	
	// low digit
	if(ascii2 >= 'A' && ascii2 <= 'F')			// A~F
	{
		low = seg2[ascii2 - 0x41];
	}
	else if(ascii2 >= '0' && ascii2 <= '9')		// 0~9
	{
		low = seg[ascii2 - 0x30];
	}
	
	fnd = (high << 8) | low;	
	
	return(fnd);
}
