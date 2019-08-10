/**
  ******************************************************************************
  * File Name          : eeprom_24Cxx.h
  * Description        : eeprom code collection
  ******************************************************************************
  */

#ifndef __EEPROM_24CXX_H
#define __EEPROM_24CXX_H

#include "main.h"

#if defined(__SLC_PRJ__) || defined(__TCP_MODEM_PRJ__)
	#include "stm32f1xx_hal.h"
#else

#endif

#if defined(__TCP_MODEM_PRJ__)
	#define __AT24C02__
#elif defined(__SLC_PRJ__)
	#define __AT24C04__
#endif
//#define __AT24C128__

#if defined (__AT24C01__) || defined (__AT24C02__)
	#define AT24C_PAGE_SIZE								8		//byte
	#define ADDR_AT24C_WRITE_8_PAGES					0xA0
#elif defined (__AT24C04__) || defined (__AT24C08__) || defined (__AT24C16__)
	#define AT24C_PAGE_SIZE								16
		#define ADDR_AT24C_WRITE_1ST_16_PAGES			0xA0
		#define ADDR_AT24C_WRITE_2ND_16_PAGES			0xA2
	#if defined (__AT24C08__) || defined (__AT24C16__)
		#define ADDR_AT24C_WRITE_3RD_16_PAGES			0xA4
		#define ADDR_AT24C_WRITE_4TH_16_PAGES			0xA6
	#endif
	#if defined (__AT24C16__)
		#define ADDR_AT24C_WRITE_5TH_16_PAGES			0xA8
		#define ADDR_AT24C_WRITE_6TH_16_PAGES			0xAA
		#define ADDR_AT24C_WRITE_7TH_16_PAGES			0xAC
		#define ADDR_AT24C_WRITE_8TH_16_PAGES			0xAE
	#endif
#elif defined (__AT24C128__) || defined (__AT24C256__) 
	#define AT24C_PAGE_SIZE								64
	#define ADDR_AT24C_WRITE_64_PAGES					0xA0
#endif

#if defined (__AT24C01__) || defined (__AT24C02__)
	#define ADDR_AT24C_WRITE							ADDR_AT24C_WRITE_8_PAGES
#elif defined (__AT24C04__) || defined (__AT24C08__) || defined (__AT24C16__)
	#define ADDR_AT24C_WRITE							ADDR_AT24C_WRITE_1ST_16_PAGES
#elif defined (__AT24C128__) || defined (__AT24C256__) 
	#define ADDR_AT24C_WRITE							ADDR_AT24C_WRITE_64_PAGES
#endif

#define ADDR_AT24C_READ									0xA1
#define AT24C_TIMEOUT									0xFF
#define BUFFER_SIZE										512


HAL_StatusTypeDef AT24C_Read(I2C_HandleTypeDef *hi2c, uint8_t *pData);
HAL_StatusTypeDef AT24C_Write(I2C_HandleTypeDef *hi2c, uint8_t *pData);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AT24C02: 256byte
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int at24_HAL_WriteBytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t TxBufferSize);
int at24_HAL_ReadBytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t RxBufferSize);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined(__TCP_MODEM_PRJ__)
	#define EEPROM_SIZE_KBIT				2		//AT24C02, 2K
	#define EEPROM24XX_I2C					hi2c1
	#define EEPROM_USE_WP_PIN				0
	
	#define EEPROM_BLOCK_SIZE				256
	#define EEPROM_PAGE_SIZE				8
	#define EEPROM_BLOCK_COUNT				1

#elif defined(__SLC_PRJ__)
	#define EEPROM_SIZE_KBIT				4		//AT24C04, 4K
	#define EEPROM24XX_I2C					hi2c1
	#define EEPROM_USE_WP_PIN				0

	#define EEPROM_BLOCK_SIZE				512		// 256(24C02)
	#define EEPROM_PAGE_SIZE				16		// 8(24C02)
	#define EEPROM_BLOCK_COUNT				2		// 1(24AA02), 2(24AA04), 4(24AA08), 8(24AA16)
#endif

ErrorStatus EEPROM24XX_IsConnected(void);
ErrorStatus EEPROM24XX_Erase(void);
ErrorStatus EEPROM24XX_Save(uint16_t Address,void *data,size_t size_of_data);
ErrorStatus EEPROM24XX_Load(uint16_t Address,void *data,size_t size_of_data);

int read_eeprom(uint8_t debug, uint16_t addr, void* data, uint16_t size);
int write_eeprom(uint8_t debug, uint16_t addr, void *data, uint16_t size);
int erase_eeprom(void);

typedef struct
{
	uint8_t buff[BUFFER_SIZE];
	uint16_t addr;
	uint16_t size;
	uint8_t page;
	uint8_t count;
} __attribute__((packed)) EEPROM_DataTypedef;

#endif /*__EEPROM_24CXX_H*/

