/**
  ******************************************************************************
  * File Name          : eeprom_24Cxx.c
  * Description        : eeprom code collection
  ******************************************************************************
  */

#include <string.h>
#include <stdlib.h>
#include "eeprom_24Cxx.h"
#include "i2c.h"

//#include "user_define.h"

static HAL_StatusTypeDef AT24C04_08_16_Write(I2C_HandleTypeDef *hi2c, uint8_t *pData)
{
	HAL_StatusTypeDef Status;

#if defined (__AT24C04__) || defined (__AT24C08__) || defined (__AT24C16__)
	uint16_t MemAddress;
	uint16_t Page = 0;
	/** The AT24C04 internally organized with 32 pages of 16 bytes each, the 4K
	* requires an 9-bit data word address for random word addressing.
	* However, AT24C04 can only receive 8-bit data per period, so the 9-bit
	* data word address is actually compromised with the page address bit in
	* device address and the 8-bit memory address.
	* The device address 0xA0 means the first 16 pages while 0xA2 means the
	* last 32 pages.
	* The jumpers will also define the address, so the address 0xA0 or 0xA2
	* are not always right.
	*/
	while(Page < 16)
	{
		MemAddress = Page << 4;
		/** A page write is initiated the same as a byte write, but the
		* microcontroller does not send a stop condition after the first data word
		* is clocked in.
		* In AT24C04, 1 page = 16 bytes.
		* When the word address, internally generated, reaches the page boundary,
		* the following byte is placed at the beginning of the same page.
		*/
		Status = HAL_I2C_Mem_Write(hi2c, ADDR_AT24C_WRITE_1ST_16_PAGES, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, AT24C_PAGE_SIZE, AT24C_TIMEOUT);
		if(Status == HAL_OK)
		{
			Page++;
			pData += AT24C_PAGE_SIZE;
			HAL_Delay(5);
		}
		else
		{
			return Status;
		}
	}

	while(Page >= 16 && Page < 32)
	{
		MemAddress = (Page - 16) << 4;
		Status = HAL_I2C_Mem_Write(hi2c, ADDR_AT24C_WRITE_2ND_16_PAGES, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, AT24C_PAGE_SIZE, AT24C_TIMEOUT);
		if(Status == HAL_OK)
		{
			Page++;
			pData += AT24C_PAGE_SIZE;
			HAL_Delay(5);
		}
		else
		{
			return Status;
		}
	}
#endif

#if defined (__AT24C08__) || defined (__AT24C16__)
	while(Page >= 32 && Page < 48)
	{
		MemAddress = (Page - 32) << 4;
		Status = HAL_I2C_Mem_Write(hi2c, ADDR_AT24C_WRITE_3RD_16_PAGES, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, AT24C_PAGE_SIZE, AT24C_TIMEOUT);
		if(Status == HAL_OK)
		{
			Page++;
			pData += AT24C_PAGE_SIZE;
			HAL_Delay(5);
		}
		else
		{
			return Status;
		}
	}

	while(Page >= 48 && Page < 64)
	{
		MemAddress = (Page - 48) << 4;
		Status = HAL_I2C_Mem_Write(hi2c, ADDR_AT24C_WRITE_4TH_16_PAGES, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, AT24C_PAGE_SIZE, AT24C_TIMEOUT);
		if(Status == HAL_OK)
		{
			Page++;
			pData += AT24C_PAGE_SIZE;
			HAL_Delay(5);
		}
		else
		{
			return Status;
		}
	}
#endif

#if defined (__AT24C16__)
	while(Page >= 64 && Page < 80)
	{
		MemAddress = (Page - 64) << 4;
		Status = HAL_I2C_Mem_Write(hi2c, ADDR_AT24C_WRITE_5TH_16_PAGES, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, AT24C_PAGE_SIZE, AT24C_TIMEOUT);
		if(Status == HAL_OK)
		{
			Page++;
			pData += AT24C_PAGE_SIZE;
			HAL_Delay(5);
		}
		else
		{
			return Status;
		}
	}

	while(Page >= 80 && Page < 96)
	{
		MemAddress = (Page - 80) << 4;
		Status = HAL_I2C_Mem_Write(hi2c, ADDR_AT24C_WRITE_6TH_16_PAGES, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, AT24C_PAGE_SIZE, AT24C_TIMEOUT);
		if(Status == HAL_OK)
		{
			Page++;
			pData += AT24C_PAGE_SIZE;
			HAL_Delay(5);
		}
		else
		{
			return Status;
		}
	}
	while(Page >= 96 && Page < 112)
	{
		MemAddress = (Page - 96) << 4;
		Status = HAL_I2C_Mem_Write(hi2c, ADDR_AT24C_WRITE_7TH_16_PAGES, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, AT24C_PAGE_SIZE, AT24C_TIMEOUT);
		if(Status == HAL_OK)
		{
			Page++;
			pData += AT24C_PAGE_SIZE;
			HAL_Delay(5);
		}
		else
		{
			return Status;
		}
	}

	while(Page >= 112 && Page < 128)
	{
		MemAddress = (Page - 112) << 4;
		Status = HAL_I2C_Mem_Write(hi2c, ADDR_AT24C_WRITE_8TH_16_PAGES, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, AT24C_PAGE_SIZE, AT24C_TIMEOUT);
		if(Status == HAL_OK)
		{
			Page++;
			pData += AT24C_PAGE_SIZE;
			HAL_Delay(5);
		}
		else
		{
			return Status;
		}
	}
#endif
	return Status;
}

HAL_StatusTypeDef AT24C_Write(I2C_HandleTypeDef *hi2c, uint8_t *pData)
{
#if defined (__AT24C01__) || defined (__AT24C02__)
	return HAL_I2C_Mem_Write(hi2c, ADDR_AT24C_WRITE, 0x0000, I2C_MEMADD_SIZE_8BIT, pData, BUFFER_SIZE, AT24C_TIMEOUT);
#elif defined (__AT24C04__) || defined (__AT24C08__) || defined (__AT24C16__)
	return AT24C04_08_16_Write(hi2c, pData);
#elif defined (__AT24C128__) || defined (__AT24C256__)
	return HAL_I2C_Mem_Write(hi2c, ADDR_AT24C_WRITE, 0x0000, I2C_MEMADD_SIZE_16BIT, pData, BUFFER_SIZE, AT24C_TIMEOUT);
#endif
}

/**
	* @brief  Read all of the data from the AT24C04 EEPROM.
	* @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
	*         the configuration information for the specified I2C.
	* @param  pData Pointer to data buffer
	* @retval HAL status
	*/
HAL_StatusTypeDef AT24C_Read(I2C_HandleTypeDef *hi2c, uint8_t *pData)
{
	uint16_t MemAddress = 0x00;
#if defined(__AT24C01__) || defined(__AT24C02__) || defined(__AT24C04__) || defined(__AT24C08__) || defined(__AT24C16__)
	return HAL_I2C_Mem_Read(hi2c, ADDR_AT24C_READ, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, BUFFER_SIZE, AT24C_TIMEOUT);
#elif defined (__AT24C128__) || defined (__AT24C256__)
	return HAL_I2C_Mem_Read(hi2c, ADDR_AT24C_READ, MemAddress, I2C_MEMADD_SIZE_16BIT, pData, BUFFER_SIZE, AT24C_TIMEOUT);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AT24C02: 256byte
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int at24_HAL_WriteBytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t TxBufferSize)
{
	if(MemAddress + TxBufferSize > 16)
	{
		while(HAL_I2C_Mem_Write(hi2c,(uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)(16-MemAddress),1000) != HAL_OK);
		*pData = *pData + (16 - MemAddress);
		while(HAL_I2C_Mem_Write(hi2c,(uint16_t)DevAddress,(uint16_t)16,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)(MemAddress+TxBufferSize-16),1000) != HAL_OK);
	}
	else
	{
		while((TxBufferSize - 16) > 0)
		{
			while(HAL_I2C_Mem_Write(hi2c,(uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)16,1000) != HAL_OK);
			TxBufferSize -= 16;
			pData += 16;
			MemAddress += 16;
		}
		while(HAL_I2C_Mem_Write(hi2c,(uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)TxBufferSize,1000) != HAL_OK);
	}
	return 1;
}

int at24_HAL_ReadBytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t RxBufferSize)
{
	int TimeOut;

	while((RxBufferSize - 16) > 0)
	{
		//if your data is more than 16 bytes,you are here
		TimeOut = 0;
		while(HAL_I2C_Mem_Read(hi2c,(uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)16,1000)!= HAL_OK && TimeOut < 10)
		{
			TimeOut++;
		}

		RxBufferSize -= 16;
		pData += 16;
		MemAddress += 16;
	}

	//remaining data
	TimeOut = 0;
	while(HAL_I2C_Mem_Read(hi2c,(uint16_t)DevAddress,(uint16_t)MemAddress,I2C_MEMADD_SIZE_8BIT,pData,(uint16_t)RxBufferSize,1000)!= HAL_OK && TimeOut < 10)
	{
		TimeOut++;
	}

	return 1;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ErrorStatus EEPROM24XX_IsConnected(void)
{
	#if (EEPROM_USE_WP_PIN == 1)
	HAL_GPIO_WritePin(_EEPROM_WP_GPIO,_EEPROM_WP_PIN,GPIO_PIN_SET);
	#endif
	if(HAL_I2C_IsDeviceReady(&EEPROM24XX_I2C, 0xa0, 1, 1000)==HAL_OK)
		return SUCCESS;
	else
		return ERROR;
}

ErrorStatus EEPROM24XX_Erase(void)
{
	uint8_t buff[EEPROM_PAGE_SIZE];
	uint8_t page_cnt = EEPROM_BLOCK_SIZE / EEPROM_PAGE_SIZE;
	uint8_t address;
	memset(buff, 0xFF, EEPROM_PAGE_SIZE);

	for(uint8_t i = 0; i < page_cnt; i++)
	{
		address = i * EEPROM_PAGE_SIZE;
		if(EEPROM24XX_Save(address, buff, EEPROM_PAGE_SIZE) == ERROR) return ERROR;
	}

	return SUCCESS;
}

ErrorStatus EEPROM24XX_Save(uint16_t Address, void *data, size_t size_of_data)
{
	#if ((EEPROM_SIZE_KBIT==1) || (EEPROM_SIZE_KBIT==2))
	if(size_of_data > 8)
		return ERROR;
	#endif
	#if ((EEPROM_SIZE_KBIT==4) || (EEPROM_SIZE_KBIT==8) || (EEPROM_SIZE_KBIT==16))
	if(size_of_data > 16)
		return ERROR;
	#endif
	#if ((EEPROM_SIZE_KBIT==32) || (EEPROM_SIZE_KBIT==64) || (EEPROM_SIZE_KBIT==128))
	if(size_of_data > 32)
		return ERROR;
	#endif
	
	#if (EEPROM_USE_WP_PIN == 1)
	HAL_GPIO_WritePin(_EEPROM_WP_GPIO,_EEPROM_WP_PIN,GPIO_PIN_RESET);
	#endif
	
	#if ((EEPROM_SIZE_KBIT == 1) || (EEPROM_SIZE_KBIT == 2))
	if(HAL_I2C_Mem_Write(&EEPROM24XX_I2C, 0xa0, Address, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, size_of_data, 100) == HAL_OK)
	#else
	if(HAL_I2C_Mem_Write(&EEPROM24XX_I2C, 0xa0, Address, I2C_MEMADD_SIZE_16BIT, (uint8_t*)data, size_of_data, 100) == HAL_OK)
	#endif
	{
		HAL_Delay(7);
		#if (EEPROM_USE_WP_PIN == 1)
		HAL_GPIO_WritePin(_EEPROM_WP_GPIO,_EEPROM_WP_PIN,GPIO_PIN_SET);
		#endif
		return SUCCESS;
	}
	else
	{
		#if (EEPROM_USE_WP_PIN == 1)
		HAL_GPIO_WritePin(_EEPROM_WP_GPIO,_EEPROM_WP_PIN,GPIO_PIN_SET);
		#endif
		return ERROR;
	}
}

ErrorStatus EEPROM24XX_Load(uint16_t Address, void *data, size_t size_of_data)
{
	#if (EEPROM_USE_WP_PIN == 1)
	HAL_GPIO_WritePin(_EEPROM_WP_GPIO,_EEPROM_WP_PIN,GPIO_PIN_SET);
	#endif
	#if ((EEPROM_SIZE_KBIT == 1) || (EEPROM_SIZE_KBIT == 2))
	if(HAL_I2C_Mem_Read(&EEPROM24XX_I2C, 0xa0, Address, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, size_of_data, 1000) == HAL_OK)
	#else
	if(HAL_I2C_Mem_Read(&EEPROM24XX_I2C, 0xa0, Address, I2C_MEMADD_SIZE_16BIT, (uint8_t*)data, size_of_data, 1000) == HAL_OK)
	#endif
	{
		return SUCCESS;
	}
	else
		return ERROR;
}

#if defined(__SLC_PRJ__)

int read_eeprom(uint8_t debug, uint16_t addr, void* data, uint16_t size)
{
	uint8_t RxBuff[EEPROM_BLOCK_SIZE];
	memset(RxBuff, '\0', EEPROM_BLOCK_SIZE);

	if(size > EEPROM_BLOCK_SIZE)
		size = EEPROM_BLOCK_SIZE;

//	if(EEPROM24XX_Load(addr, RxBuff, size) != SUCCESS)
	if(AT24C_Read(&hi2c1, RxBuff) != HAL_OK)
	{
		printf("\r\n failed to load data from eeprom!!");
		return 0;
	}

	memcpy(data, &RxBuff[addr], size);

	if(debug != 0)
	{
		printf("\r\n\nRead from eeprom:");
		uint8_t j = 0;
		printf("\r\naddr = %03d, size = %03d", addr, size);
		for(uint16_t i = 0; i < size; i++)
		{
			if(i % EEPROM_PAGE_SIZE == 0) 
			{
				printf("\r\n[%02d]",j++);
			}
			else if(i % 4 == 0)
			{
				printf(" |");
			}
			if(debug == 1) printf(" %3d", RxBuff[addr + i]);
			else printf(" %02x", RxBuff[addr + i]);
		}
		printf("\r\n");
	}

//	HAL_Delay(50);

	return 1;
}

int write_eeprom(uint8_t debug, uint16_t addr, void *data, uint16_t size)
{
	uint8_t buff[EEPROM_BLOCK_SIZE];
	uint8_t page_number = 0;
	uint16_t dev_addr = 0; 
	uint16_t mem_addr = 0;

	if(size > EEPROM_BLOCK_SIZE)
		return 0;
	else if(size > EEPROM_PAGE_SIZE)
	{
		uint8_t final_page_number;
		uint8_t rest_bytes;
		uint16_t start_page_number, start_mem_addr;

		dev_addr = 0xA0;

		if(addr > 0)
		{
			start_mem_addr = addr % EEPROM_PAGE_SIZE;
			if(HAL_I2C_Mem_Write(&hi2c1, dev_addr, addr, I2C_MEMADD_SIZE_8BIT, data, EEPROM_PAGE_SIZE - start_mem_addr , 1000) != HAL_OK)
			{
				printf("\r\n failed to write data to the eeprom!!");
				return 0;
			}
			HAL_Delay(5);

			start_page_number = (addr + EEPROM_PAGE_SIZE - start_mem_addr) / EEPROM_PAGE_SIZE;
			final_page_number = (size - (EEPROM_PAGE_SIZE - start_mem_addr)) / EEPROM_PAGE_SIZE + start_page_number; 
			rest_bytes = (size - (EEPROM_PAGE_SIZE - start_mem_addr)) % EEPROM_PAGE_SIZE;
		}
		else
		{
			final_page_number = size / EEPROM_PAGE_SIZE;
			rest_bytes = size % EEPROM_PAGE_SIZE;
			mem_addr = 0;
			start_page_number = 0;
			start_mem_addr = 0;
		}

		memcpy(buff, data, size);

		for(; start_page_number < final_page_number; start_page_number++)
		{
			start_mem_addr = EEPROM_PAGE_SIZE * start_page_number;
			mem_addr = start_mem_addr;
			if(mem_addr >= 256)
			{
				mem_addr -= 256;
				dev_addr = 0xA2;
			}

			if(HAL_I2C_Mem_Write(&hi2c1, dev_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &buff[start_mem_addr], EEPROM_PAGE_SIZE, 1000) != HAL_OK)
			{
				printf("\r\n failed to write data to the eeprom");
				return 0;
			}
			HAL_Delay(5);
		}
		
		if(rest_bytes != 0)
		{
			mem_addr += EEPROM_PAGE_SIZE;
			start_mem_addr = EEPROM_PAGE_SIZE * final_page_number;
			if(HAL_I2C_Mem_Write(&hi2c1, dev_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, &buff[start_mem_addr], rest_bytes, 1000) != HAL_OK)
			{
				printf("\r\n failed to write data to the eeprom");
				return 0;
			}
			HAL_Delay(5);
		}
	}
	else
	{
		page_number = addr / EEPROM_PAGE_SIZE;

		if(page_number < 16) 
		{
			dev_addr = 0xA0; 
			mem_addr = addr;
		}
		else if(page_number >=  16 && page_number <  32) // 24C04
		{
			dev_addr = 0xA2; 
			mem_addr = addr - 256; 
		}
		
		if(HAL_I2C_Mem_Write(&hi2c1, dev_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, data, size, 1000) != HAL_OK)
		{
			printf("\r\n failed to write data to the eeprom");
			return 0;
		}
	}

	if(debug)
	{
		printf("\r\nWrite to eeprom:");
		printf("\r\npage[%02d] addr[%3d] dev_addr[%02x] mem_addr[%3d] data: ", page_number, addr, dev_addr, mem_addr);
		HAL_Delay(50);
		if(HAL_I2C_Mem_Read(&hi2c1, dev_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, buff, size, 1000) != HAL_OK)
		{
			printf("\r\n failed to load data from eeprom!!");
		}

		for(uint16_t i = 0; i < size; i++) printf("%02x ", buff[i]);
		printf("\r\n");
	}

	HAL_Delay(50);

	return 1;
}

int erase_eeprom(void)
{
	uint8_t tmp[EEPROM_BLOCK_SIZE];
	memset(tmp, 0xFF, EEPROM_BLOCK_SIZE);
	//if(AT24C_Write(&hi2c1, tmp) != HAL_OK) 
	if(!write_eeprom(0, 0, tmp, EEPROM_BLOCK_SIZE))
		return 0;

	return 1;
}

#endif

