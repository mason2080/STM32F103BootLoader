#include "BootLoader.h"

#define FLASH_USER_START_ADDR ((uint32_t)0x08004000)
#define FLASH_USER_END_ADDR ((uint32_t)0x0800FB00)

enum ErrorCode errorcode=NORMAL;

  /*
  * 
  * Erase APP_ROM
  *
  ******************************************************************************
  */
void Flash_Erase_AppFlash()
{
  //uint32_t FLASH_USER_START_ADDR =  ((uint32_t)0x08004000);
	uint32_t pageError=0;
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase=FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress=FLASH_USER_START_ADDR;
	EraseInitStruct.NbPages=11;

	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&EraseInitStruct,&pageError);
	HAL_FLASH_Lock();

}  

  /*
  * 
  * Program One line to  Flash
  *
  ******************************************************************************
  */
void Flash_Program_Oneline_Word(uint32_t startAdd,uint8_t data[],uint8_t startIndex,uint8_t wordLen)
{
	uint32_t address=startAdd;
	uint32_t tempWrite=0;
	HAL_FLASH_Unlock();
	for(int i=0;i<wordLen/4;i++)
	{
		tempWrite=(data[startIndex+i*4+3]<<24)+(data[startIndex+i*4+2]<<16)+(data[startIndex+i*4+1]<<8)+data[startIndex+i*4];
		
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,tempWrite);
		
		address+=4;
	}
	HAL_FLASH_Lock();
}  

  /*
  * 
  * Program One line to  Flash
  *
  ******************************************************************************
  */
enum ErrorCode Flash_Program_Oneline(uint32_t startAdd,uint8_t data[],uint8_t startIndex,uint8_t wordLen)
{
	uint32_t address=startAdd;
	uint32_t tempWrite=0;
	if((startAdd>=FLASH_USER_START_ADDR)&&(startAdd<=FLASH_USER_END_ADDR))
	{
		HAL_FLASH_Unlock();
		for(int i=0;i<wordLen/2;i++)
		{
			tempWrite=(data[startIndex+i*2+1]<<8)+data[startIndex+i*2];
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,address,tempWrite);
			address+=2;
		}
		HAL_FLASH_Lock();
		
		//errorcode=NORMAL;
		return NORMAL;
  }
	else
	{
		//errorcode=ADDRESS_ERROR;
		return ADDRESS_ERROR;
	}
}  


  /*
  * 
  * Program One line to  Flash
  *
  ******************************************************************************
  */
void Flash_Program_WriteFlag(uint16_t data)
{
 // uint32_t FLASH_USER_START_ADDR =  ((uint32_t)0x0800FC00);
	uint32_t pageError=0;
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase=FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress=((uint32_t)0x0800FC00);
	EraseInitStruct.NbPages=1;

	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&EraseInitStruct,&pageError);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,0x0800FFFC,data);
	HAL_FLASH_Lock();
}  

/**
  * @brief  Read half words (16-bit data) of the specified address
  * @note   This function can be used for all STM32F10x devices.
  * @param  faddr: The address to be read (the multiple of the address, which is 2)
  * @retval Value of specified address
  */
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr)
{
	return *(uint16_t*)faddr; 
}

  /*
  * 
  * Program One line to  Flash
  *
  ******************************************************************************
  */
uint16_t Flash_Program_ReadFlag()
{
	return STMFLASH_ReadHalfWord(0x0800FFFC);
}  






