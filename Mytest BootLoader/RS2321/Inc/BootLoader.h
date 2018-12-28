/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __BOOTLOADER_H__
#define __BOOTLOADER_H__
#include "stm32f3xx_hal.h"

enum ErrorCode
{
	NORMAL,
	CRC_ERROR,
	ADDRESS_ERROR
} ;

extern void Flash_Erase_AppFlash();
extern enum ErrorCode Flash_Program_Oneline(uint32_t startAdd,uint8_t data[],uint8_t startIndex,uint8_t wordLen);
extern uint16_t Flash_Program_ReadFlag();
extern void Flash_Program_WriteFlag(uint16_t data);

extern enum ErrorCode errorcode;

#endif
