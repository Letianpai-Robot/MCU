/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VI530x_FIRMWARE_H
#define __VI530x_FIRMWARE_H			 

/* Includes ------------------------------------------------------------------*/
#include "VI530x_User_Handle.h"


extern const uint8_t VI5300_M31_firmware_buff[8192];

uint8_t Get_VI530x_Download_Firmware_Status(void);
uint8_t VI530x_Write_Firmware_PreConfig(void);
uint8_t VI530x_Write_Firmware_Post_Config(void);
uint8_t VI530x_Download_Firmware(uint8_t *Firmware_buff, uint16_t size);
uint16_t FirmwareSize(void);

#endif 

