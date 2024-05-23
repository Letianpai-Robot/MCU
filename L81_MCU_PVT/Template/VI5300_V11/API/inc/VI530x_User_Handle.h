/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VI530x_USER_HANDLE_H
#define __VI530x_USER_HANDLE_H

typedef enum 
{
    VI530x_OK       = 0x00,
    VI530x_RANGING  = 0x01,
    VI530x_BUSY     = 0x02,
    VI530x_BUS_BUSY = 0x03,
    VI530x_SLEEP    = 0x04,
    VI530x_BOOTING  = 0x05,
    VI530x_ERROR    = 0x06,
		VI530x_IIC_ERROR	= 0x07
} VI530x_Status;
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdio.h"
#include "VI530x_API.h"
#include "VI530x_Firmware.h"
#include "VI530x_System_Data.h"
#include "VI530x_Algorithm.h"
/* Start user code for adding. */

/* End user code.  */

//i2c????
uint8_t VI530x_IIC_Read_One_Byte(uint8_t addr, uint8_t *value);
uint8_t VI530x_IIC_Read_X_Bytes(uint8_t addr, uint8_t *value, uint16_t tlen);
uint8_t VI530x_IIC_Write_One_Byte(uint8_t addr, uint8_t value);
uint8_t VI530x_IIC_Write_X_Bytes(uint8_t addr, uint8_t *pValue, uint16_t tlen);
uint8_t l81_VI53_dem_func(char params[]);
uint8_t l81_VI53_cal_read_raw_func(char params[]);
uint8_t l81_VI53_cal_func(char params[]);
uint8_t l81_VI53_TofSet_func(char params[]);
/**
 * @brief ??????
 * @return [type]
 */
void VI530x_GPIO_Interrupt_Handle(void);
/**
 * @brief XSHUT??
 * @param [uint8_t] state   1--enable, 0--disable
 * @return [type]
 */
void VI530x_XSHUT_Enable(uint8_t state);



void VI530x_Delay_Ms(uint16_t nMs);

//void VI530x_main(void);
void VI530x_init();
int16_t distance_measurement_result();   //0620-rjq  void -> int16_t
#endif

