

#ifndef __GXHTC3_IIC_H__
#define __GXHTC3_IIC_H__

#include <stdint.h>
#include <stddef.h>

#include "main.h"
#include "gd32l23x_i2c.h"
#include "gd32l23x_gpio.h"

/**********************************************************/
//PORT DEFINE
#define GXHTC3_USE_HARD_I2C_EN     0
#if GXHTC3_USE_HARD_I2C_EN
#define GXHTC3_IIC_BUS_NUM         I2C1              
#define GXHTC3_IIC_PIN_CLOCK       RCU_GPIOB
#define GXHTC3_IIC_PERIPH_CLOCK    RCU_I2C1

#define GXHTC3_IIC_SCK_PORT        GPIOB
#define GXHTC3_IIC_SDA_PORT        GPIOB

#define GXHTC3_IIC_SCK_AF          GPIO_AF_4
#define GXHTC3_IIC_SDA_AF          GPIO_AF_4

#define GXHTC3_IIC_SCK_PIN         GPIO_PIN_10
#define GXHTC3_IIC_SDA_PIN         GPIO_PIN_11
#else


#endif

/**********************************************************/


#define GXHTC3_IIC_OK              0
#define GXHTC3_IIC_ERR             1

#define GXHTC3_IIC_FLAG_ON         1
#define GXHTC3_IIC_FLAG_OFF        0
#define GXHTC3_IIC_MAX_BLOCK       0X000000FF


/*GXHTC3 START*/
#define GXHTC3_ID              0x20F0

//cmd 2BYTE send (fist->secend) H->L, read back 3Byte = (data 2B)+ (crc8 1B)
#define GXHTC3_ADDR7           0x70
#define GXHTC3_SLEEP_CMD       0xB098
#define GXHTC3_WAKEUP_CMD      0x3517


#define GXHTC3_NORMAL_T_CMD    0x7866  //clock stretching close; normal mode 100kHz
#define GXHTC3_NORMAL_H_CMD    0x58E0
#define GXHTC3_FAST_T_CMD      0x609C  //clock stretching close;fast mode 1000kHz
#define GXHTC3_FAST_H_CMD      0x401A


#define GXHTC3_ID_CMD          0xEFC8   //
#define GXHTC3_S_RESET_CMD     0x805D   //


void gxhtc3_iic_init(void);
uint8_t gxhtc3_write_cmd(uint16_t data);
uint8_t gxhtc3_read_data_3B(uint8_t *p_dat);
uint8_t gxhtc3_cmd_read_3B(uint16_t cmd, uint8_t *p_dat);

uint16_t gxhtc3_read_id(void);
//uint16_t gxhtc3_read_tem(void);
//uint16_t gxhtc3_read_hum(void);

uint8_t l81_AT_gxhtcr_func(char params[]);    //at cmd fun
/*GXHTC3 END*/


#endif


