

#ifndef __QMC6308_IIC_H__
#define __QMC6308_IIC_H__

#include <stdint.h>
#include <stddef.h>

#include "main.h"
#include "gd32l23x_i2c.h"
#include "gd32l23x_gpio.h"

/**********************************************************/
//PORT DEFINE

#define QMC6308_IIC_BUS_NUM         I2C1              
#define QMC6308_IIC_PIN_CLOCK       RCU_GPIOB
#define QMC6308_IIC_PERIPH_CLOCK    RCU_I2C1

#define QMC6308_IIC_SCK_PORT        GPIOB
#define QMC6308_IIC_SDA_PORT        GPIOB

#define QMC6308_IIC_SCK_AF          GPIO_AF_4
#define QMC6308_IIC_SDA_AF          GPIO_AF_4

#define QMC6308_IIC_SCK_PIN         GPIO_PIN_10
#define QMC6308_IIC_SDA_PIN         GPIO_PIN_11

/**********************************************************/



#define QMC6308_IIC_OK              0
#define QMC6308_IIC_ERR             1

#define QMC6308_IIC_FLAG_ON         1
#define QMC6308_IIC_FLAG_OFF        0
#define QMC6308_IIC_MAX_BLOCK       0X000000FF

/**/
#define QMC6308_ID              0X80

#define QMC_REG_ID              0X00  //Chip ID
#define QMC_REG_DATA_XL         0X01  //Data Output X LSB Register XOUT[7:0]
#define QMC_REG_DATA_XH         0X02  //Data Output X MSB Register XOUT[15:8]
#define QMC_REG_DATA_YL         0X03  //Data Output Y LSB Register YOUT[7:0]
#define QMC_REG_DATA_YH         0X04  //Data Output Y MSB Register YOUT[15:8]
#define QMC_REG_DATA_ZL         0X05  //Data Output Z LSB Register ZOUT[7:0]
#define QMC_REG_DATA_ZH         0X06  //Data Output Z MSB Register ZOUT[15:8]
#define QMC_REG_STATUS          0X09  //NVM_ LD, NVM_ RDY, OVFL, DRDY
#define QMC_REG_CTR1            0X0A  //OSR2<1:0>, OSR1<1:0>, ODR<1:0>, MODE<1:0>
#define QMC_REG_CTR2            0X0B  //SOFT_RST, SELF_TEST, RFU, RNG<1:0>, SET/RESET MODE<1:0>
#define QMC_REG_CTR3            0X0D  //SR_CTRL
#define QMC_REG_CTR4            0X29  //SIGNZ, SIGNY, SIGNX

void qmc6308_iic_init(void);
uint8_t QMC_WriteReg(uint8_t reg, uint8_t data);
uint8_t QMC_ReadReg(uint8_t reg, uint8_t *p_dat);
uint8_t QMC_read_id(void);


uint8_t l81_AT_qmcr_func(char params[]); //for at cmd
/**/

#endif


