

#ifndef __QMI8658_IIC_H__
#define __QMI8658_IIC_H__

#include <stdint.h>
#include <stddef.h>

#include "main.h"
#include "gd32l23x_i2c.h"
#include "gd32l23x_gpio.h"

/**********************************************************/
//PORT DEFINE


#define QMI_IIC_BUS_NUM         I2C1              
#define QMI_IIC_PIN_CLOCK       RCU_GPIOB
#define QMI_IIC_PERIPH_CLOCK    RCU_I2C1

#define QMI_IIC_SCK_PORT        GPIOB
#define QMI_IIC_SDA_PORT        GPIOB

#define QMI_IIC_SCK_AF          GPIO_AF_4
#define QMI_IIC_SDA_AF          GPIO_AF_4

#define QMI_IIC_SCK_PIN         GPIO_PIN_10
#define QMI_IIC_SDA_PIN         GPIO_PIN_11


/**********************************************************/

#define QMI_IIC_OK              0
#define QMI_IIC_ERR             1

#define QMI_IIC_FLAG_ON         1
#define QMI_IIC_FLAG_OFF        0



#define QMI_IIC_MAX_BLOCK       0X000000FF


// extern fun
void QMI_iic_init(void);

uint8_t QMI_WriteReg(uint8_t reg, uint8_t data);
uint8_t QMI_ReadReg_1(uint8_t reg, uint8_t *p_dat);
uint8_t QMI_ReadReg(uint8_t reg);



//user
uint8_t QMI_id_read(void);


#endif


