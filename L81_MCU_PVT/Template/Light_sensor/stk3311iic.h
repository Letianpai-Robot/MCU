

#ifndef __STK3311_IIC_H__
#define __STK3311_IIC_H__

#include <stdint.h>
#include <stddef.h>

#include "main.h"
#include "gd32l23x_i2c.h"
#include "gd32l23x_gpio.h"

/**********************************************************/
//PORT DEFINE


#define STK3311_IIC_BUS_NUM         I2C2              
#define STK3311_IIC_PIN_CLOCK       RCU_GPIOC
#define STK3311_IIC_PERIPH_CLOCK    RCU_I2C2

#define STK3311_IIC_SCK_PORT        GPIOC
#define STK3311_IIC_SDA_PORT        GPIOC

#define STK3311_IIC_SCK_AF          GPIO_AF_4
#define STK3311_IIC_SDA_AF          GPIO_AF_4

#define STK3311_IIC_SCK_PIN         GPIO_PIN_0
#define STK3311_IIC_SDA_PIN         GPIO_PIN_1


/**********************************************************/

#define STK3311_IIC_OK              0
#define STK3311_IIC_ERR             1

#define STK3311_IIC_FLAG_ON         1
#define STK3311_IIC_FLAG_OFF        0



#define STK3311_IIC_MAX_BLOCK       0X000000FF

typedef struct
{
  uint8_t check_flag;
  uint16_t light_value;
}T_STK3311_LIGHT_TYPDEF;


extern T_STK3311_LIGHT_TYPDEF t_stk3311_litht;

// extern fun
void stk3311_iic_init(void);

uint8_t STK_WriteReg(uint8_t reg, uint8_t data);
uint8_t STK_ReadReg_1(uint8_t reg, uint8_t *p_dat);
uint8_t STK_ReadReg(uint8_t reg);



//user
void STK_en_light(void);    //en light check
void STK_den_light(void);   //den light check

void STK_init(void);        //use for main init
uint8_t STK_light_read(void); //use for cycle call


#endif


