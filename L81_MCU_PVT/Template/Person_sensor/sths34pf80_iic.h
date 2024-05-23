

#ifndef __STHS34PF80_IIC_H__
#define __STHS34PF80_IIC_H__

#include <stdint.h>
#include <stddef.h>

#include "main.h"
#include "gd32l23x_i2c.h"
#include "gd32l23x_gpio.h"

/**********************************************************/
//PORT DEFINE


#define STHS34_IIC_BUS_NUM         I2C2              
#define STHS34_IIC_PIN_CLOCK       RCU_GPIOC
#define STHS34_IIC_PERIPH_CLOCK    RCU_I2C2

#define STHS34_IIC_SCK_PORT        GPIOC
#define STHS34_IIC_SDA_PORT        GPIOC

#define STHS34_IIC_SCK_AF          GPIO_AF_4
#define STHS34_IIC_SDA_AF          GPIO_AF_4

#define STHS34_IIC_SCK_PIN         GPIO_PIN_0
#define STHS34_IIC_SDA_PIN         GPIO_PIN_1


/**********************************************************/

#define STHS34_IIC_OK              0
#define STHS34_IIC_ERR             1

#define STHS34_IIC_FLAG_ON         1
#define STHS34_IIC_FLAG_OFF        0


#define STHS34_IIC_MAX_BLOCK       0X000000FF




/**/
#define STHS34_REG_RES1             0X00    //00-0Bh Reserved
#define STHS34_REG_LPF1             0X0C    //04h
#define STHS34_REG_LPF2             0X0D    //22h
#define STHS34_REG_RES2             0X0E    //Reserved
#define STHS34_REG_ID               0X0F    //D3h Who am I
#define STHS34_REG_AVG_TRIM         0X10    //03h
#define STHS34_REG_Res3             0X11    //0X11-1Fh Reserved
#define STHS34_REG_CTRL1            0X20
#define STHS34_REG_CTRL2            0X21
#define STHS34_REG_CTRL3            0X22    //Interrupt Control
#define STHS34_REG_STATUS           0X23
#define STHS34_REG_FUNC_STATUS      0X25
#define STHS34_REG_TOBJECT_L        0X26
#define STHS34_REG_TOBJECT_H        0X27
#define STHS34_REG_TAMBIENT_L       0X28
#define STHS34_REG_TAMBIENT_H       0X29
#define STHS34_REG_TPRESENCE_L      0X3A
#define STHS34_REG_TPRESENCE_H      0X3B
#define STHS34_REG_TMOTION_L        0X3C
#define STHS34_REG_TMOTION_H        0X3D
#define STHS34_REG_TAMB_SHOCK_L     0X3E
#define STHS34_REG_TAMB_SHOCK_H     0X3F

#define STHS34_REG_PRESENCE_THS     0X20    //C8h
#define STHS34_REG_MOTION_THS       0X22    //C8h
#define STHS34_REG_TAMB_SHOCK_THS   0X24    //0Ah
#define STHS34_REG_HYST_MOTION      0X26    //32h
#define STHS34_REG_HYST_PRESENCE    0X27    //32h
#define STHS34_REG_ALGO_CONFIG      0X28    //00h
#define STHS34_REG_HYST_TAMB_SHOCK  0X29    //02h
/**/








typedef struct
{
  uint8_t  read_flag;
  int16_t tobject;
  int16_t tambient;
  int16_t tperson;
  int16_t tmotion;
}T_STH34_PERSON_DATA_TYPDEF;



typedef struct
{
  uint8_t check_flag; //
  uint8_t pres_flag;
  uint8_t mot_flag;
  uint8_t tamb_shock_flag;
}T_STHS34PF80_FLAG_TYPDEF;


extern T_STHS34PF80_FLAG_TYPDEF t_sths34pf80_flag;


// extern fun
/*iic bus init*/
void sths34_iic_init(void);

/*base fun*/
uint8_t STHS_WriteReg(uint8_t reg, uint8_t data);     //write reg
uint8_t STHS_ReadReg(uint8_t reg);                    //read reg
uint8_t STHS_ReadReg_1(uint8_t reg, uint8_t *p_dat);  //read reg

uint8_t STHS_writ_func(uint8_t reg, uint8_t *pdat);   //write fun reg
uint8_t STHS_read_func(uint8_t reg, uint8_t *pdat);   //write read reg


/*fun*/
void STHS_en_check(void);
void STHS_den_check(void);
void SHHS_boot_data(void);

/*user fun*/
void SHHS34pf80_init(void); //use for main init
uint8_t SHHS34pf80_person_flag_read(void);//use for cycle call 
uint8_t SHHS34pf80_person_data_read(void);


//void sths_test_reg_read(void); //wxf-test





#endif


