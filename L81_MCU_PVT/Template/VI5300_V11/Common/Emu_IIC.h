/***********************************************************************************************************************
* File Name    : Emu_IIC.h
* Version      : V1.0
* Device(s)    : Common
* Description  : This file implements device driver for IIC module.
* Creation Date: 2022/06/01
***********************************************************************************************************************/

#ifndef EMU_IIC_H
#define EMU_IIC_H
/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "VI530x_Common.h"

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/

typedef unsigned char	IIC_STATUS;
#define IIC_OK 		0
#define IIC_ERROR 1


#define IICPORT_SCL_LOW()					VI530x_IIC_SCL_LOW()	
#define IICPORT_SCL_HIGH()				VI530x_IIC_SCL_HIGH()
#define IICPORT_SCL_OutputMode()	__NOP()
#define IICPORT_SCL_InputMode()		__NOP()
#define IICPORT_SDA_LOW()					VI530x_IIC_SDA_LOW()		
#define IICPORT_SDA_HIGH()				VI530x_IIC_SDA_HIGH()
#define IICPORT_SDA_OutputMode()	VI530x_IIC_SDA_OUTPUTMODE()
#define IICPORT_SDA_InputMode()		VI530x_IIC_SDA_INPUTMODE()			
#define IICPORT_SDA_READ()				VI530x_IIC_SDA_READ()

#define IICFREQ_WIDTH	1		//1us



/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
void Emu_I2C_Init(void);
void Emu_I2C_Stop(void);
IIC_STATUS Emu_I2C_Write(unsigned char Dev_addr, unsigned char Access_addr, unsigned char *Wr_Buffer, unsigned char Num);
IIC_STATUS Emu_I2C_Read(unsigned char Dev_addr, unsigned char Access_addr, unsigned char *Re_Buffer, unsigned char Num);

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/

#endif
