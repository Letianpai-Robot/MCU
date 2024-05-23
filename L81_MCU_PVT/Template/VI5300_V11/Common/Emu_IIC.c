/***********************************************************************************************************************
* File Name    : Emu_IIC.c
* Version      : V1.0
* Device(s)    : Common
* Description  : This file implements device driver for IIC module.
* Creation Date: 2022/06/01
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "Emu_IIC.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: IICDelay_1us
* Description  : The definition of time Delay_1us funtion.
* Arguments    : num: Multiples of 1us periods
* Return Value : None
***********************************************************************************************************************/
static void IICDelay_1us(unsigned int num)
{
	unsigned int time_num;	
	time_num = num;
  while(time_num--)
	{
		__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();
	}
}


/***********************************************************************************************************************
* Function Name: Emu_I2C_Init
* Description  : This function initializes IIC.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void Emu_I2C_Init(void)
{
	/* Set clock pin for IIC0 */
  IICPORT_SCL_LOW();
	IICPORT_SCL_OutputMode();
	/* Set data transfer I/O pin for IIC */
	IICPORT_SDA_LOW();
	IICPORT_SDA_OutputMode();	
}


/***********************************************************************************************************************
* Function Name: Emu_I2C_Stop
* Description  : This function stop IIC.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void Emu_I2C_Stop(void)
{
	IICPORT_SCL_InputMode();	
	IICPORT_SDA_InputMode();	
}


/***********************************************************************************************************************
* Function Name: Emu_I2C_StartBit
* Description  : This function is IIC Transfer start conditions.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void Emu_I2C_StartBit()
{
	IICPORT_SDA_HIGH();
		IICDelay_1us(5);
	IICPORT_SCL_HIGH();
		IICDelay_1us(5);
	IICPORT_SDA_LOW();
		IICDelay_1us(5);
	IICPORT_SCL_LOW();
}


/***********************************************************************************************************************
* Function Name: Emu_I2C_StopBit
* Description  : This function is IIC Transfer stop conditions.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void Emu_I2C_StopBit()
{
	IICPORT_SDA_LOW();
		IICDelay_1us(2);
	IICPORT_SCL_HIGH();
		IICDelay_1us(5);
	IICPORT_SDA_HIGH();
		IICDelay_1us(5);
}


/***********************************************************************************************************************
* Function Name: ACK_Master
* Description  : This function is the ACK after slave devices receive a data from master(mcu).
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static IIC_STATUS ACK_Master()
{
	unsigned char timeout = 10;

	IICPORT_SDA_InputMode();
		IICDelay_1us(IICFREQ_WIDTH);    
	IICPORT_SCL_HIGH();
		IICDelay_1us(IICFREQ_WIDTH);    
	while(IICPORT_SDA_READ() == 1)
	{
		if ((timeout--) == 0) 
		{
			IICPORT_SCL_LOW();
			IICPORT_SDA_OutputMode(); 
			return IIC_ERROR;
		}	
		IICDelay_1us(IICFREQ_WIDTH); 
	}
	IICPORT_SCL_LOW();
	IICPORT_SDA_OutputMode(); 
	return IIC_OK;
}  


/***********************************************************************************************************************
* Function Name: ACK_Slave
* Description  : The definition of the ACK ,only used when read data sequentially,
*									after master(mcu) receive a Byte from slave devices 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void ACK_Slave()
{
	IICPORT_SCL_LOW(); 
	IICPORT_SDA_LOW(); 
		IICDelay_1us(IICFREQ_WIDTH);  
	IICPORT_SCL_HIGH();
		IICDelay_1us(IICFREQ_WIDTH); 
	IICPORT_SCL_LOW();  
}


/***********************************************************************************************************************
* Function Name: Write_OneByteTo_Slave
* Description  : This function is send a byte of data by IIC.
* Arguments    : wr_byte :	send a byte of data
* Return Value : None
***********************************************************************************************************************/
static void Write_OneByteTo_Slave(unsigned char wr_byte)
{
	unsigned char i;
	unsigned char BYTE;
	BYTE = wr_byte;
	for(i=0; i<8; i++)
	{     
			IICPORT_SCL_LOW();
			if(BYTE&0x80)  
			{
				IICPORT_SDA_HIGH();
			}
			else
			{
				IICPORT_SDA_LOW();
			}
			IICDelay_1us(IICFREQ_WIDTH);
			IICPORT_SCL_HIGH();
			BYTE = BYTE<<1;
				IICDelay_1us(IICFREQ_WIDTH);
	}
	IICPORT_SCL_LOW();     
} 


/***********************************************************************************************************************
* Function Name: Read_OneByteFrom_Slave
* Description  : This function is receive a byte of data by IIC.
* Arguments    : None
* Return Value : receive a byte of data
***********************************************************************************************************************/
static unsigned char Read_OneByteFrom_Slave()
{

  unsigned char i,read_data;
  IICPORT_SDA_InputMode();
	for(i=0; i<8; i++)
	{
		IICPORT_SCL_LOW();   
			IICDelay_1us(IICFREQ_WIDTH);
		IICPORT_SCL_HIGH();
			IICDelay_1us(IICFREQ_WIDTH);
		read_data = (read_data<<1)|IICPORT_SDA_READ();
	}
	IICPORT_SCL_LOW();
	IICPORT_SDA_OutputMode();
	return read_data;
}


/***********************************************************************************************************************
* Function Name: Emu_I2C_Write
* Description  : This function starts IIC send data as master mode.
* Arguments    : 	Dev_addr :		set address for select slave
*									Access_addr :	slave devices Access address
*									Wr_Buffer :		transfer buffer pointer
*									num :			buffer size
* Return Value : None
***********************************************************************************************************************/
IIC_STATUS Emu_I2C_Write(unsigned char Dev_addr, unsigned char Access_addr, unsigned char *Wr_Buffer, unsigned char Num)
{
    unsigned char i;
    Emu_I2C_StartBit();
    Write_OneByteTo_Slave(Dev_addr);
    if( ACK_Master() != IIC_OK )
			return IIC_ERROR;
    Write_OneByteTo_Slave(Access_addr);
    if( ACK_Master() != IIC_OK )
			return IIC_ERROR;

    for(i=0; i<Num; i++)
    {
        Write_OneByteTo_Slave(*(Wr_Buffer+i));
        if( ACK_Master() != IIC_OK )
					return IIC_ERROR;
    }
    Emu_I2C_StopBit();	
		return IIC_OK;
}


/***********************************************************************************************************************
* Function Name: Emu_I2C_Write
* Description  : This function starts IIC send data as master mode.
* Arguments    : 	Dev_addr :		set address for select slave
*									Access_addr :	slave devices Access address
*									Re_Buffer :		transfer buffer pointer
*									num :			buffer size
* Return Value : None
***********************************************************************************************************************/
IIC_STATUS Emu_I2C_Read(unsigned char Dev_addr, unsigned char Access_addr, unsigned char *Re_Buffer, unsigned char Num)
{
    unsigned char i = 0;    
    Emu_I2C_StartBit();
    Write_OneByteTo_Slave(Dev_addr);
    if( ACK_Master() != IIC_OK )
			return IIC_ERROR;
    Write_OneByteTo_Slave(Access_addr);
    if( ACK_Master() != IIC_OK )
			return IIC_ERROR;
		//Emu_I2C_StopBit();
    
    Emu_I2C_StartBit();
    Write_OneByteTo_Slave(Dev_addr|0x01);
    if( ACK_Master() != IIC_OK )
			return IIC_ERROR;

    for(i=0; i<Num-1; i++)
    {
       *(Re_Buffer+i) = Read_OneByteFrom_Slave();
       ACK_Slave();
    }
    *(Re_Buffer+i) = Read_OneByteFrom_Slave(); 
    Emu_I2C_StopBit();
		return IIC_OK;
}

/* Start adding user code. Do not edit comment generated here */
/* End user code adding. Do not edit comment generated here */
