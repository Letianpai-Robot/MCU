
#include <stdio.h>
#include "gxhtc3iic.h"

#include "L81_AT.h"
#include "L81_inf.h"
#include "L81_humiture_task.h"

#define GXHTC3_IIC_DEBUG_EN            0     //debug EN=1 DEN=0
#if GXHTC3_IIC_DEBUG_EN
#define Gxhtc3I2cLog(...) printf(__VA_ARGS__)
#else
#define Gxhtc3I2cLog(...) 
#endif

// address 1d
//#define STK3311_ADDRESS_WRITE 0x90
//#define STK3311_ADDRESS_READ 0x91
#define GXHTC3_ADDRESS7    (0x70<<1)


uint32_t gxhtc3_block_count = 0;


#if GXHTC3_USE_HARD_I2C_EN
static void rcu_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(GXHTC3_IIC_PIN_CLOCK);
    /* enable GXHTC3_IIC_BUS_NUM clock */
    rcu_periph_clock_enable(GXHTC3_IIC_PERIPH_CLOCK);
}

/*!
    \brief      configure the I2C GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void i2c_gpio_config(void)
{
    /* connect PB10 to I2C2_SCL */
    gpio_af_set(GXHTC3_IIC_SCK_PORT, GXHTC3_IIC_SCK_AF, GXHTC3_IIC_SCK_PIN);
    /* connect PB11 to I2C2_SDA */
    gpio_af_set(GXHTC3_IIC_SDA_PORT, GXHTC3_IIC_SDA_AF, GXHTC3_IIC_SDA_PIN);
    /* configure GPIO pins of GXHTC3_IIC_BUS_NUM */
    gpio_mode_set(GXHTC3_IIC_SCK_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GXHTC3_IIC_SCK_PIN);
    gpio_output_options_set(GXHTC3_IIC_SCK_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GXHTC3_IIC_SCK_PIN);
    gpio_mode_set(GXHTC3_IIC_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GXHTC3_IIC_SDA_PIN);
    gpio_output_options_set(GXHTC3_IIC_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GXHTC3_IIC_SDA_PIN);
  
}

/*!
    \brief      configure the I2C interface
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void i2c_config(void)
{
  /* configure I2C timing */
//  i2c_timing_config(GXHTC3_IIC_BUS_NUM, 0, 0x3, 0);
//  i2c_master_clock_config(GXHTC3_IIC_BUS_NUM, 0x13, 0x36);
  
  i2c_timing_config(GXHTC3_IIC_BUS_NUM, 0, 0x3, 0);
  i2c_master_clock_config(GXHTC3_IIC_BUS_NUM, 0x33, 0x36); 
  /* enable I2C1 */
  i2c_enable(GXHTC3_IIC_BUS_NUM);	
}


static uint8_t writeCont(uint8_t *p_dat, uint8_t len)
{
  
  if(p_dat == NULL)
  {
    return GXHTC3_IIC_ERR;
  }
  
  __disable_irq();
  i2c_address_config(GXHTC3_IIC_BUS_NUM, GXHTC3_ADDRESS7, I2C_ADDFORMAT_7BITS);
  /* configure slave address */
  i2c_master_addressing(GXHTC3_IIC_BUS_NUM, GXHTC3_ADDRESS7, I2C_MASTER_TRANSMIT);
  /* configure number of bytes to be transferred */
  i2c_transfer_byte_number_config(GXHTC3_IIC_BUS_NUM, len);
  
  
  /* wait until I2C bus is idle */
  gxhtc3_block_count = 0;
  while(i2c_flag_get(GXHTC3_IIC_BUS_NUM, I2C_FLAG_I2CBSY))
  {
    gxhtc3_block_count ++;
    if(gxhtc3_block_count >= GXHTC3_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return GXHTC3_IIC_ERR;
    }
  }
  /* send a start condition to I2C bus */
  i2c_start_on_bus(GXHTC3_IIC_BUS_NUM);

  /* wait until the transmit data buffer is empty */
  I2C_STAT(GXHTC3_IIC_BUS_NUM) |= I2C_STAT_TBE;
  gxhtc3_block_count = 0;
  while(!i2c_flag_get(GXHTC3_IIC_BUS_NUM, I2C_FLAG_TBE))
  {
    gxhtc3_block_count ++;
    if(gxhtc3_block_count >= GXHTC3_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return GXHTC3_IIC_ERR;
    }
  }
  for(uint8_t i=0; i<len; i++)
  {
    /* data transmission */
    i2c_data_transmit(GXHTC3_IIC_BUS_NUM, p_dat[i]);
    /* wait until the TI bit is set */
    gxhtc3_block_count = 0;
    while(!i2c_flag_get(GXHTC3_IIC_BUS_NUM, I2C_FLAG_TBE))
    {
      gxhtc3_block_count ++;
      if(gxhtc3_block_count >= GXHTC3_IIC_MAX_BLOCK)
      {
        __enable_irq();
        return GXHTC3_IIC_ERR;
      }
    }
  }
	
  /* wait for transfer complete flag */
  gxhtc3_block_count = 0;
  while(!i2c_flag_get(GXHTC3_IIC_BUS_NUM, I2C_FLAG_TC))
  {
    gxhtc3_block_count ++;
    if(gxhtc3_block_count >= GXHTC3_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return GXHTC3_IIC_ERR;
    }
  }
  /* send a stop condition to I2C bus */
  i2c_stop_on_bus(GXHTC3_IIC_BUS_NUM);
  /* wait until stop condition generate */
  gxhtc3_block_count = 0;
  while(!i2c_flag_get(GXHTC3_IIC_BUS_NUM, I2C_FLAG_STPDET))
  {
    gxhtc3_block_count ++;
    if(gxhtc3_block_count >= GXHTC3_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return GXHTC3_IIC_ERR;
    }
  }
  /* clear the STPDET bit */
  i2c_flag_clear(GXHTC3_IIC_BUS_NUM, I2C_FLAG_STPDET);
  
  
  __enable_irq();
  return GXHTC3_IIC_OK;
}

static uint8_t readCont(uint8_t *p_dat, uint8_t len)
{
  
  if(p_dat == NULL)
  {
    return GXHTC3_IIC_ERR;
  }
  
  __disable_irq();
  i2c_address_config(GXHTC3_IIC_BUS_NUM, GXHTC3_ADDRESS7, I2C_ADDFORMAT_7BITS);
  i2c_master_addressing(GXHTC3_IIC_BUS_NUM, GXHTC3_ADDRESS7, I2C_MASTER_RECEIVE);
  i2c_transfer_byte_number_config(GXHTC3_IIC_BUS_NUM, len);
  
  /* wait until I2C bus is idle */
  gxhtc3_block_count = 0;
  while(i2c_flag_get(GXHTC3_IIC_BUS_NUM, I2C_FLAG_I2CBSY))
  {
    gxhtc3_block_count ++;
    if(gxhtc3_block_count >= GXHTC3_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return GXHTC3_IIC_ERR;
    }
  }
  /* send a start condition to I2C bus */
  i2c_start_on_bus(GXHTC3_IIC_BUS_NUM);
  for(uint8_t i = 0; i < len; i++) 
  {
      /* wait until the RBNE bit is set */
    gxhtc3_block_count = 0;
    while(!i2c_flag_get(GXHTC3_IIC_BUS_NUM, I2C_FLAG_RBNE))
    {
      gxhtc3_block_count ++;
      if(gxhtc3_block_count >= GXHTC3_IIC_MAX_BLOCK)
      {
        __enable_irq();
        return GXHTC3_IIC_ERR;
      }
    }
    /* read a data from I2C_DATA */
    p_dat[i] = i2c_data_receive(GXHTC3_IIC_BUS_NUM);
  }
  gxhtc3_block_count = 0;
  while(!i2c_flag_get(GXHTC3_IIC_BUS_NUM, I2C_FLAG_TC))
  {
    gxhtc3_block_count ++;
    if(gxhtc3_block_count >= GXHTC3_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return GXHTC3_IIC_ERR;
    }
  }
  /* send a stop condition to I2C bus */
  i2c_stop_on_bus(GXHTC3_IIC_BUS_NUM);
  /* wait until stop condition generate */
  gxhtc3_block_count = 0;
  while(!i2c_flag_get(GXHTC3_IIC_BUS_NUM, I2C_FLAG_STPDET))
  {
    gxhtc3_block_count ++;
    if(gxhtc3_block_count >= GXHTC3_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return GXHTC3_IIC_ERR;
    }
  }
  /* clear the STPDET bit */
  i2c_flag_clear(GXHTC3_IIC_BUS_NUM, I2C_FLAG_STPDET);
  
  __enable_irq();
  return GXHTC3_IIC_OK;
}
#else

/**
  ******************************************************************************
  * @file    
  * @author  wxf
  * @version V1.0
  * @date    sim I2C
  * @brief   

  ******************************************************************************
  */

/**/

#define I2C1_USE      0   //1=hard i2c pin to soft; 0=soft i2c
#if I2C1_USE

#define GPIO_PORT_I2C	    GPIOB			              
#define RCC_I2C_PORT 	    RCU_GPIOB		

#define I2C_SCL_PIN		    GPIO_PIN_10			
#define I2C_SDA_PIN		    GPIO_PIN_11		

#else 

#define GPIO_PORT_I2C	    GPIOA			              
#define RCC_I2C_PORT 	    RCU_GPIOA		

#define I2C_SCL_PIN		    GPIO_PIN_10		
#define I2C_SDA_PIN		    GPIO_PIN_11			

#endif 

	#define I2C_SCL_OUTPUT()	  do{gpio_mode_set(GPIO_PORT_I2C, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, I2C_SCL_PIN);\
                                gpio_output_options_set(GPIO_PORT_I2C, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SCL_PIN);}while(0)
	#define I2C_SCL_INPUT()		  do{gpio_mode_set(GPIO_PORT_I2C, GPIO_MODE_INPUT, GPIO_PUPD_NONE, I2C_SCL_PIN);\
                                gpio_output_options_set(GPIO_PORT_I2C, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SCL_PIN);}while(0)
	
	#define I2C_SDA_OUTPUT()	do{gpio_mode_set(GPIO_PORT_I2C, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, I2C_SDA_PIN);\
                                gpio_output_options_set(GPIO_PORT_I2C, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SDA_PIN);}while(0)      
	#define I2C_SDA_INPUT()			do{gpio_mode_set(GPIO_PORT_I2C, GPIO_MODE_INPUT, GPIO_PUPD_NONE, I2C_SDA_PIN);\
                                gpio_output_options_set(GPIO_PORT_I2C, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SDA_PIN);}while(0)
	
	#define I2C_SCL_1()       gpio_bit_set(GPIO_PORT_I2C, I2C_SCL_PIN)		  /* SCL = 1 */
	#define I2C_SCL_0()       gpio_bit_reset(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 0 */
	
	#define I2C_SDA_1()       gpio_bit_set(GPIO_PORT_I2C, I2C_SDA_PIN)		  /* SDA = 1 */
	#define I2C_SDA_0()       gpio_bit_reset(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 0 */
	
	#define I2C_SDA_READ()    gpio_input_bit_get(GPIO_PORT_I2C, I2C_SDA_PIN)	/* SDA=? */


static void i2c_Ack(void);
static void i2c_NAck(void);

static void i2c_delay_us(int nus)
{
	//volatile uint32_t i;

	while(nus--)
	{
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
#if 1

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
#endif
#if 0
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
#endif

	}
}


#define i2c_Delay()		i2c_delay_us(2)

static void i2c_Start(void)
{
	/* ?SCL????,SDA?????????I2C?????? */
	
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();
	
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************

*********************************************************************************************************
*/
static void i2c_Stop(void)
{
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();
	I2C_SCL_0(); // Junger
	i2c_Delay(); //Junger

	/* ?SCL????,SDA?????????I2C?????? */
	I2C_SDA_0();
	i2c_Delay();  //Junger
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
}

/*
*********************************************************************************************************

*********************************************************************************************************
*/
static void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* ????????bit7 */
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		//i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();	
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); // ????
		}
		_ucByte <<= 1;	/* ????bit */
		i2c_Delay();
	}
}

/*
*********************************************************************************************************

*********************************************************************************************************
*/
static uint8_t i2c_ReadByte(uint8_t ack)
{
	uint8_t i;
	uint8_t value;

	/* ???1?bit????bit7 */
	I2C_SDA_INPUT();	// set data input	
	i2c_Delay();
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		//I2C_SCL_1();
		//i2c_Delay();
		I2C_SCL_0();
		i2c_Delay();
	}
	
	I2C_SDA_OUTPUT();	// set data output	
	i2c_Delay();
	if(ack==0)
		i2c_NAck();
	else
		i2c_Ack();
	return value;
}

/*
*********************************************************************************************************

*********************************************************************************************************
*/
static uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU??SDA?? */
	I2C_SDA_INPUT();	//set data input
	i2c_Delay();
	I2C_SCL_1();	/* CPU??SCL = 1, ???????ACK?? */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU??SDA???? */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	I2C_SDA_OUTPUT();	//set data output
	i2c_Delay();
	return re;
}

/*
*********************************************************************************************************

*********************************************************************************************************
*/
static void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU??SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU??1??? */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU??SDA?? */
}

/*
*********************************************************************************************************

*********************************************************************************************************
*/
static void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU??SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU??1??? */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();	
}

/*
*********************************************************************************************************

*********************************************************************************************************
*/
void i2c_sw_gpio_config(void)
{
  /* enable the GPIO clock */
  rcu_periph_clock_enable(RCC_I2C_PORT);
  
  /*config */
  gpio_mode_set(GPIO_PORT_I2C, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, I2C_SCL_PIN);
  gpio_output_options_set(GPIO_PORT_I2C, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SCL_PIN);
  
  gpio_mode_set(GPIO_PORT_I2C, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, I2C_SDA_PIN);
  gpio_output_options_set(GPIO_PORT_I2C, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SDA_PIN);
  
  /*set*/
  gpio_bit_set(GPIO_PORT_I2C, I2C_SCL_PIN);
  gpio_bit_set(GPIO_PORT_I2C, I2C_SDA_PIN);
  
  /**/
	i2c_Stop();
}





uint8_t qst_sw_writereg(uint8_t slave, uint8_t reg_add,uint8_t reg_dat)
{
	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_add);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_dat);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_Stop();

	return 1;
}

uint8_t qst_sw_writeregs(uint8_t slave, uint8_t reg_add, uint8_t *reg_dat, uint8_t len)
{
	uint8_t i;

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_add);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	for(i=0; i<len; i++)
	{
		i2c_SendByte(reg_dat[i]);	
		if(i2c_WaitAck())
		{
			return 0;
		}
	}
	i2c_Stop();

	return 1;
}

uint8_t qst_sw_readreg(uint8_t slave, uint8_t reg_add, uint8_t *buf, uint16_t num)
{
	uint16_t i;

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_add);
	if(i2c_WaitAck())
	{
		return 0;
	}

	i2c_Start();
	i2c_SendByte(slave|0x01);
	if(i2c_WaitAck())
	{
		return 0;
	}

	for(i=0;i<(num-1);i++){
		*buf=i2c_ReadByte(1);
		buf++;
	}
	*buf=i2c_ReadByte(0);
	i2c_Stop();

	return 1;
}

static uint8_t writeCont(uint8_t *p_dat, uint8_t len)
{
  
  i2c_Start();
	i2c_SendByte(GXHTC3_ADDRESS7 | 0x00);
	if(i2c_WaitAck())
	{
		return 0;
	}
  
  for(uint16_t i=0; i<len; i++)
  {
    i2c_SendByte(p_dat[i]);
		//Gxhtc3I2cLog("i2c_WaitAck() %d :  %d\r\n",i, i2c_WaitAck());	
    if(i2c_WaitAck())
    {
      return 0;
    }
  }
	
	i2c_Stop();

	return 1;
  
}

static uint8_t readCont(uint8_t *p_dat, uint8_t len)
{
  int16_t i = 0;
  int16_t num = (int16_t)len;

	i2c_Start();
	i2c_SendByte(GXHTC3_ADDRESS7 | 0x01);
	if(i2c_WaitAck())
	{
		return 0;
	}
 
  if(num >1)
  {
    for(i=0; i<num-1; i++)
    {
      p_dat[i] = i2c_ReadByte(1);
			//Gxhtc3I2cLog("i2c_WaitAck() %d :  %d\r\n",i, i2c_WaitAck());
    }
  }
  
  p_dat[i]=i2c_ReadByte(0);
	//Gxhtc3I2cLog("i2c_WaitAck() %d :  %d\r\n",i, i2c_WaitAck());
  
	i2c_Stop();

	return 0;//wxf-test 0506
}


#endif


void gxhtc3_iic_init(void) 
{    
  #if GXHTC3_USE_HARD_I2C_EN
  i2c_deinit(GXHTC3_IIC_BUS_NUM);
  rcu_config();
  i2c_gpio_config();
  i2c_config();
  #else
  
  i2c_sw_gpio_config();
  #endif
}



//write reg cmd-2BYT
uint8_t gxhtc3_write_cmd(uint16_t data)
{
  uint8_t err = 0;
  union data
  {
    uint8_t u_datbuf[2];
    struct add_data
    {
      uint8_t cmd_h;
      uint8_t cmd_l;
    }t_add_data;
  }u_data;
  
  u_data.t_add_data.cmd_l = data & 0x00ff;
  u_data.t_add_data.cmd_h = (data>>8)&0x00ff;
  
  err = writeCont(u_data.u_datbuf, 2);
  return err;

}

uint8_t gxhtc3_read_data_3B(uint8_t *p_dat)
{
  uint8_t err = 0;
  err = readCont(p_dat, 3);
  return err;
}

uint8_t gxhtc3_cmd_read_3B(uint16_t cmd, uint8_t *p_dat)
{
  uint8_t err = 0;
  
  err = gxhtc3_write_cmd(cmd);
  err = readCont(p_dat, 3);
  return err;
}



//user

void gxhtc3_init(void)
{
  gxhtc3_iic_init(); //init i2c bus
}


uint16_t gxhtc3_read_id(void)
{
  uint16_t cmd_reset = GXHTC3_S_RESET_CMD;
	gxhtc3_write_cmd(cmd_reset);
	
  uint8_t buf[3]= {0};
  uint16_t cmd = GXHTC3_ID_CMD;
  uint16_t id = 0;
  uint8_t err = 0;
  err = gxhtc3_cmd_read_3B(cmd, buf);
  
  id = buf[0];
  id<<=8;
  id |= buf[1];
  
  Gxhtc3I2cLog("TEM ID= %02X %02X  err=%d\r\n",id, buf[2], err);
  
  return id;
}
//uint16_t gxhtc3_read_tem(void)
//{
//  uint8_t buf[3]= {0};
//  uint16_t cmd = GXHTC3_NORMAL_T_CMD;
//  uint16_t tem = 0;
//  uint8_t err = 0;
//  err = gxhtc3_cmd_read_3B(cmd, buf);
//  
//  tem = buf[0];
//  tem<<=8;
//  tem |= buf[1];
//  
//  Gxhtc3I2cLog("TEM ID= %02X %02X  err=%d\r\n",tem, buf[2], err);
//  
//  return tem;
//}

//uint16_t gxhtc3_read_hum(void)
//{
//  uint8_t buf[3]= {0};
//  uint16_t cmd = GXHTC3_NORMAL_H_CMD;
//  uint16_t hum = 0;
//  uint8_t err = 0;
//  err = gxhtc3_cmd_read_3B(cmd, buf);
//  
//  hum = buf[0];
//  hum<<=8;
//  hum |= buf[1];
//  
//  Gxhtc3I2cLog("TEM ID= %02X %02X  err=%d\r\n",hum, buf[2], err);
//  
//  return hum;
//}

//AT CMD FUN
uint8_t l81_AT_gxhtcr_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	float tem = 0;
	float hum = 0;
	T_HUMITURE_PARAM_TYPDEF humiture = get_humiture();
	
  ATcmd_split_params(params, param, &param_num);

	if (param_num != 1U){  //
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
	
	uint32_t cmd = String2Int(param[0]);
		
	if ((cmd < 0U) || (cmd > 3u)) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,err cmd value eg:[0~3]\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
  //
  switch (cmd)
  {
    case 0://read id
    {
//			humiture_task_init(); //rjq-add 0505
//      gxhtc3_iic_init();
      uint16_t id = gxhtc3_read_id();
      
      if(id != 0)
      {
        printf("AT+RES,ACK\r\n");
        printf("AT+RES,0x%04X\r\n",id);
        printf("AT+RES,end\r\n");
      }
      else
      {
        printf("AT+RES,ACK\r\n");
        printf("AT+RES,ERR,didn't get device id\r\n");
        printf("AT+RES,end\r\n");
      }
    }
    break;
    case 1://temp
			tem = get_tem();
			printf("AT+RES,ACK\r\n");
			printf("AT+RES,%f\r\n",tem);
			printf("AT+RES,end\r\n");
      break;
    case 2://hum
			hum = get_hum();
      printf("AT+RES,ACK\r\n");
      printf("AT+RES,%f\r\n",hum);
      printf("AT+RES,end\r\n");			
      break;
    case 3://temp+hum
			tem = get_tem();
			hum = get_hum();
      printf("AT+RES,ACK\r\n");
      printf("AT+RES,%f,%f\r\n",tem,hum);
      printf("AT+RES,end\r\n");			
      break;
    
    default:
      break;
  }
    
    return 1U;	
}




