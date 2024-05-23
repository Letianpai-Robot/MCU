

#include <stdio.h>
#include "stk3311iic.h"

#define STK_IIC_DEBUG_EN            0     //debug EN=1 DEN=0
#if STK_IIC_DEBUG_EN
#define StkI2cLog(...) printf(__VA_ARGS__)
#else
#define StkI2cLog(...) 
#endif

// address 1d
//#define STK3311_ADDRESS_WRITE 0x90
//#define STK3311_ADDRESS_READ 0x91
#define STK3311_ADDRESS7    (0x48<<1)

T_STK3311_LIGHT_TYPDEF t_stk3311_litht = {0};

uint32_t block_count = 0;




static void rcu_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(STK3311_IIC_PIN_CLOCK);
    /* enable STK3311_IIC_BUS_NUM clock */
    rcu_periph_clock_enable(STK3311_IIC_PERIPH_CLOCK);
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
    gpio_af_set(STK3311_IIC_SCK_PORT, STK3311_IIC_SCK_AF, STK3311_IIC_SCK_PIN);
    /* connect PB11 to I2C2_SDA */
    gpio_af_set(STK3311_IIC_SDA_PORT, STK3311_IIC_SDA_AF, STK3311_IIC_SDA_PIN);
    /* configure GPIO pins of STK3311_IIC_BUS_NUM */
    gpio_mode_set(STK3311_IIC_SCK_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, STK3311_IIC_SCK_PIN);
    gpio_output_options_set(STK3311_IIC_SCK_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, STK3311_IIC_SCK_PIN);
    gpio_mode_set(STK3311_IIC_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, STK3311_IIC_SDA_PIN);
    gpio_output_options_set(STK3311_IIC_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, STK3311_IIC_SDA_PIN);
  
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
//  i2c_timing_config(STK3311_IIC_BUS_NUM, 0, 0x3, 0);
//  i2c_master_clock_config(STK3311_IIC_BUS_NUM, 0x13, 0x36);
  
  i2c_timing_config(STK3311_IIC_BUS_NUM, 0, 0x3, 0);
  i2c_master_clock_config(STK3311_IIC_BUS_NUM, 0x33, 0x36); 
  /* enable I2C1 */
  i2c_enable(STK3311_IIC_BUS_NUM);	
}


static uint8_t writeCont(uint8_t *p_dat, uint8_t len)
{
  
  if(p_dat == NULL)
  {
    return STK3311_IIC_ERR;
  }
  
  __disable_irq();
  i2c_address_config(STK3311_IIC_BUS_NUM, STK3311_ADDRESS7, I2C_ADDFORMAT_7BITS);
  /* configure slave address */
  i2c_master_addressing(STK3311_IIC_BUS_NUM, STK3311_ADDRESS7, I2C_MASTER_TRANSMIT);
  /* configure number of bytes to be transferred */
  i2c_transfer_byte_number_config(STK3311_IIC_BUS_NUM, len);
  
  
  /* wait until I2C bus is idle */
  block_count = 0;
  while(i2c_flag_get(STK3311_IIC_BUS_NUM, I2C_FLAG_I2CBSY))
  {
    block_count ++;
    if(block_count >= STK3311_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STK3311_IIC_ERR;
    }
  }
  /* send a start condition to I2C bus */
  i2c_start_on_bus(STK3311_IIC_BUS_NUM);

  /* wait until the transmit data buffer is empty */
  I2C_STAT(STK3311_IIC_BUS_NUM) |= I2C_STAT_TBE;
  block_count = 0;
  while(!i2c_flag_get(STK3311_IIC_BUS_NUM, I2C_FLAG_TBE))
  {
    block_count ++;
    if(block_count >= STK3311_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STK3311_IIC_ERR;
    }
  }
  for(uint8_t i=0; i<len; i++)
  {
    /* data transmission */
    i2c_data_transmit(STK3311_IIC_BUS_NUM, p_dat[i]);
    /* wait until the TI bit is set */
    block_count = 0;
    while(!i2c_flag_get(STK3311_IIC_BUS_NUM, I2C_FLAG_TBE))
    {
      block_count ++;
      if(block_count >= STK3311_IIC_MAX_BLOCK)
      {
        __enable_irq();
        return STK3311_IIC_ERR;
      }
    }
  }
	
  /* wait for transfer complete flag */
  block_count = 0;
  while(!i2c_flag_get(STK3311_IIC_BUS_NUM, I2C_FLAG_TC))
  {
    block_count ++;
    if(block_count >= STK3311_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STK3311_IIC_ERR;
    }
  }
  /* send a stop condition to I2C bus */
  i2c_stop_on_bus(STK3311_IIC_BUS_NUM);
  /* wait until stop condition generate */
  block_count = 0;
  while(!i2c_flag_get(STK3311_IIC_BUS_NUM, I2C_FLAG_STPDET))
  {
    block_count ++;
    if(block_count >= STK3311_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STK3311_IIC_ERR;
    }
  }
  /* clear the STPDET bit */
  i2c_flag_clear(STK3311_IIC_BUS_NUM, I2C_FLAG_STPDET);
  
  
  __enable_irq();
  return STK3311_IIC_OK;
}

static uint8_t readCont(uint8_t *p_dat, uint8_t len)
{
  
  if(p_dat == NULL)
  {
    return STK3311_IIC_ERR;
  }
  
  __disable_irq();
  i2c_address_config(STK3311_IIC_BUS_NUM, STK3311_ADDRESS7, I2C_ADDFORMAT_7BITS);
  i2c_master_addressing(STK3311_IIC_BUS_NUM, STK3311_ADDRESS7, I2C_MASTER_RECEIVE);
  i2c_transfer_byte_number_config(STK3311_IIC_BUS_NUM, len);
  
  /* wait until I2C bus is idle */
  block_count = 0;
  while(i2c_flag_get(STK3311_IIC_BUS_NUM, I2C_FLAG_I2CBSY))
  {
    block_count ++;
    if(block_count >= STK3311_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STK3311_IIC_ERR;
    }
  }
  /* send a start condition to I2C bus */
  i2c_start_on_bus(STK3311_IIC_BUS_NUM);
  for(uint8_t i = 0; i < len; i++) 
  {
      /* wait until the RBNE bit is set */
    block_count = 0;
    while(!i2c_flag_get(STK3311_IIC_BUS_NUM, I2C_FLAG_RBNE))
    {
      block_count ++;
      if(block_count >= STK3311_IIC_MAX_BLOCK)
      {
        __enable_irq();
        return STK3311_IIC_ERR;
      }
    }
    /* read a data from I2C_DATA */
    p_dat[i] = i2c_data_receive(STK3311_IIC_BUS_NUM);
  }
  block_count = 0;
  while(!i2c_flag_get(STK3311_IIC_BUS_NUM, I2C_FLAG_TC))
  {
    block_count ++;
    if(block_count >= STK3311_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STK3311_IIC_ERR;
    }
  }
  /* send a stop condition to I2C bus */
  i2c_stop_on_bus(STK3311_IIC_BUS_NUM);
  /* wait until stop condition generate */
  block_count = 0;
  while(!i2c_flag_get(STK3311_IIC_BUS_NUM, I2C_FLAG_STPDET))
  {
    block_count ++;
    if(block_count >= STK3311_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STK3311_IIC_ERR;
    }
  }
  /* clear the STPDET bit */
  i2c_flag_clear(STK3311_IIC_BUS_NUM, I2C_FLAG_STPDET);
  
  __enable_irq();
  return STK3311_IIC_OK;
}




void stk3311_iic_init(void) 
{                     
  rcu_config();
  i2c_gpio_config();
  i2c_config();
}



//write reg cmd-1BYT
uint8_t STK_WriteReg(uint8_t reg, uint8_t data)
{
  uint8_t err = 0;
  union data
  {
    uint8_t u_datbuf[2];
    struct add_data
    {
      uint8_t reg;
      uint8_t data;
    }t_add_data;
  }u_data;
  
  u_data.t_add_data.reg = reg;
  u_data.t_add_data.data = data;
  
  err = writeCont(u_data.u_datbuf, 2);
  return err;

}


//read one BYTE data
uint8_t STK_ReadReg(uint8_t reg)
{
  uint8_t reg_add = reg;
  uint8_t reg_dat = 0;
  writeCont(&reg_add, 1);
  readCont(&reg_dat, 1);
  
  return reg_dat;
}

uint8_t STK_ReadReg_1(uint8_t reg, uint8_t *p_dat)
{
  uint8_t err = 0;
  uint8_t reg_add = reg;
//  uint8_t reg_dat = 0;
  err = writeCont(&reg_add, 1);
  if(err)
  {
    return err;
  }
  err = readCont(p_dat, 1);
  return err;
}



//user

void STK_en_light(void)
{
  STK_WriteReg(0x00, 0x02); //en light 
}

void STK_den_light(void)
{
  STK_WriteReg(0x00, 0x00); //den light 
}

void STK_init(void)
{
  stk3311_iic_init(); //init i2c bus
  STK_en_light();     //en check light
}

uint8_t STK_light_read(void)
{
  uint8_t reg = 0;
  uint8_t data = 0;
  uint8_t err = 0;
  
//  uint16_t *p_dat = &t_stk3311_litht.light_value;
  
  reg = 0x13; //reg  DATA_ALS_H
  err = STK_ReadReg_1(reg, &data);
  if(err == STK3311_IIC_ERR)
  {
    return STK3311_IIC_ERR;
  }
  
  t_stk3311_litht.light_value = data;
  
  reg = 0x14; //reg  DATA_ALS_L
  err = STK_ReadReg_1(0x14, &data);
  if(err == STK3311_IIC_ERR)
  {
    return STK3311_IIC_ERR;
  }
  t_stk3311_litht.light_value <<= 8;
  t_stk3311_litht.light_value |= data;
  
  t_stk3311_litht.check_flag = STK3311_IIC_FLAG_ON;
  
  StkI2cLog("stk3311_test light=%d\r\n",t_stk3311_litht.light_value);
  
  return STK3311_IIC_ERR;
}


