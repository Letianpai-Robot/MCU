

#include <stdio.h>
#include "qmi8658iic.h"

#define QMI_IIC_DEBUG_EN            0     //debug EN=1 DEN=0
#if QMI_IIC_DEBUG_EN
#define QmiI2cLog(...) printf(__VA_ARGS__)
#else
#define QmiI2cLog(...) 
#endif

// address 1d
//#define QMI_ADDRESS_WRITE 0x90
//#define QMI_ADDRESS_READ 0x91
#define QMI_ADDRESS7    (0x6B<<1)


uint32_t qmi_block_count = 0;




static void rcu_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(QMI_IIC_PIN_CLOCK);
    /* enable QMI_IIC_BUS_NUM clock */
    rcu_periph_clock_enable(QMI_IIC_PERIPH_CLOCK);
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
    gpio_af_set(QMI_IIC_SCK_PORT, QMI_IIC_SCK_AF, QMI_IIC_SCK_PIN);
    /* connect PB11 to I2C2_SDA */
    gpio_af_set(QMI_IIC_SDA_PORT, QMI_IIC_SDA_AF, QMI_IIC_SDA_PIN);
    /* configure GPIO pins of QMI_IIC_BUS_NUM */
    gpio_mode_set(QMI_IIC_SCK_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, QMI_IIC_SCK_PIN);
    gpio_output_options_set(QMI_IIC_SCK_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, QMI_IIC_SCK_PIN);
    gpio_mode_set(QMI_IIC_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, QMI_IIC_SDA_PIN);
    gpio_output_options_set(QMI_IIC_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, QMI_IIC_SDA_PIN);
  
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
//  i2c_timing_config(QMI_IIC_BUS_NUM, 0, 0x3, 0);
//  i2c_master_clock_config(QMI_IIC_BUS_NUM, 0x13, 0x36);
  
  //400kHz
  i2c_timing_config(QMI_IIC_BUS_NUM, 0, 0x3, 0);
  i2c_master_clock_config(QMI_IIC_BUS_NUM, 0x0a, 0x20); 
  /* enable I2C1 */
  i2c_enable(QMI_IIC_BUS_NUM);	
}


static uint8_t writeCont(uint8_t *p_dat, uint8_t len)
{
  
  if(p_dat == NULL)
  {
    return QMI_IIC_ERR;
  }
  
  __disable_irq();
  i2c_address_config(QMI_IIC_BUS_NUM, QMI_ADDRESS7, I2C_ADDFORMAT_7BITS);
  /* configure slave address */
  i2c_master_addressing(QMI_IIC_BUS_NUM, QMI_ADDRESS7, I2C_MASTER_TRANSMIT);
  /* configure number of bytes to be transferred */
  i2c_transfer_byte_number_config(QMI_IIC_BUS_NUM, len);
  
  
  /* wait until I2C bus is idle */
  qmi_block_count = 0;
  while(i2c_flag_get(QMI_IIC_BUS_NUM, I2C_FLAG_I2CBSY))
  {
    qmi_block_count ++;
    if(qmi_block_count >= QMI_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMI_IIC_ERR;
    }
  }
  /* send a start condition to I2C bus */
  i2c_start_on_bus(QMI_IIC_BUS_NUM);

  /* wait until the transmit data buffer is empty */
  I2C_STAT(QMI_IIC_BUS_NUM) |= I2C_STAT_TBE;
  qmi_block_count = 0;
  while(!i2c_flag_get(QMI_IIC_BUS_NUM, I2C_FLAG_TBE))
  {
    qmi_block_count ++;
    if(qmi_block_count >= QMI_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMI_IIC_ERR;
    }
  }
  for(uint8_t i=0; i<len; i++)
  {
    /* data transmission */
    i2c_data_transmit(QMI_IIC_BUS_NUM, p_dat[i]);
    /* wait until the TI bit is set */
    qmi_block_count = 0;
    while(!i2c_flag_get(QMI_IIC_BUS_NUM, I2C_FLAG_TBE))
    {
      qmi_block_count ++;
      if(qmi_block_count >= QMI_IIC_MAX_BLOCK)
      {
        __enable_irq();
        return QMI_IIC_ERR;
      }
    }
  }
	
  /* wait for transfer complete flag */
  qmi_block_count = 0;
  while(!i2c_flag_get(QMI_IIC_BUS_NUM, I2C_FLAG_TC))
  {
    qmi_block_count ++;
    if(qmi_block_count >= QMI_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMI_IIC_ERR;
    }
  }
  /* send a stop condition to I2C bus */
  i2c_stop_on_bus(QMI_IIC_BUS_NUM);
  /* wait until stop condition generate */
  qmi_block_count = 0;
  while(!i2c_flag_get(QMI_IIC_BUS_NUM, I2C_FLAG_STPDET))
  {
    qmi_block_count ++;
    if(qmi_block_count >= QMI_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMI_IIC_ERR;
    }
  }
  /* clear the STPDET bit */
  i2c_flag_clear(QMI_IIC_BUS_NUM, I2C_FLAG_STPDET);
  
  
  __enable_irq();
  return QMI_IIC_OK;
}

static uint8_t readCont(uint8_t *p_dat, uint8_t len)
{
  
  if(p_dat == NULL)
  {
    return QMI_IIC_ERR;
  }
  
  __disable_irq();
  i2c_address_config(QMI_IIC_BUS_NUM, QMI_ADDRESS7, I2C_ADDFORMAT_7BITS);
  i2c_master_addressing(QMI_IIC_BUS_NUM, QMI_ADDRESS7, I2C_MASTER_RECEIVE);
  i2c_transfer_byte_number_config(QMI_IIC_BUS_NUM, len);
  
  /* wait until I2C bus is idle */
  qmi_block_count = 0;
  while(i2c_flag_get(QMI_IIC_BUS_NUM, I2C_FLAG_I2CBSY))
  {
    qmi_block_count ++;
    if(qmi_block_count >= QMI_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMI_IIC_ERR;
    }
  }
  /* send a start condition to I2C bus */
  i2c_start_on_bus(QMI_IIC_BUS_NUM);
  for(uint8_t i = 0; i < len; i++) 
  {
      /* wait until the RBNE bit is set */
    qmi_block_count = 0;
    while(!i2c_flag_get(QMI_IIC_BUS_NUM, I2C_FLAG_RBNE))
    {
      qmi_block_count ++;
      if(qmi_block_count >= QMI_IIC_MAX_BLOCK)
      {
        __enable_irq();
        return QMI_IIC_ERR;
      }
    }
    /* read a data from I2C_DATA */
    p_dat[i] = i2c_data_receive(QMI_IIC_BUS_NUM);
  }
  qmi_block_count = 0;
  while(!i2c_flag_get(QMI_IIC_BUS_NUM, I2C_FLAG_TC))
  {
    qmi_block_count ++;
    if(qmi_block_count >= QMI_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMI_IIC_ERR;
    }
  }
  /* send a stop condition to I2C bus */
  i2c_stop_on_bus(QMI_IIC_BUS_NUM);
  /* wait until stop condition generate */
  qmi_block_count = 0;
  while(!i2c_flag_get(QMI_IIC_BUS_NUM, I2C_FLAG_STPDET))
  {
    qmi_block_count ++;
    if(qmi_block_count >= QMI_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMI_IIC_ERR;
    }
  }
  /* clear the STPDET bit */
  i2c_flag_clear(QMI_IIC_BUS_NUM, I2C_FLAG_STPDET);
  
  __enable_irq();
  return QMI_IIC_OK;
}





void QMI_iic_init(void) 
{                     
  rcu_config();
  i2c_gpio_config();
  i2c_config();
}



//write reg cmd-1BYT
uint8_t QMI_WriteReg(uint8_t reg, uint8_t data)
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
uint8_t QMI_ReadReg(uint8_t reg)
{
  uint8_t reg_add = reg;
  uint8_t reg_dat = 0;
  writeCont(&reg_add, 1);
  readCont(&reg_dat, 1);
  
  return reg_dat;
}

uint8_t QMI_ReadReg_1(uint8_t reg, uint8_t *p_dat)
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
uint8_t QMI_id_read(void)
{
  uint8_t reg = 0;
  uint8_t data = 0;
  uint8_t err = 0;
  
//  uint16_t *p_dat = &t_QMI_litht.light_value;
  
  reg = 0x00; //reg  DATA_ALS_H
  err = QMI_ReadReg_1(reg, &data);
  if(err == QMI_IIC_ERR)
  {
    return QMI_IIC_ERR;
  }
  
  QmiI2cLog("QMI_test id=%02x\r\n",data);
  
  return QMI_IIC_ERR;
}


