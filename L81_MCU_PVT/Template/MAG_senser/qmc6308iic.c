

#include <stdio.h>
#include "qmc6308iic.h"

//#include "i2c_inf.h"
#include "L81_AT.h"
#include "L81_inf.h"
//#include "L81_FMC.h"

#define QMC_IIC_DEBUG_EN            0     //debug EN=1 DEN=0
#if QMC_IIC_DEBUG_EN
#define QmcI2cLog(...) printf(__VA_ARGS__)
#else
#define QmcI2cLog(...) 
#endif

// address 1d
//#define STK3311_ADDRESS_WRITE 0x90
//#define STK3311_ADDRESS_READ 0x91
#define QMC6308_ADDRESS7    (0x2C<<1)

uint32_t qmc_block_count = 0;

static void rcu_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(QMC6308_IIC_PIN_CLOCK);
    /* enable QMC6308_IIC_BUS_NUM clock */
    rcu_periph_clock_enable(QMC6308_IIC_PERIPH_CLOCK);
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
    gpio_af_set(QMC6308_IIC_SCK_PORT, QMC6308_IIC_SCK_AF, QMC6308_IIC_SCK_PIN);
    /* connect PB11 to I2C2_SDA */
    gpio_af_set(QMC6308_IIC_SDA_PORT, QMC6308_IIC_SDA_AF, QMC6308_IIC_SDA_PIN);
    /* configure GPIO pins of QMC6308_IIC_BUS_NUM */
    gpio_mode_set(QMC6308_IIC_SCK_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, QMC6308_IIC_SCK_PIN);//GPIO_PUPD_PULLDOWN GPIO_PUPD_PULLUP
    gpio_output_options_set(QMC6308_IIC_SCK_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, QMC6308_IIC_SCK_PIN);//GPIO_OTYPE_PP GPIO_OTYPE_OD
    gpio_mode_set(QMC6308_IIC_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, QMC6308_IIC_SDA_PIN);
    gpio_output_options_set(QMC6308_IIC_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, QMC6308_IIC_SDA_PIN);
  
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
//  i2c_timing_config(QMC6308_IIC_BUS_NUM, 0, 0x3, 0);
//  i2c_master_clock_config(QMC6308_IIC_BUS_NUM, 0x13, 0x36);
//  i2c_disable(QMC6308_IIC_BUS_NUM);
  i2c_timing_config(QMC6308_IIC_BUS_NUM, 0, 0x3, 0);
  i2c_master_clock_config(QMC6308_IIC_BUS_NUM, 0x33, 0x36); 
  /* enable I2C1 */
  i2c_enable(QMC6308_IIC_BUS_NUM);	
}


static uint8_t writeCont(uint8_t *p_dat, uint8_t len)
{
  
  if(p_dat == NULL)
  {
    return QMC6308_IIC_ERR;
  }
  
  __disable_irq();
  i2c_address_config(QMC6308_IIC_BUS_NUM, QMC6308_ADDRESS7, I2C_ADDFORMAT_7BITS);
  /* configure slave address */
  i2c_master_addressing(QMC6308_IIC_BUS_NUM, QMC6308_ADDRESS7, I2C_MASTER_TRANSMIT);
  /* configure number of bytes to be transferred */
  i2c_transfer_byte_number_config(QMC6308_IIC_BUS_NUM, len);
  
  
  /* wait until I2C bus is idle */
  qmc_block_count = 0;
  while(i2c_flag_get(QMC6308_IIC_BUS_NUM, I2C_FLAG_I2CBSY))
  {
    qmc_block_count ++;
    if(qmc_block_count >= QMC6308_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMC6308_IIC_ERR;
    }
  }
  /* send a start condition to I2C bus */
  i2c_start_on_bus(QMC6308_IIC_BUS_NUM);

  /* wait until the transmit data buffer is empty */
  I2C_STAT(QMC6308_IIC_BUS_NUM) |= I2C_STAT_TBE;
  qmc_block_count = 0;
  while(!i2c_flag_get(QMC6308_IIC_BUS_NUM, I2C_FLAG_TBE))
  {
    qmc_block_count ++;
    if(qmc_block_count >= QMC6308_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMC6308_IIC_ERR;
    }
  }
  for(uint8_t i=0; i<len; i++)
  {
    /* data transmission */
    i2c_data_transmit(QMC6308_IIC_BUS_NUM, p_dat[i]);
    /* wait until the TI bit is set */
    qmc_block_count = 0;
    while(!i2c_flag_get(QMC6308_IIC_BUS_NUM, I2C_FLAG_TBE))
    {
      qmc_block_count ++;
      if(qmc_block_count >= QMC6308_IIC_MAX_BLOCK)
      {
        __enable_irq();
        return QMC6308_IIC_ERR;
      }
    }
  }
	
  /* wait for transfer complete flag */
  qmc_block_count = 0;
  while(!i2c_flag_get(QMC6308_IIC_BUS_NUM, I2C_FLAG_TC))
  {
    qmc_block_count ++;
    if(qmc_block_count >= QMC6308_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMC6308_IIC_ERR;
    }
  }
  /* send a stop condition to I2C bus */
  i2c_stop_on_bus(QMC6308_IIC_BUS_NUM);
  /* wait until stop condition generate */
  qmc_block_count = 0;
  while(!i2c_flag_get(QMC6308_IIC_BUS_NUM, I2C_FLAG_STPDET))
  {
    qmc_block_count ++;
    if(qmc_block_count >= QMC6308_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMC6308_IIC_ERR;
    }
  }
  /* clear the STPDET bit */
  i2c_flag_clear(QMC6308_IIC_BUS_NUM, I2C_FLAG_STPDET);
  
  
  __enable_irq();
  return QMC6308_IIC_OK;
}

static uint8_t readCont(uint8_t *p_dat, uint8_t len)
{
  
  if(p_dat == NULL)
  {
    return QMC6308_IIC_ERR;
  }
  
  __disable_irq();
  i2c_address_config(QMC6308_IIC_BUS_NUM, QMC6308_ADDRESS7, I2C_ADDFORMAT_7BITS);
  i2c_master_addressing(QMC6308_IIC_BUS_NUM, QMC6308_ADDRESS7, I2C_MASTER_RECEIVE);
  i2c_transfer_byte_number_config(QMC6308_IIC_BUS_NUM, len);
  
  /* wait until I2C bus is idle */
  qmc_block_count = 0;
  while(i2c_flag_get(QMC6308_IIC_BUS_NUM, I2C_FLAG_I2CBSY))
  {
    qmc_block_count ++;
    if(qmc_block_count >= QMC6308_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMC6308_IIC_ERR;
    }
  }
  /* send a start condition to I2C bus */
  i2c_start_on_bus(QMC6308_IIC_BUS_NUM);
  for(uint8_t i = 0; i < len; i++) 
  {
      /* wait until the RBNE bit is set */
    qmc_block_count = 0;
    while(!i2c_flag_get(QMC6308_IIC_BUS_NUM, I2C_FLAG_RBNE))
    {
      qmc_block_count ++;
      if(qmc_block_count >= QMC6308_IIC_MAX_BLOCK)
      {
        __enable_irq();
        return QMC6308_IIC_ERR;
      }
    }
    /* read a data from I2C_DATA */
    p_dat[i] = i2c_data_receive(QMC6308_IIC_BUS_NUM);
  }
  qmc_block_count = 0;
  while(!i2c_flag_get(QMC6308_IIC_BUS_NUM, I2C_FLAG_TC))
  {
    qmc_block_count ++;
    if(qmc_block_count >= QMC6308_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMC6308_IIC_ERR;
    }
  }
  /* send a stop condition to I2C bus */
  i2c_stop_on_bus(QMC6308_IIC_BUS_NUM);
  /* wait until stop condition generate */
  qmc_block_count = 0;
  while(!i2c_flag_get(QMC6308_IIC_BUS_NUM, I2C_FLAG_STPDET))
  {
    qmc_block_count ++;
    if(qmc_block_count >= QMC6308_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return QMC6308_IIC_ERR;
    }
  }
  /* clear the STPDET bit */
  i2c_flag_clear(QMC6308_IIC_BUS_NUM, I2C_FLAG_STPDET);
  
  __enable_irq();
  return QMC6308_IIC_OK;
}




void qmc6308_iic_init(void) 
{    
  i2c_deinit(QMC6308_IIC_BUS_NUM);
  rcu_config();
  i2c_gpio_config();
  i2c_config();
}


/**/


//write reg cmd-1BYT
uint8_t QMC_WriteReg(uint8_t reg, uint8_t data)
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


uint8_t QMC_ReadReg(uint8_t reg, uint8_t *p_dat)
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


uint8_t QMC_read_id(void)
{
  uint8_t id = 0;
  uint8_t err = 0;
  err = QMC_ReadReg(QMC_REG_ID, &id);
  
  QmcI2cLog("QMC ID=%02X err=%d\r\n",id, err);
  return id;
}
typedef struct
{
  float x;
  float y;
  float z;
}QMC_FLOAT_DATA_TYPDEF;
QMC_FLOAT_DATA_TYPDEF qmc_data;

void QMC_read_data()
{
	uint8_t data_h = 0;
	uint8_t data_l = 0;
	
	qmc_data.x = 0;
	
	
	
	
}


//
uint8_t l81_AT_qmcr_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 1U){  //
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
	
	uint32_t cmd = String2Int(param[0]);
		
	if ((cmd < 0U) || (cmd > 1u)) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,err cmd value eg:[0~1]\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
  //
  switch (cmd)
  {
    case 0://read id
    {
      qmc6308_iic_init();
      uint8_t id = QMC_read_id();
      
      if(id == QMC6308_ID)
      {
        printf("AT+RES,ACK\r\n");
        printf("AT+RES,0x%02X\r\n",QMC6308_ID);
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
    case 1://read value
      printf("AT+RES,ACK\r\n");
      printf("AT+RES,x,y,z\r\n");
      printf("AT+RES,end\r\n");			
      break;
    default:
      break;
  }
    
    return 1U;	
}

/**/

