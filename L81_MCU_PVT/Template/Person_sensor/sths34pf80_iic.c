

#include <stdio.h>
#include "sths34pf80_iic.h"
#include "systick.h"

#define STHS34PF80_IIC_DEBUG_EN            0     //debug EN=1 DEN=0
#if STHS34PF80_IIC_DEBUG_EN
#define Sths34pf80I2cLog(...) printf(__VA_ARGS__)
#else
#define Sths34pf80I2cLog(...) 
#endif


#define STHS34_HARD_CHECK_PERSON          1 //1=hard 0=soft

// address 1d
//#define STK3311_ADDRESS_WRITE 0x90
//#define STK3311_ADDRESS_READ 0x91
#define STHS34_ADDRESS7    (0x5A<<1)

T_STH34_PERSON_DATA_TYPDEF t_sth32_person_data = {0};
T_STHS34PF80_FLAG_TYPDEF t_sths34pf80_flag = {0};


uint32_t STHS34_block_count = 0;




static void rcu_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(STHS34_IIC_PIN_CLOCK);
    /* enable STHS34_IIC_BUS_NUM clock */
    rcu_periph_clock_enable(STHS34_IIC_PERIPH_CLOCK);
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
    gpio_af_set(STHS34_IIC_SCK_PORT, STHS34_IIC_SCK_AF, STHS34_IIC_SCK_PIN);
    /* connect PB11 to I2C2_SDA */
    gpio_af_set(STHS34_IIC_SDA_PORT, STHS34_IIC_SDA_AF, STHS34_IIC_SDA_PIN);
    /* configure GPIO pins of STHS34_IIC_BUS_NUM */
    gpio_mode_set(STHS34_IIC_SCK_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, STHS34_IIC_SCK_PIN);
    gpio_output_options_set(STHS34_IIC_SCK_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, STHS34_IIC_SCK_PIN);
    gpio_mode_set(STHS34_IIC_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, STHS34_IIC_SDA_PIN);
    gpio_output_options_set(STHS34_IIC_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, STHS34_IIC_SDA_PIN);
  
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
//  i2c_timing_config(STHS34_IIC_BUS_NUM, 0, 0x3, 0);
//  i2c_master_clock_config(STHS34_IIC_BUS_NUM, 0x13, 0x36);
  
  i2c_timing_config(STHS34_IIC_BUS_NUM, 0, 0x3, 0);
  i2c_master_clock_config(STHS34_IIC_BUS_NUM, 0x33, 0x36); 
  /* enable I2C1 */
  i2c_enable(STHS34_IIC_BUS_NUM);	
}


static uint8_t writeCont(uint8_t *p_dat, uint8_t len)
{
  
  if(p_dat == NULL)
  {
    return STHS34_IIC_ERR;
  }
  
  __disable_irq();
  i2c_address_config(STHS34_IIC_BUS_NUM, STHS34_ADDRESS7, I2C_ADDFORMAT_7BITS);
  /* configure slave address */
  i2c_master_addressing(STHS34_IIC_BUS_NUM, STHS34_ADDRESS7, I2C_MASTER_TRANSMIT);
  /* configure number of bytes to be transferred */
  i2c_transfer_byte_number_config(STHS34_IIC_BUS_NUM, len);
  
  
  /* wait until I2C bus is idle */
  STHS34_block_count = 0;
  while(i2c_flag_get(STHS34_IIC_BUS_NUM, I2C_FLAG_I2CBSY))
  {
    STHS34_block_count ++;
    if(STHS34_block_count >= STHS34_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STHS34_IIC_ERR;
    }
  }
  /* send a start condition to I2C bus */
  i2c_start_on_bus(STHS34_IIC_BUS_NUM);

  /* wait until the transmit data buffer is empty */
  I2C_STAT(STHS34_IIC_BUS_NUM) |= I2C_STAT_TBE;
  STHS34_block_count = 0;
  while(!i2c_flag_get(STHS34_IIC_BUS_NUM, I2C_FLAG_TBE))
  {
    STHS34_block_count ++;
    if(STHS34_block_count >= STHS34_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STHS34_IIC_ERR;
    }
  }
  for(uint8_t i=0; i<len; i++)
  {
    /* data transmission */
    i2c_data_transmit(STHS34_IIC_BUS_NUM, p_dat[i]);
    /* wait until the TI bit is set */
    STHS34_block_count = 0;
    while(!i2c_flag_get(STHS34_IIC_BUS_NUM, I2C_FLAG_TBE))
    {
      STHS34_block_count ++;
      if(STHS34_block_count >= STHS34_IIC_MAX_BLOCK)
      {
        __enable_irq();
        return STHS34_IIC_ERR;
      }
    }
  }
	
  /* wait for transfer complete flag */
  STHS34_block_count = 0;
  while(!i2c_flag_get(STHS34_IIC_BUS_NUM, I2C_FLAG_TC))
  {
    STHS34_block_count ++;
    if(STHS34_block_count >= STHS34_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STHS34_IIC_ERR;
    }
  }
  /* send a stop condition to I2C bus */
  i2c_stop_on_bus(STHS34_IIC_BUS_NUM);
  /* wait until stop condition generate */
  STHS34_block_count = 0;
  while(!i2c_flag_get(STHS34_IIC_BUS_NUM, I2C_FLAG_STPDET))
  {
    STHS34_block_count ++;
    if(STHS34_block_count >= STHS34_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STHS34_IIC_ERR;
    }
  }
  /* clear the STPDET bit */
  i2c_flag_clear(STHS34_IIC_BUS_NUM, I2C_FLAG_STPDET);
  
  
  __enable_irq();
  return STHS34_IIC_OK;
}

static uint8_t readCont(uint8_t *p_dat, uint8_t len)
{
  
  if(p_dat == NULL)
  {
    return STHS34_IIC_ERR;
  }
  
  __disable_irq();
  i2c_address_config(STHS34_IIC_BUS_NUM, STHS34_ADDRESS7, I2C_ADDFORMAT_7BITS);
  i2c_master_addressing(STHS34_IIC_BUS_NUM, STHS34_ADDRESS7, I2C_MASTER_RECEIVE);
  i2c_transfer_byte_number_config(STHS34_IIC_BUS_NUM, len);
  
  /* wait until I2C bus is idle */
  STHS34_block_count = 0;
  while(i2c_flag_get(STHS34_IIC_BUS_NUM, I2C_FLAG_I2CBSY))
  {
    STHS34_block_count ++;
    if(STHS34_block_count >= STHS34_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STHS34_IIC_ERR;
    }
  }
  /* send a start condition to I2C bus */
  i2c_start_on_bus(STHS34_IIC_BUS_NUM);
  for(uint8_t i = 0; i < len; i++) 
  {
      /* wait until the RBNE bit is set */
    STHS34_block_count = 0;
    while(!i2c_flag_get(STHS34_IIC_BUS_NUM, I2C_FLAG_RBNE))
    {
      STHS34_block_count ++;
      if(STHS34_block_count >= STHS34_IIC_MAX_BLOCK)
      {
        __enable_irq();
        return STHS34_IIC_ERR;
      }
    }
    /* read a data from I2C_DATA */
    p_dat[i] = i2c_data_receive(STHS34_IIC_BUS_NUM);
  }
  STHS34_block_count = 0;
  while(!i2c_flag_get(STHS34_IIC_BUS_NUM, I2C_FLAG_TC))
  {
    STHS34_block_count ++;
    if(STHS34_block_count >= STHS34_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STHS34_IIC_ERR;
    }
  }
  /* send a stop condition to I2C bus */
  i2c_stop_on_bus(STHS34_IIC_BUS_NUM);
  /* wait until stop condition generate */
  STHS34_block_count = 0;
  while(!i2c_flag_get(STHS34_IIC_BUS_NUM, I2C_FLAG_STPDET))
  {
    STHS34_block_count ++;
    if(STHS34_block_count >= STHS34_IIC_MAX_BLOCK)
    {
      __enable_irq();
      return STHS34_IIC_ERR;
    }
  }
  /* clear the STPDET bit */
  i2c_flag_clear(STHS34_IIC_BUS_NUM, I2C_FLAG_STPDET);
  
  __enable_irq();
  return STHS34_IIC_OK;
}




void sths34_iic_init(void) 
{                     
  rcu_config();
  i2c_gpio_config();
  i2c_config();
}



//write reg cmd-1BYT
uint8_t STHS_WriteReg(uint8_t reg, uint8_t data)
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
uint8_t STHS_ReadReg(uint8_t reg)
{
  uint8_t reg_add = reg;
  uint8_t reg_dat = 0;
  writeCont(&reg_add, 1);
  readCont(&reg_dat, 1);
  
  return reg_dat;
}

uint8_t STHS_ReadReg_1(uint8_t reg, uint8_t *p_dat)
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



uint8_t STHS_writ_func(uint8_t reg, uint8_t *pdat)
{
  uint8_t err = 0;
  
  //reg check 
  
  err = STHS_WriteReg(0x21, 0x10);  //en access to the embedded; note: boot_mask 0x80,func_cfg_acc_mask 0x10, one_shot_mask 0x01
//  Sths34pf80I2cLog("w_func w_err=%d reg=%02X data=%02X\r\n",err, 0x21, 0x10);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  
  err = STHS_WriteReg(0x11, 0x40);  //Select write operation mode; func_cfg_w_mask 0x40,func_cfg_rmask 0x20;
//  Sths34pf80I2cLog("w_func w_err=%d reg=%02X data=%02X\r\n",err, 0x11, 0x40);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  err = STHS_WriteReg(0x08, reg);   //Set address
//  Sths34pf80I2cLog("w_func w_err=%d reg=%02X data=%02X\r\n",err, 0x08, reg);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  err = STHS_WriteReg(0x09, *pdat); //Set value to be written
//  Sths34pf80I2cLog("w_func w_err=%d reg=%02X data=%02X\r\n",err, 0x09, *pdat);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  err = STHS_WriteReg(0x11, 0x00);  //Disable write operation
//  Sths34pf80I2cLog("w_func w_err=%d reg=%02X data=%02X\r\n",err, 0x11, 0x00);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  err = STHS_WriteReg(0x21, 0x00);  //Disable access to the embedded functions registers
//  Sths34pf80I2cLog("w_func w_err=%d reg=%02X data=%02X\r\n",err, 0x21, 0x00);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  return err;
}

uint8_t STHS_read_func(uint8_t reg, uint8_t *pdat)
{
  uint8_t err = 0;
  
  //reg check 
  
  err = STHS_WriteReg(0x21, 0x10);  //en access to the embedded; note: boot_mask 0x80,func_cfg_acc_mask 0x10, one_shot_mask 0x01
//  Sths34pf80I2cLog("w_func w_err=%d reg=%02X data=%02X\r\n",err, 0x21, 0x10);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  err = STHS_WriteReg(0x11, 0x20);  //Select write operation mode; func_cfg_w_mask 0x40,func_cfg_rmask 0x20;
//  Sths34pf80I2cLog("w_func w_err=%d reg=%02X data=%02X\r\n",err, 0x11, 0x20);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  err = STHS_WriteReg(0x08, reg);   //Set address
//  Sths34pf80I2cLog("w_func w_err=%d reg=%02X data=%02X\r\n",err, 0x08, reg);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  err = STHS_ReadReg_1(0x09, pdat); //Set value to be written
//  Sths34pf80I2cLog("w_func r_err=%d reg=%02X data=%02X\r\n",err, 0x09, *pdat);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  err = STHS_WriteReg(0x11, 0x00);  //Disable write operation
//  Sths34pf80I2cLog("w_func w_err=%d reg=%02X data=%02X\r\n",err, 0x11, 0x00);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  err = STHS_WriteReg(0x21, 0x00);  //Disable access to the embedded functions registers
//  Sths34pf80I2cLog("w_func w_err=%d reg=%02X data=%02X\r\n",err, 0x21, 0x00);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  return err;
}



void STHS_en_check(void)
{
  uint8_t reg=0x20;
  uint8_t dat = 0x18;//0x18;  en and 33ms
  uint8_t err=0;

  err = STHS_WriteReg(reg, dat);
  Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
}


void STHS_den_check(void)
{
  uint8_t reg=0x20;
  uint8_t dat = 0x00;//0x18; 
  uint8_t err = 0;
  
  err = STHS_ReadReg_1(reg, &dat);
  Sths34pf80I2cLog("sths_test r_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
  
  dat &= 0x0f;
  err = STHS_WriteReg(reg, dat);
  Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
}

void SHHS_boot_data(void)
{
  uint8_t reg=0x21;
  uint8_t dat = 0x00;//0x80;
  uint8_t err=0;
  err = STHS_ReadReg_1(reg, &dat);
  Sths34pf80I2cLog("sths_test r_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
  
  dat |= 0x80;//boot cmd
  err = STHS_WriteReg(reg, dat);
  Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
}


void SHHS34pf80_init(void)
{
  sths34_iic_init();
  
  uint8_t reg=0x20;
  uint8_t dat = 0x18;//0x18;  start
  uint8_t err=0;


  {
    reg = 0x1d; 
    err = STHS_ReadReg_1(reg,&dat);
    if(dat == 0)
    {
      dat = 0xde;
      err = STHS_WriteReg(reg, dat);
    }
    reg = 0x20; dat = 0x00; //Disable ODR
    err = STHS_WriteReg(reg, dat);
    delay_1ms(20);
    
    reg = 0x21; dat = 0x10;
    err = STHS_WriteReg(reg, dat);
    delay_1ms(200);
    reg = 0x11; dat = 0x40;
    err = STHS_WriteReg(reg, dat);
    delay_1ms(200);
    reg = 0x08; dat = 0x28;
    err = STHS_WriteReg(reg, dat);
    delay_1ms(200);
    reg = 0x09; dat = 0x04;//(enable ? 0x04 : 0x00);
    err = STHS_WriteReg(reg, dat);
    delay_1ms(200);
    reg = 0x09; dat = 0x00;
    err = STHS_WriteReg(reg, dat);
    delay_1ms(200);
    reg = 0x11; dat = 0x00;
    err = STHS_WriteReg(reg, dat);
    delay_1ms(200);
    reg = 0x21; dat = 0x00;
    err = STHS_WriteReg(reg, dat);
    delay_1ms(200);
    
    reg = 0x21; dat = 0x10;
    err = STHS_WriteReg(reg, dat);
    reg = 0x08; dat = 0x2a;
    err = STHS_WriteReg(reg, dat);
    reg = 0x09; dat = 0x01;
    err = STHS_WriteReg(reg, dat);
    reg = 0x21; dat = 0x00;
    err = STHS_WriteReg(reg, dat);
  }
  {

    reg=0x0c; dat = 0x07; //
    err = STHS_WriteReg(reg, dat);
    Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
    

    reg=0x0d; dat = 0x71; //
    err = STHS_WriteReg(reg, dat);
    Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
    
   
    reg=0x10; dat = 0x02; //
    err = STHS_WriteReg(reg, dat);
    Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
    

  }

  { //fun_cof
    reg = 0x20; dat = 0x2c; //300 person THS_L
    err = STHS_writ_func(reg, &dat);
    Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
    reg = 0x21; dat = 0x01; //300 person THS_H
    err = STHS_writ_func(reg, &dat);
    Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
    
    reg = 0x27; dat = 0xFA; //250  hyst_person
    err = STHS_writ_func(reg, &dat);
    Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
    
    
    reg = 0x22; dat = 0x2c; //300  motion THS_L
    err = STHS_writ_func(reg, &dat);
    Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
    reg = 0x23; dat = 0x01; //300   motion THS_H
    err = STHS_writ_func(reg, &dat);
    Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
    
    reg = 0x26; dat = 0xC8; //200  hyst_motion
    err = STHS_writ_func(reg, &dat);
    Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
    
    reg = 0x24; dat = 0x64; //100  tamb_shock_ths
    err = STHS_writ_func(reg, &dat);
    Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
  }
  
  STHS_en_check();      //en check
}



uint8_t SHHS34pf80_person_data_read(void)
{
  
  uint8_t dat = 0;
  uint8_t err = 0;
  
  T_STH34_PERSON_DATA_TYPDEF *pt_data = &t_sth32_person_data;
  
  
  err = STHS_ReadReg_1(STHS34_REG_TOBJECT_H  , &dat);
  pt_data->tobject = dat;
  pt_data->tobject <<= 8;
  err = STHS_ReadReg_1(STHS34_REG_TOBJECT_L  , &dat);
  pt_data->tobject |= dat;
  
  err = STHS_ReadReg_1(STHS34_REG_TAMBIENT_H , &dat);
  pt_data->tambient = dat;
  pt_data->tambient <<= 8;
  err = STHS_ReadReg_1(STHS34_REG_TAMBIENT_L , &dat);
  pt_data->tambient |= dat;
  
  err = STHS_ReadReg_1(STHS34_REG_TPRESENCE_H, &dat);
  pt_data->tperson = dat;
  pt_data->tperson <<= 8;
  err = STHS_ReadReg_1(STHS34_REG_TPRESENCE_L, &dat);
  pt_data->tperson |= dat;
  
  err = STHS_ReadReg_1(STHS34_REG_TMOTION_H  , &dat);
  pt_data->tmotion = dat;
  pt_data->tmotion <<= 8;
  err = STHS_ReadReg_1(STHS34_REG_TMOTION_L  , &dat);
  pt_data->tmotion |= dat;
  

  pt_data->read_flag = STHS34_IIC_FLAG_ON;
  
  Sths34pf80I2cLog("sths_obj=%02X(%d)\r\n",pt_data->tobject,pt_data->tobject);
  Sths34pf80I2cLog("sths_tam=%02X(%d)\r\n",pt_data->tambient,pt_data->tambient);
  Sths34pf80I2cLog("sths_per=%02X(%d)\r\n",pt_data->tperson,pt_data->tperson);
  Sths34pf80I2cLog("sths_tmo=%02X(%d)\r\n",pt_data->tmotion,pt_data->tmotion);
  
  return err;
  
}

uint8_t SHHS34pf80_person_flag_read(void)
{
//  uint8_t reg = 0x25;
  uint8_t dat = 0;
  uint8_t err = 0;
  
  err = STHS_ReadReg_1(STHS34_REG_FUNC_STATUS, &dat);
  if(err == STHS34_IIC_ERR)
  {
    return STHS34_IIC_ERR;
  }
  
  t_sths34pf80_flag.check_flag      = STHS34_IIC_FLAG_ON;
  t_sths34pf80_flag.pres_flag       = (dat&0x04)>0?STHS34_IIC_FLAG_ON:STHS34_IIC_FLAG_OFF;
  t_sths34pf80_flag.mot_flag        = (dat&0x02)>0?STHS34_IIC_FLAG_ON:STHS34_IIC_FLAG_OFF;
  t_sths34pf80_flag.tamb_shock_flag = (dat&0x01)>0?STHS34_IIC_FLAG_ON:STHS34_IIC_FLAG_OFF;
  
  
  Sths34pf80I2cLog("sths_test_flag r_err=%d reg=%02X    data=%02X p=%d m=%d ta=%d\r\n",err, reg, dat, 
                  t_sths34pf80_flag.pres_flag, 
                  t_sths34pf80_flag.mot_flag , 
                  t_sths34pf80_flag.tamb_shock_flag);

//  #if STHS34PF80_IIC_DEBUG_EN
  SHHS34pf80_person_data_read();
  
//  #endif

  
  return STHS34_IIC_OK;
}

//200ms Periodic callback 
//uint8_t SHHS34pf80_person_soft_read(void)
//{
//  int32_t last_per = 
//  SHHS34pf80_person_data_read();
//  
//  
//}


//test
#if 0 
void delay_test(uint32_t delay_num)
{
  for(uint32_t i=0; i<delay_num; i++);
}

void sths_test_reg_read(void)
{
  uint8_t err = 0;
  uint8_t reg = 0x20;
  uint8_t dat = 0x18;
  
  uint8_t p=0;
  uint8_t m=0;
  uint8_t ta=0;
  
  int16_t per_data = 0;
  
  //write
  
//  err = STHS_WriteReg(reg, dat);
//  Sths34pf80I2cLog("sths_test w_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
  
  #if 1
  
  //p flag
  reg=0x25;
  err = STHS_ReadReg_1(reg, &dat);
  p = (dat&0x04)>0?1:0;
  m = (dat&0x02)>0?1:0;
  ta = (dat&0x01)>0?1:0;
  Sths34pf80I2cLog("sths_test_flag r_err=%d reg=%02X    data=%02X p=%d m=%d ta=%d\r\n",err, reg, dat, p, m, ta);
  
  


  //objec data
  
  reg = 0x27; //MSB
  err = STHS_ReadReg_1(reg, &dat);
  
  per_data = dat;
  per_data <<= 8;
  Sths34pf80I2cLog("sths_test_obj r_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
  
  reg = 0x26; //LSB
  err = STHS_ReadReg_1(reg, &dat);
  per_data |= dat;
  Sths34pf80I2cLog("sths_test_obj r_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
  
  Sths34pf80I2cLog("sths_obj int=%d hex=%02X \r\n",per_data, per_data);
  
  //p data
  
  reg = 0x3B; //MSB
  err = STHS_ReadReg_1(reg, &dat);
  
  per_data = dat;
  per_data <<= 8;
  Sths34pf80I2cLog("sths_test_dat r_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
  
  reg = 0x3A; //LSB
  err = STHS_ReadReg_1(reg, &dat);
  per_data |= dat;
  Sths34pf80I2cLog("sths_test_dat r_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
  
  Sths34pf80I2cLog("sths_data int=%d hex=%02X \r\n",per_data, per_data);
  
  //temp
  reg = 0x29; //MSB
  err = STHS_ReadReg_1(reg, &dat);
  
  per_data = dat;
  per_data <<= 8;
//  Sths34pf80I2cLog("sths_test_temp r_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
  
  reg = 0x28; //LSB
  err = STHS_ReadReg_1(reg, &dat);
  per_data |= dat;
//  Sths34pf80I2cLog("sths_test_temp r_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
  
  Sths34pf80I2cLog("sths_temp int=%d hex=%02X \r\n",per_data, per_data);
  
  
  //p comp data
  reg = 0x21; //LSB
  err = STHS_read_func(reg, &dat);
  per_data = dat;
  per_data <<= 8;
  Sths34pf80I2cLog("sths_test_fun r_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
  
  reg = 0x20; //MSB
  err = STHS_read_func(reg, &dat);
  per_data |= dat;
  Sths34pf80I2cLog("sths_test_fun r_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
  
//  per_data = (per_data & 0x7fff);
  Sths34pf80I2cLog("sths_comp int=%d hex=%02X \r\n",per_data, per_data);
  
  
  
  
  
  #else
  
  //read reg
  for(uint8_t i=0x0c; i<=0x0d; i++)
  {
    err = STHS_ReadReg_1(i, &dat);
    Sths34pf80I2cLog("sths_test r_err=%d reg=%02X data=%02X\r\n",err, i, dat);
//    delay_test(10000);
  }
  for(uint8_t i=0x0f; i<=0x10; i++)
  {
    err = STHS_ReadReg_1(i, &dat);
    Sths34pf80I2cLog("sths_test r_err=%d reg=%02X data=%02X\r\n",err, i, dat);
//    delay_test(10000);
  }
  
  for(uint8_t i=0x20; i<=0x23; i++)
  {
    err = STHS_ReadReg_1(i, &dat);
    Sths34pf80I2cLog("sths_test r_err=%d reg=%02X data=%02X\r\n",err, i, dat);
//    delay_test(10000);
  }
  
  for(uint8_t i=0x25; i<=0x29; i++)
  {
    err = STHS_ReadReg_1(i, &dat);
    if(i==0x25)
    {
      p = (dat&0x04)>0?1:0;
      m = (dat&0x02)>0?1:0;
      ta = (dat&0x01)>0?1:0;
      Sths34pf80I2cLog("sths_test r_err=%d reg=%02X    data=%02X p=%d m=%d ta=%d\r\n",err, i, dat, p, m, ta);
    }
    else
    {
      Sths34pf80I2cLog("sths_test r_err=%d reg=%02X data=%02X\r\n",err, i, dat);
    }
    
//    delay_test(10000);
  }
  
  for(uint8_t i=0x3A; i<=0x3F; i++)
  {
    err = STHS_ReadReg_1(i, &dat);
    Sths34pf80I2cLog("sths_test r_err=%d reg=%02X     data=%02X \r\n",err, i, dat);
//    delay_test(10000);
  }
  
  for(uint8_t i=0x20; i<=0x29; i++)
  {
//    err = STHS_ReadReg_1(i, &dat);
    err = STHS_read_func(i, &dat);
    Sths34pf80I2cLog("sths_test r_err=%d reg=%02X     --data=%02X \r\n",err, i, dat);
//    delay_test(10000);
  }
  #endif
  
//  { //read-test
//  reg = 0x0f;
//  err = STHS_ReadReg_1(reg, &dat);
//  Sths34pf80I2cLog("sths_test r_err=%d reg=%02X data=%02X\r\n",err, reg, dat);
//  }
  
//  dat = STHS_ReadReg(reg);
  
  Sths34pf80I2cLog("\r\n");
  
}
#endif




