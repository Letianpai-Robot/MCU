/**
 ******************************************************************************
 * @file    STHS34PF80.c
 * @author  MEMS Software Solutions Team
 * @brief   STHS34PF80 driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "gd32l23x.h"
#include <stdio.h>
#include "sths34pf80.h"
#include "tmos_algo.h"
#include "systick.h"
#include "i2c_inf.h"

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
}i162u8_t;

#define TOBJ_HUMAN_RADIATION_THR	500
#define PRESENCE_THS 300
#define PRESENCE_HYS 200

#define MOTION_THS 300
#define MOTION_HYS 200

#define ENABLE_EMB  1
#define ENABLE_INT	0

#define ENABLE_TOBJ_COMP	1
STHS34PF80_CommonDrv_t STHS34PF80_COMMON_Driver =
{
  STHS34PF80_Init,
  STHS34PF80_DeInit,
  STHS34PF80_ReadID,
  STHS34PF80_GetCapabilities,
};

STHS34PF80_Drv_t STHS34PF80_Driver =
{
  STHS34PF80_Enable,
  STHS34PF80_Disable,
  STHS34PF80_GetSensitivity,
  STHS34PF80_GetOutputDataRate,
  STHS34PF80_SetOutputDataRate,
  STHS34PF80_GetFullScale,
  STHS34PF80_SetFullScale,
  STHS34PF80_GetAxes,
  STHS34PF80_GetAxesRaw,
};

/**
 * @}
 */

/** @defgroup STHS34PF80_Private_Function_Prototypes STHS34PF80 Private Function Prototypes
 * @{
 */


static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteRegWrap(void *Handle, uint8_t Reg,  uint8_t *pData, uint16_t Length);

//The slave address of the STHS34PF80 is SAD=1011010
#define STHS34PF80_SLAVE_ADDR (0x5A << 1)
unsigned char STHS34PF80_write_reg(uint16_t address, unsigned char reg, unsigned char value)
{
	unsigned char ret=0;
	unsigned int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		i2c_buffer_write(I2C2, address, reg, &value, 1);
	}
	return ret;
}

int32_t STHS34PF80_write_regs(uint16_t address, uint16_t reg, uint8_t *value, uint16_t len)
{
	int i, ret;

	for(i=0u; i < len; i++)
	{
		ret = STHS34PF80_write_reg(address, reg + i, value[i]);
	}

	return ret;
}

int32_t STHS34PF80_read_regs(uint16_t address, uint16_t reg, uint8_t* buf, uint16_t len)
{
	unsigned char ret=0;
	unsigned int retry = 0;
	
	uint16_t i = 0U;

	while((!ret) && (retry++ < 5))
	{
		for (i = 0U; i < len; i++){
		  i2c_buffer_read(I2C2, address, reg + i, buf + i, 1);
		}
	}
	return ret;
}

void STHS34PF80_IO_init(STHS34PF80_IO_t *pIO)
{
	pIO->Address = STHS34PF80_SLAVE_ADDR;
	pIO->ReadReg = STHS34PF80_read_regs;
	pIO->WriteReg = STHS34PF80_write_regs;
}

/**
 * @}
 */

/** @defgroup STHS34PF80_Exported_Functions STHS34PF80 Exported Functions
 * @{
 */

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t STHS34PF80_RegisterBusIO(STHS34PF80_Object_t *pObj, STHS34PF80_IO_t *pIO)
{
  int32_t ret = STHS34PF80_OK;

  if (pObj == NULL)
  {
    ret = STHS34PF80_ERROR;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.BusType   = pIO->BusType;
    pObj->IO.Address   = pIO->Address;
    pObj->IO.WriteReg  = pIO->WriteReg;
    pObj->IO.ReadReg   = pIO->ReadReg;
    pObj->IO.GetTick   = pIO->GetTick;

    pObj->Ctx.read_reg  = ReadRegWrap;
    pObj->Ctx.write_reg = WriteRegWrap;
    pObj->Ctx.handle   = pObj;

    if (pObj->IO.Init == NULL)
    {
      ret = STHS34PF80_ERROR;
    }
	else if (pObj->IO.Init() != STHS34PF80_OK)
    {
      ret = STHS34PF80_ERROR;
    }
  }

  return ret;
}


int32_t STHS34PF80_GetSensitivity(STHS34PF80_Object_t *pObj, float *Sensitivity)
{
	return STHS34PF80_OK;
}
int32_t STHS34PF80_GetFullScale(STHS34PF80_Object_t *pObj, int32_t *fs)
{
	return STHS34PF80_OK;
}
int32_t STHS34PF80_SetFullScale(STHS34PF80_Object_t *pObj, int32_t fs)
{
	return STHS34PF80_OK;
}
int32_t STHS34PF80_GetAxes(STHS34PF80_Object_t *pObj, STHS34PF80_Data_t * data)
{
	return STHS34PF80_OK;
}

int32_t STHS34PF80_Enable_Comp(stmdev_ctx_t *ctx, uint8_t enable)
{
  uint8_t reg_value;
  uint8_t reg_addr;
  int32_t ret;

  reg_addr = 0x1d; 
  ret = sths34pf80_read_reg(ctx, reg_addr, &reg_value, 1);
  if(reg_value == 0)
  {
    reg_value = 0xde;
    ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  }
  reg_addr = 0x20; reg_value = 0x00; //Disable ODR
  ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  delay_1ms(20);
  
  reg_addr = 0x21; reg_value = 0x10;
  ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  delay_1ms(200);
  reg_addr = 0x11; reg_value = 0x40;
  ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  delay_1ms(200);
  reg_addr = 0x08; reg_value = 0x28;
  ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  delay_1ms(200);
  reg_addr = 0x09; reg_value = (enable ? 0x04 : 0x00);
  ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  delay_1ms(200);
  reg_addr = 0x09; reg_value = 0x00;
  ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  delay_1ms(200);
  reg_addr = 0x11; reg_value = 0x00;
  ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  delay_1ms(200);
  reg_addr = 0x21; reg_value = 0x00;
  ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  delay_1ms(200);
  
  reg_addr = 0x21; reg_value = 0x10;
  ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  reg_addr = 0x08; reg_value = 0x2a;
  ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  reg_addr = 0x09; reg_value = 0x01;
  ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  reg_addr = 0x21; reg_value = 0x00;
  ret = sths34pf80_write_reg(ctx, reg_addr, &reg_value, 1);
  return ret;
}


/**
 * @brief  Initialize the STHS34PF80 sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t STHS34PF80_Init(STHS34PF80_Object_t *pObj)
{
  sths34pf80_tmos_odr_t reg_odr_val;
  sths34pf80_lpf_bandwidth_t lpf_bw;
  uint8_t reg_value;
  tmos_algo_sw_params_t params;
  
  sths34pf80_boot_set(&pObj->Ctx, 1);
  do{
	sths34pf80_boot_get(&pObj->Ctx, &reg_value);
  }while(reg_value == 1);
  delay_1ms(20);
  STHS34PF80_Enable_Comp(&pObj->Ctx, ENABLE_TOBJ_COMP); //Tobj Compensation - Enable: 1, Disable: 0
  
  lpf_bw = STHS34PF80_LPF_ODR_DIV_800;
  sths34pf80_lpf_m_bandwidth_set(&pObj->Ctx, lpf_bw); //motion BW
  lpf_bw = STHS34PF80_LPF_ODR_DIV_800;
  sths34pf80_lpf_p_bandwidth_set(&pObj->Ctx, lpf_bw); //PRESENCE BW
  lpf_bw = STHS34PF80_LPF_ODR_DIV_9;
  sths34pf80_lpf_p_m_bandwidth_set(&pObj->Ctx, lpf_bw); // PPRESENCE_MOTION BW
  lpf_bw = STHS34PF80_LPF_ODR_DIV_20;
  sths34pf80_lpf_a_t_bandwidth_set(&pObj->Ctx, lpf_bw);
  
  sths34pf80_avg_tobject_num_set(&pObj->Ctx,STHS34PF80_AVG_TMOS_32);
  sths34pf80_avg_tambient_num_set(&pObj->Ctx,STHS34PF80_AVG_T_8);
#if ENABLE_EMB
  uint8_t   embed_data[2];
  embed_data[0] = (PRESENCE_THS&0xFF); embed_data[1] = ((PRESENCE_THS&0xFF00) >> 8); //presence_THR = 300
  sths34pf80_func_cfg_write(&pObj->Ctx,STHS34PF80_PRESENCE_THS, embed_data, 2);
  embed_data[0] = PRESENCE_HYS;
  sths34pf80_func_cfg_write(&pObj->Ctx,STHS34PF80_HYST_PRESENCE, embed_data, 1); //Presence_HYS = 200

  embed_data[0] = (MOTION_THS&0xFF); embed_data[1] = ((MOTION_THS&0xFF00) >> 8); //MOTION_THR = 400
  sths34pf80_func_cfg_write(&pObj->Ctx,STHS34PF80_MOTION_THS, embed_data, 2);
  embed_data[0] = MOTION_HYS;
  sths34pf80_func_cfg_write(&pObj->Ctx,STHS34PF80_HYST_MOTION, embed_data, 1); //Motion_hys = 200

  embed_data[0] = 0x64; embed_data[1] = 0x00;
  sths34pf80_func_cfg_write(&pObj->Ctx,STHS34PF80_TAMB_SHOCK_THS, embed_data, 2);
#endif
#if ENABLE_INT
  sths34pf80_algo_config_t algo_conf;

  sths34pf80_pin_polarity_set(&pObj->Ctx,STHS34PF80_ACTIVE_HIGH);
  sths34pf80_int_pin_mode_set(&pObj->Ctx,STHS34PF80_PUSH_PULL);
  sths34pf80_tmos_int_set(&pObj->Ctx,STHS34PF80_TMOS_INT_MOTION_PRESENCE);
  sths34pf80_tmos_route_int_set(&pObj->Ctx,STHS34PF80_TMOS_INT_OR);
  
  sths34pf80_func_cfg_read(&pObj->Ctx,STHS34PF80_ALGO_CONFIG, (uint8_t*)&algo_conf, 1);
  algo_conf.int_pulsed = 1;
  sths34pf80_func_cfg_write(&pObj->Ctx,STHS34PF80_ALGO_CONFIG, (uint8_t*)&algo_conf, 1);
#endif

  sths34pf80_block_data_update_set(&pObj->Ctx,1);
  reg_odr_val = STHS34PF80_TMOS_ODR_AT_30Hz;
  sths34pf80_tmos_odr_set(&pObj->Ctx,reg_odr_val);

  if(reg_odr_val == STHS34PF80_TMOS_ODR_AT_30Hz)
  {
  	params.odr = 30; 
  }
  else if(reg_odr_val == STHS34PF80_TMOS_ODR_AT_8Hz)
  {
	params.odr = 8; 
  }
  else
  {
	params.odr = 4; 
  }
  params.presence_frozen_exit_thr = PRESENCE_THS;
  params.tobj_human_radiation_thr = TOBJ_HUMAN_RADIATION_THR;
  tmos_algo_sw_init(&params);
  
  return STHS34PF80_OK;
}

/**
 * @brief  Deinitialize the STTS751 sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t STHS34PF80_DeInit(STHS34PF80_Object_t *pObj)
{
  return STHS34PF80_OK;
}

/**
 * @brief  Get WHO_AM_I value
 * @param  pObj the device pObj
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
int32_t STHS34PF80_ReadID(STHS34PF80_Object_t *pObj, uint8_t *Id)
{
  if (sths34pf80_device_id_get(&(pObj->Ctx), Id) != STHS34PF80_OK)
  {
    return STHS34PF80_ERROR;
  }
	
  return STHS34PF80_OK;
}

/**
 * @brief  Get STHS34PF80 sensor capabilities
 * @param  pObj Component object pointer
 * @param  Capabilities pointer to STHS34PF80 sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t STHS34PF80_GetCapabilities(STHS34PF80_Object_t *pObj, STHS34PF80_Capabilities_t *Capabilities)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(pObj);
  Capabilities->tmos = 1;
  Capabilities->MaxOdr = 30.0f;
  return STHS34PF80_OK;
}

/**
 * @brief  Enable the STHS34PF80 temperature sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t STHS34PF80_Enable(STHS34PF80_Object_t *pObj)
{
  return STHS34PF80_OK;
}

/**
 * @brief  Disable the STHS34PF80 temperature sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t STHS34PF80_Disable(STHS34PF80_Object_t *pObj)
{
  return STHS34PF80_OK;
}

/**
 * @brief  Get the STHS34PF80 temperature sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t STHS34PF80_GetOutputDataRate(STHS34PF80_Object_t *pObj, float *Odr)
{
  return STHS34PF80_OK;
}

/**
 * @brief  Set the STHS34PF80 temperature sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t STHS34PF80_SetOutputDataRate(STHS34PF80_Object_t *pObj, float Odr)
{
  return STHS34PF80_OK;
}


/**
 * @brief  Get the STHS34PF80 temperature value
 * @param  pObj the device pObj
 * @param  Value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
uint8_t recovery_cnts = 0;
int32_t STHS34PF80_GetAxesRaw(STHS34PF80_Object_t *pObj, STHS34PF80_RawData_t *Value)
{
  tmos_algo_sw_input_t algo_input_signal;
  tmos_algo_sw_output_t algo_output_signal;
  i162u8_t recovery_thr;
  uint8_t	embed_data[2];
  int16_t data;
  uint8_t reg_value;

  sths34pf80_tmos_drdy_status_t drdy_status;
  sths34pf80_tmos_func_status_t func_status;

  sths34pf80_tmos_drdy_status_get(&pObj->Ctx,&drdy_status);
  if(drdy_status.drdy == 0)
  {
    return 0; //Tobj not ready, return.
  }
  sths34pf80_tambient_raw_get(&pObj->Ctx, &data);
  Value->Tamb= data;
  sths34pf80_tobject_raw_get(&pObj->Ctx, &data);
  Value->Tobj = data;
 #if ENABLE_TOBJ_COMP
  sths34pf80_tobject_comp_get(&pObj->Ctx, &data);
  Value->Tobj_comp= data;
 #endif
  sths34pf80_tpresence_raw_get(&pObj->Ctx, &data);
  Value->presence = data;
  sths34pf80_tmotion_raw_get(&pObj->Ctx, &data);
  Value->motion = data;
  sths34pf80_tmos_func_status_get(&pObj->Ctx, &func_status);
  Value->pres_flag = func_status.presence;
  Value->mot_flag = func_status.motion;
  Value->tamb_shock_flag = func_status.tamb_shock;
#if ENABLE_TOBJ_COMP
  algo_input_signal.tobj = Value->Tobj_comp;
#else
  algo_input_signal.tobj = Value->Tobj;
#endif
  algo_input_signal.p_flag = Value->pres_flag;
  tmos_algo_sw_processing(&algo_input_signal, &algo_output_signal);
  if(algo_output_signal.recovery_flag == 1)
  {
	recovery_thr.i16bit = Value->presence + 2 * PRESENCE_THS;
	embed_data[0] = recovery_thr.u8bit[0];
	embed_data[1] = recovery_thr.u8bit[1];
	sths34pf80_func_cfg_write(&pObj->Ctx, STHS34PF80_PRESENCE_THS, embed_data, 2);
	while(1)
	{
		sths34pf80_tmos_func_status_get(&pObj->Ctx, &func_status);;
		if(func_status.presence == 0)
		{
			break;
		}
		delay_1ms(20);
	}
	while(1)
	{
		sths34pf80_tpresence_raw_get(&pObj->Ctx, &data);
		if(data < (PRESENCE_THS-PRESENCE_HYS))
		{
			break;
		}
		delay_1ms(20);
	}
	embed_data[0] = (PRESENCE_THS&0xFF); embed_data[1] = ((PRESENCE_THS&0xFF00) >> 8);
  	sths34pf80_func_cfg_write(&pObj->Ctx, STHS34PF80_PRESENCE_THS, embed_data, 2);
  }
  return STHS34PF80_OK;
}

/**
 * @brief  Wrap Read register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  STHS34PF80_Object_t *pObj = (STHS34PF80_Object_t *)Handle;

  return pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length);
}


/**
 * @brief  Wrap Write register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  STHS34PF80_Object_t *pObj = (STHS34PF80_Object_t *)Handle;
  return pObj->IO.WriteReg(pObj->IO.Address, Reg, pData, Length);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
void l81_factory_readid(void)
{

#if 0	
	uint32_t cnt = 20;
	uint32_t reg_data = 0;
	uint32_t chip_init_flag = 0;
	
	while (cnt--) {
  i2c_buffer_read(I2C0, 0x12 << 1, 0xf080, &reg_data, 1u);
  printf("REG_IRQSRC = 0x%x\n", reg_data);
  chip_init_flag = reg_data & 0x01;
  if (chip_init_flag == 1) {
   printf("chip init success cnt = %d\n", cnt);
		break;
  }
  delay_1ms(1);
 }
	#endif
}