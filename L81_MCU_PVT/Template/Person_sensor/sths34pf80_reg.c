/**
  ******************************************************************************
  * @file    sths34pf80_reg.c
  * @author  Sensors Software Solution Team
  * @brief   STHS34PF80 driver file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#include "sths34pf80_reg.h"

/**
  * @defgroup  STHS34PF80
  * @brief     This file provides a set of functions needed to drive the
  *            sths34pf80 enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup  Interfaces functions
  * @brief     This section provide a set of functions used to read and
  *            write a generic register of the device.
  *            MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  reg   first register address to read.
  * @param  data  buffer for data read.(ptr)
  * @param  len   number of consecutive register to read.
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
                            uint8_t *data,
                            uint16_t len)
{
  int32_t ret;

  ret = ctx->read_reg(ctx->handle, reg, data, len);

  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   communication interface handler.(ptr)
  * @param  reg   first register address to write.
  * @param  data  the buffer contains data to be written.(ptr)
  * @param  len   number of consecutive register to write.
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                             uint8_t *data,
                             uint16_t len)
{
  int32_t ret;

  ret = ctx->write_reg(ctx->handle, reg, data, len);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  Private functions
  * @brief     Section collect all the utility functions needed by APIs.
  * @{
  *
  */

static void bytecpy(uint8_t *target, uint8_t *source)
{
  if ((target != NULL) && (source != NULL))
  {
    *target = *source;
  }
}

/**
  * @}
  *
  */

/**
  * @defgroup Common
  * @brief    Common
  * @{/
  *
  */
/**
  * @brief  Device ID.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Device ID.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_device_id_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_WHO_AM_I, val, 1);

  return ret;
}

/**
  * @brief  Select number of averages for object temperature.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      AVG_TMOS_2, AVG_TMOS_8, AVG_TMOS_32, AVG_TMOS_128, AVG_TMOS_256, AVG_TMOS_512, AVG_TMOS_1024, AVG_TMOS_2048,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_avg_tobject_num_set(stmdev_ctx_t *ctx, sths34pf80_avg_tobject_num_t val)
{
  sths34pf80_avg_trim_t avg_trim;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_AVG_TRIM, (uint8_t *)&avg_trim, 1);

  if (ret == 0)
  {
    avg_trim.avg_tmos = ((uint8_t)val & 0x7);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_AVG_TRIM, (uint8_t *)&avg_trim, 1);
  }

  return ret;
}

/**
  * @brief  Select number of averages for object temperature.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      AVG_TMOS_2, AVG_TMOS_8, AVG_TMOS_32, AVG_TMOS_128, AVG_TMOS_256, AVG_TMOS_512, AVG_TMOS_1024, AVG_TMOS_2048,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_avg_tobject_num_get(stmdev_ctx_t *ctx, sths34pf80_avg_tobject_num_t *val)
{
  sths34pf80_avg_trim_t avg_trim;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_AVG_TRIM, (uint8_t *)&avg_trim, 1);

  switch (avg_trim.avg_tmos)
  {
    case STHS34PF80_AVG_TMOS_2:
      *val = STHS34PF80_AVG_TMOS_2;
      break;

    case STHS34PF80_AVG_TMOS_8:
      *val = STHS34PF80_AVG_TMOS_8;
      break;

    case STHS34PF80_AVG_TMOS_32:
      *val = STHS34PF80_AVG_TMOS_32;
      break;

    case STHS34PF80_AVG_TMOS_128:
      *val = STHS34PF80_AVG_TMOS_128;
      break;

    case STHS34PF80_AVG_TMOS_256:
      *val = STHS34PF80_AVG_TMOS_256;
      break;

    case STHS34PF80_AVG_TMOS_512:
      *val = STHS34PF80_AVG_TMOS_512;
      break;

    case STHS34PF80_AVG_TMOS_1024:
      *val = STHS34PF80_AVG_TMOS_1024;
      break;

    case STHS34PF80_AVG_TMOS_2048:
      *val = STHS34PF80_AVG_TMOS_2048;
      break;

    default:
      *val = STHS34PF80_AVG_TMOS_2;
      break;
  }
  return ret;
}

/**
  * @brief  Select number of averages for ambient temperature.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      AVG_T_8, AVG_T_4, AVG_T_2, AVG_T_1,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_avg_tambient_num_set(stmdev_ctx_t *ctx, sths34pf80_avg_tambient_num_t val)
{
  sths34pf80_avg_trim_t avg_trim;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_AVG_TRIM, (uint8_t *)&avg_trim, 1);

  if (ret == 0)
  {
    avg_trim.avg_t = ((uint8_t)val & 0x3);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_AVG_TRIM, (uint8_t *)&avg_trim, 1);
  }

  return ret;
}

/**
  * @brief  Select number of averages for ambient temperature.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      AVG_T_8, AVG_T_4, AVG_T_2, AVG_T_1,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_avg_tambient_num_get(stmdev_ctx_t *ctx, sths34pf80_avg_tambient_num_t *val)
{
  sths34pf80_avg_trim_t avg_trim;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_AVG_TRIM, (uint8_t *)&avg_trim, 1);

  switch (avg_trim.avg_t)
  {
    case STHS34PF80_AVG_T_8:
      *val = STHS34PF80_AVG_T_8;
      break;

    case STHS34PF80_AVG_T_4:
      *val = STHS34PF80_AVG_T_4;
      break;

    case STHS34PF80_AVG_T_2:
      *val = STHS34PF80_AVG_T_2;
      break;

    case STHS34PF80_AVG_T_1:
      *val = STHS34PF80_AVG_T_1;
      break;

    default:
      *val = STHS34PF80_AVG_T_8;
      break;
  }
  return ret;
}

/**
  * @brief  Selects the tmos odr.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      TMOS_ODR_OFF, TMOS_ODR_AT_0Hz25, TMOS_ODR_AT_0Hz50, TMOS_ODR_1Hz, TMOS_ODR_2Hz, TMOS_ODR_4Hz, TMOS_ODR_8Hz, TMOS_ODR_15Hz, TMOS_ODR_30Hz,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tmos_odr_set(stmdev_ctx_t *ctx, sths34pf80_tmos_odr_t val)
{
  sths34pf80_ctrl1_t ctrl1;
  sths34pf80_avg_trim_t avg_trim;
  sths34pf80_tmos_odr_t max_odr = STHS34PF80_TMOS_ODR_AT_30Hz;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL1, (uint8_t *)&ctrl1, 1);

  if (ret == 0)
  {
    ret = sths34pf80_read_reg(ctx, STHS34PF80_AVG_TRIM, (uint8_t *)&avg_trim, 1);

    switch(avg_trim.avg_tmos)
    {
      case 0:
      case 1:
      case 2:
        max_odr = STHS34PF80_TMOS_ODR_AT_30Hz;
        break;
      case 3:
        max_odr = STHS34PF80_TMOS_ODR_AT_8Hz;
        break;
      case 4:
        max_odr = STHS34PF80_TMOS_ODR_AT_4Hz;
        break;
      case 5:
        max_odr = STHS34PF80_TMOS_ODR_AT_2Hz;
        break;
      case 6:
        max_odr = STHS34PF80_TMOS_ODR_AT_1Hz;
        break;
      case 7:
        max_odr = STHS34PF80_TMOS_ODR_AT_0Hz50;
        break;
    }
  }

  if (ret == 0)
  {
    if (val > max_odr)
    {
      return -1;
    }

    ctrl1.odr = ((uint8_t)val & 0xf);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_CTRL1, (uint8_t *)&ctrl1, 1);
  }

  return ret;
}

/**
  * @brief  Selects the tmos odr.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      TMOS_ODR_OFF, TMOS_ODR_AT_0Hz25, TMOS_ODR_AT_0Hz50, TMOS_ODR_1Hz, TMOS_ODR_2Hz, TMOS_ODR_4Hz, TMOS_ODR_8Hz, TMOS_ODR_15Hz, TMOS_ODR_30Hz,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tmos_odr_get(stmdev_ctx_t *ctx, sths34pf80_tmos_odr_t *val)
{
  sths34pf80_ctrl1_t ctrl1;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL1, (uint8_t *)&ctrl1, 1);

  switch (ctrl1.odr)
  {
    case STHS34PF80_TMOS_ODR_OFF:
      *val = STHS34PF80_TMOS_ODR_OFF;
      break;

    case STHS34PF80_TMOS_ODR_AT_0Hz25:
      *val = STHS34PF80_TMOS_ODR_AT_0Hz25;
      break;

    case STHS34PF80_TMOS_ODR_AT_0Hz50:
      *val = STHS34PF80_TMOS_ODR_AT_0Hz50;
      break;

    case STHS34PF80_TMOS_ODR_AT_1Hz:
      *val = STHS34PF80_TMOS_ODR_AT_1Hz;
      break;

    case STHS34PF80_TMOS_ODR_AT_2Hz:
      *val = STHS34PF80_TMOS_ODR_AT_2Hz;
      break;

    case STHS34PF80_TMOS_ODR_AT_4Hz:
      *val = STHS34PF80_TMOS_ODR_AT_4Hz;
      break;

    case STHS34PF80_TMOS_ODR_AT_8Hz:
      *val = STHS34PF80_TMOS_ODR_AT_8Hz;
      break;

    case STHS34PF80_TMOS_ODR_AT_15Hz:
      *val = STHS34PF80_TMOS_ODR_AT_15Hz;
      break;

    case STHS34PF80_TMOS_ODR_AT_30Hz:
      *val = STHS34PF80_TMOS_ODR_AT_30Hz;
      break;

    default:
      *val = STHS34PF80_TMOS_ODR_OFF;
      break;
  }
  return ret;
}

/**
  * @brief  Block Data Update (BDU): output registers are not updated until LSB and MSB have been read). [set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Block Data Update (BDU): output registers are not updated until LSB and MSB have been read).
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val)
{
  sths34pf80_ctrl1_t ctrl1;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL1, (uint8_t *)&ctrl1, 1);

  if (ret == 0)
  {
    ctrl1.bdu = val;
    ret = sths34pf80_write_reg(ctx, STHS34PF80_CTRL1, (uint8_t *)&ctrl1, 1);
  }

  return ret;
}

/**
  * @brief  Block Data Update (BDU): output registers are not updated until LSB and MSB have been read). [get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Block Data Update (BDU): output registers are not updated until LSB and MSB have been read).
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  sths34pf80_ctrl1_t ctrl1;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL1, (uint8_t *)&ctrl1, 1);

  *val = ctrl1.bdu;


  return ret;
}

/**
  * @brief  Selects data output mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      TMOS_IDLE_MODE, TMOS_ONE_SHOT,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tmos_one_shot_set(stmdev_ctx_t *ctx, sths34pf80_tmos_one_shot_t val)
{
  sths34pf80_ctrl2_t ctrl2;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL2, (uint8_t *)&ctrl2, 1);

  if (ret == 0)
  {
    ctrl2.one_shot = ((uint8_t)val & 0x1);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_CTRL2, (uint8_t *)&ctrl2, 1);
  }

  return ret;
}

/**
  * @brief  Selects data output mode.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      TMOS_IDLE_MODE, TMOS_ONE_SHOT,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tmos_one_shot_get(stmdev_ctx_t *ctx, sths34pf80_tmos_one_shot_t *val)
{
  sths34pf80_ctrl2_t ctrl2;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL2, (uint8_t *)&ctrl2, 1);

  switch (ctrl2.one_shot)
  {
    case STHS34PF80_TMOS_IDLE_MODE:
      *val = STHS34PF80_TMOS_IDLE_MODE;
      break;

    case STHS34PF80_TMOS_ONE_SHOT:
      *val = STHS34PF80_TMOS_ONE_SHOT;
      break;

    default:
      *val = STHS34PF80_TMOS_IDLE_MODE;
      break;
  }
  return ret;
}

/**
  * @brief  Change memory bank.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      MAIN_MEM_BANK, EMBED_FUNC_MEM_BANK, SENSOR_HUB_MEM_BANK, STRED_MEM_BANK,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_mem_bank_set(stmdev_ctx_t *ctx, sths34pf80_mem_bank_t val)
{
  sths34pf80_ctrl2_t ctrl2;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL2, (uint8_t *)&ctrl2, 1);

  if (ret == 0)
  {
    ctrl2.func_cfg_access = ((uint8_t)val & 0x1);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_CTRL2, (uint8_t *)&ctrl2, 1);
  }

  return ret;
}

/**
  * @brief  Change memory bank.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      MAIN_MEM_BANK, EMBED_FUNC_MEM_BANK, SENSOR_HUB_MEM_BANK, STRED_MEM_BANK,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_mem_bank_get(stmdev_ctx_t *ctx, sths34pf80_mem_bank_t *val)
{
  sths34pf80_ctrl2_t ctrl2;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL2, (uint8_t *)&ctrl2, 1);

  switch (ctrl2.func_cfg_access)
  {
    case STHS34PF80_MAIN_MEM_BANK:
      *val = STHS34PF80_MAIN_MEM_BANK;
      break;

    case STHS34PF80_EMBED_FUNC_MEM_BANK:
      *val = STHS34PF80_EMBED_FUNC_MEM_BANK;
      break;

    default:
      *val = STHS34PF80_MAIN_MEM_BANK;
      break;
  }
  return ret;
}

/**
  * @brief  Global reset of the device.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      READY, RESTORE_CTRL_REGS,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_boot_set(stmdev_ctx_t *ctx, uint8_t val)
{
  sths34pf80_ctrl2_t ctrl2;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL2, (uint8_t *)&ctrl2, 1);

  if (ret == 0)
  {
    ctrl2.boot = val;
    ret = sths34pf80_write_reg(ctx, STHS34PF80_CTRL2, (uint8_t *)&ctrl2, 1);
  }

  return ret;
}

/**
  * @brief  Global reset of the device.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      READY, RESTORE_CTRL_REGS,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_boot_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  sths34pf80_ctrl2_t ctrl2;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL2, (uint8_t *)&ctrl2, 1);
  *val = ctrl2.boot;

  return ret;
}

/**
  * @brief  status of drdy.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      status of drdy bit (TAMB, TOBJ, TAMB_SHOCK, TPRESENCE, TMOTION).
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tmos_drdy_status_get(stmdev_ctx_t *ctx, sths34pf80_tmos_drdy_status_t *val)
{
  sths34pf80_status_t status;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_STATUS, (uint8_t *)&status, 1);

  val->drdy = status.drdy;

  return ret;
}

/**
  * @brief  status of internal functions.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      status of internal functions.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tmos_func_status_get(stmdev_ctx_t *ctx, sths34pf80_tmos_func_status_t *val)
{
  sths34pf80_func_status_t func_status;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_FUNC_STATUS, (uint8_t *)&func_status, 1);

  val->tamb_shock = func_status.tamb_shock_flag;
  val->motion = func_status.mot_flag;
  val->presence = func_status.pres_flag;

  return ret;
}

/**
  * @brief  Object temperature output register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Object temperature output register.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tobject_raw_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_TOBJECT_L, &buff[0], 2);

  *val = (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  return ret;
}

int32_t sths34pf80_tobject_comp_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, 0x38, &buff[0], 2);

  *val = (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  return ret;
}


/**
  * @brief  Ambient temperature output register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Ambient temperature output register.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tambient_raw_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_TAMBIENT_L, &buff[0], 2);

  *val = (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  return ret;
}

/**
  * @brief  Presence algo data output register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Presence algo data output register.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tpresence_raw_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_TPRESENCE_L, &buff[0], 2);

  *val = (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  return ret;
}

/**
  * @brief  Motion algo data output register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Motion algo data output register.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tmotion_raw_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_TMOTION_L, &buff[0], 2);

  *val = (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  return ret;
}

/**
  * @brief  Temperature ambient shock algo data output register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Temperature ambient shock algo data output register.
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tamb_shock_raw_get(stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[2];
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_TAMB_SHOCK_L, &buff[0], 2);

  *val = (int16_t)buff[1];
  *val = (*val * 256) + (int16_t)buff[0];

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup Filters
  * @brief    Filters
  * @{/
  *
  */
/**
  * @brief  low-pass filter configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      LPF_ODR_DIV_9, LPF_ODR_DIV_20, LPF_ODR_DIV_50, LPF_ODR_DIV_100, LPF_ODR_DIV_200, LPF_ODR_DIV_400, LPF_ODR_DIV_800,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_lpf_m_bandwidth_set(stmdev_ctx_t *ctx, sths34pf80_lpf_bandwidth_t val)
{
  sths34pf80_lpf1_t lpf1;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_LPF1, (uint8_t *)&lpf1, 1);

  if (ret == 0)
  {
    lpf1.lpf_m = ((uint8_t)val & 0x7);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_LPF1, (uint8_t *)&lpf1, 1);
  }

  return ret;
}

/**
  * @brief  low-pass filter configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      LPF_ODR_DIV_9, LPF_ODR_DIV_20, LPF_ODR_DIV_50, LPF_ODR_DIV_100, LPF_ODR_DIV_200, LPF_ODR_DIV_400, LPF_ODR_DIV_800,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_lpf_m_bandwidth_get(stmdev_ctx_t *ctx, sths34pf80_lpf_bandwidth_t *val)
{
  sths34pf80_lpf1_t lpf1;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_LPF1, (uint8_t *)&lpf1, 1);

  switch ((lpf1.lpf_m))
  {
    case STHS34PF80_LPF_ODR_DIV_9:
      *val = STHS34PF80_LPF_ODR_DIV_9;
      break;

    case STHS34PF80_LPF_ODR_DIV_20:
      *val = STHS34PF80_LPF_ODR_DIV_20;
      break;

    case STHS34PF80_LPF_ODR_DIV_50:
      *val = STHS34PF80_LPF_ODR_DIV_50;
      break;

    case STHS34PF80_LPF_ODR_DIV_100:
      *val = STHS34PF80_LPF_ODR_DIV_100;
      break;

    case STHS34PF80_LPF_ODR_DIV_200:
      *val = STHS34PF80_LPF_ODR_DIV_200;
      break;

    case STHS34PF80_LPF_ODR_DIV_400:
      *val = STHS34PF80_LPF_ODR_DIV_400;
      break;

    case STHS34PF80_LPF_ODR_DIV_800:
      *val = STHS34PF80_LPF_ODR_DIV_800;
      break;

    default:
      *val = STHS34PF80_LPF_ODR_DIV_9;
      break;
  }
  return ret;
}

/**
  * @brief  low-pass filter configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      LPF_ODR_DIV_9, LPF_ODR_DIV_20, LPF_ODR_DIV_50, LPF_ODR_DIV_100, LPF_ODR_DIV_200, LPF_ODR_DIV_400, LPF_ODR_DIV_800,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_lpf_p_m_bandwidth_set(stmdev_ctx_t *ctx, sths34pf80_lpf_bandwidth_t val)
{
  sths34pf80_lpf1_t lpf1;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_LPF1, (uint8_t *)&lpf1, 1);

  if (ret == 0)
  {
    lpf1.lpf_p_m = ((uint8_t)val & 0x7);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_LPF1, (uint8_t *)&lpf1, 1);
  }

  return ret;
}

/**
  * @brief  low-pass filter configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      LPF_ODR_DIV_9, LPF_ODR_DIV_20, LPF_ODR_DIV_50, LPF_ODR_DIV_100, LPF_ODR_DIV_200, LPF_ODR_DIV_400, LPF_ODR_DIV_800,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_lpf_p_m_bandwidth_get(stmdev_ctx_t *ctx, sths34pf80_lpf_bandwidth_t *val)
{
  sths34pf80_lpf1_t lpf1;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_LPF1, (uint8_t *)&lpf1, 1);

  switch ((lpf1.lpf_p_m))
  {
    case STHS34PF80_LPF_ODR_DIV_9:
      *val = STHS34PF80_LPF_ODR_DIV_9;
      break;

    case STHS34PF80_LPF_ODR_DIV_20:
      *val = STHS34PF80_LPF_ODR_DIV_20;
      break;

    case STHS34PF80_LPF_ODR_DIV_50:
      *val = STHS34PF80_LPF_ODR_DIV_50;
      break;

    case STHS34PF80_LPF_ODR_DIV_100:
      *val = STHS34PF80_LPF_ODR_DIV_100;
      break;

    case STHS34PF80_LPF_ODR_DIV_200:
      *val = STHS34PF80_LPF_ODR_DIV_200;
      break;

    case STHS34PF80_LPF_ODR_DIV_400:
      *val = STHS34PF80_LPF_ODR_DIV_400;
      break;

    case STHS34PF80_LPF_ODR_DIV_800:
      *val = STHS34PF80_LPF_ODR_DIV_800;
      break;

    default:
      *val = STHS34PF80_LPF_ODR_DIV_9;
      break;
  }
  return ret;
}

/**
  * @brief  low-pass filter configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      LPF_ODR_DIV_9, LPF_ODR_DIV_20, LPF_ODR_DIV_50, LPF_ODR_DIV_100, LPF_ODR_DIV_200, LPF_ODR_DIV_400, LPF_ODR_DIV_800,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_lpf_a_t_bandwidth_set(stmdev_ctx_t *ctx, sths34pf80_lpf_bandwidth_t val)
{
  sths34pf80_lpf2_t lpf2;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_LPF2, (uint8_t *)&lpf2, 1);

  if (ret == 0)
  {
    lpf2.lpf_a_t = ((uint8_t)val & 0x7);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_LPF2, (uint8_t *)&lpf2, 1);
  }

  return ret;
}

/**
  * @brief  low-pass filter configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      LPF_ODR_DIV_9, LPF_ODR_DIV_20, LPF_ODR_DIV_50, LPF_ODR_DIV_100, LPF_ODR_DIV_200, LPF_ODR_DIV_400, LPF_ODR_DIV_800,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_lpf_a_t_bandwidth_get(stmdev_ctx_t *ctx, sths34pf80_lpf_bandwidth_t *val)
{
  sths34pf80_lpf2_t lpf2;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_LPF2, (uint8_t *)&lpf2, 1);

  switch ((lpf2.lpf_a_t))
  {
    case STHS34PF80_LPF_ODR_DIV_9:
      *val = STHS34PF80_LPF_ODR_DIV_9;
      break;

    case STHS34PF80_LPF_ODR_DIV_20:
      *val = STHS34PF80_LPF_ODR_DIV_20;
      break;

    case STHS34PF80_LPF_ODR_DIV_50:
      *val = STHS34PF80_LPF_ODR_DIV_50;
      break;

    case STHS34PF80_LPF_ODR_DIV_100:
      *val = STHS34PF80_LPF_ODR_DIV_100;
      break;

    case STHS34PF80_LPF_ODR_DIV_200:
      *val = STHS34PF80_LPF_ODR_DIV_200;
      break;

    case STHS34PF80_LPF_ODR_DIV_400:
      *val = STHS34PF80_LPF_ODR_DIV_400;
      break;

    case STHS34PF80_LPF_ODR_DIV_800:
      *val = STHS34PF80_LPF_ODR_DIV_800;
      break;

    default:
      *val = STHS34PF80_LPF_ODR_DIV_9;
      break;
  }
  return ret;
}

/**
  * @brief  low-pass filter configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      LPF_ODR_DIV_9, LPF_ODR_DIV_20, LPF_ODR_DIV_50, LPF_ODR_DIV_100, LPF_ODR_DIV_200, LPF_ODR_DIV_400, LPF_ODR_DIV_800,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_lpf_p_bandwidth_set(stmdev_ctx_t *ctx, sths34pf80_lpf_bandwidth_t val)
{
  sths34pf80_lpf2_t lpf2;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_LPF2, (uint8_t *)&lpf2, 1);

  if (ret == 0)
  {
    lpf2.lpf_p = ((uint8_t)val & 0x7);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_LPF2, (uint8_t *)&lpf2, 1);
  }

  return ret;
}

/**
  * @brief  low-pass filter configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      LPF_ODR_DIV_9, LPF_ODR_DIV_20, LPF_ODR_DIV_50, LPF_ODR_DIV_100, LPF_ODR_DIV_200, LPF_ODR_DIV_400, LPF_ODR_DIV_800,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_lpf_p_bandwidth_get(stmdev_ctx_t *ctx, sths34pf80_lpf_bandwidth_t *val)
{
  sths34pf80_lpf2_t lpf2;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_LPF2, (uint8_t *)&lpf2, 1);

  switch ((lpf2.lpf_p))
  {
    case STHS34PF80_LPF_ODR_DIV_9:
      *val = STHS34PF80_LPF_ODR_DIV_9;
      break;

    case STHS34PF80_LPF_ODR_DIV_20:
      *val = STHS34PF80_LPF_ODR_DIV_20;
      break;

    case STHS34PF80_LPF_ODR_DIV_50:
      *val = STHS34PF80_LPF_ODR_DIV_50;
      break;

    case STHS34PF80_LPF_ODR_DIV_100:
      *val = STHS34PF80_LPF_ODR_DIV_100;
      break;

    case STHS34PF80_LPF_ODR_DIV_200:
      *val = STHS34PF80_LPF_ODR_DIV_200;
      break;

    case STHS34PF80_LPF_ODR_DIV_400:
      *val = STHS34PF80_LPF_ODR_DIV_400;
      break;

    case STHS34PF80_LPF_ODR_DIV_800:
      *val = STHS34PF80_LPF_ODR_DIV_800;
      break;

    default:
      *val = STHS34PF80_LPF_ODR_DIV_9;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup Interrupt PINs
  * @brief    Interrupt PINs
  * @{/
  *
  */
/**
  * @brief  Selects interrupts to be routed.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      TMOS_INT_HIZ, TMOS_INT_DRDY, TMOS_INT_OR,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tmos_route_int_set(stmdev_ctx_t *ctx, sths34pf80_tmos_route_int_t val)
{
  sths34pf80_ctrl3_t ctrl3;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL3, (uint8_t *)&ctrl3, 1);

  if (ret == 0)
  {
    ctrl3.ien = ((uint8_t)val & 0x3);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_CTRL3, (uint8_t *)&ctrl3, 1);
  }

  return ret;
}

/**
  * @brief  Selects interrupts to be routed.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      TMOS_INT_HIZ, TMOS_INT_DRDY, TMOS_INT_OR,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tmos_route_int_get(stmdev_ctx_t *ctx, sths34pf80_tmos_route_int_t *val)
{
  sths34pf80_ctrl3_t ctrl3;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL3, (uint8_t *)&ctrl3, 1);

  switch ((ctrl3.ien))
  {
    case STHS34PF80_TMOS_INT_HIZ:
      *val = STHS34PF80_TMOS_INT_HIZ;
      break;

    case STHS34PF80_TMOS_INT_DRDY:
      *val = STHS34PF80_TMOS_INT_DRDY;
      break;

    case STHS34PF80_TMOS_INT_OR:
      *val = STHS34PF80_TMOS_INT_OR;
      break;

    default:
      *val = STHS34PF80_TMOS_INT_HIZ;
      break;
  }
  return ret;
}

/**
  * @brief  Selects interrupts output.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      TMOS_INT_NONE, TMOS_INT_TSHOCK, TMOS_INT_MOTION, TMOS_INT_TSHOCK_MOTION, TMOS_INT_PRESENCE, TMOS_INT_TSHOCK_PRESENCE, TMOS_INT_MOTION_PRESENCE, TMOS_INT_ALL,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tmos_int_set(stmdev_ctx_t *ctx, sths34pf80_tmos_int_t val)
{
  sths34pf80_ctrl3_t ctrl3;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL3, (uint8_t *)&ctrl3, 1);

  if (ret == 0)
  {
    ctrl3.int_msk = ((uint8_t)val & 0x7);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_CTRL3, (uint8_t *)&ctrl3, 1);
  }

  return ret;
}

/**
  * @brief  Selects interrupts output.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      TMOS_INT_NONE, TMOS_INT_TSHOCK, TMOS_INT_MOTION, TMOS_INT_TSHOCK_MOTION, TMOS_INT_PRESENCE, TMOS_INT_TSHOCK_PRESENCE, TMOS_INT_MOTION_PRESENCE, TMOS_INT_ALL,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_tmos_int_get(stmdev_ctx_t *ctx, sths34pf80_tmos_int_t *val)
{
  sths34pf80_ctrl3_t ctrl3;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL3, (uint8_t *)&ctrl3, 1);

  switch ((ctrl3.int_msk))
  {
    case STHS34PF80_TMOS_INT_NONE:
      *val = STHS34PF80_TMOS_INT_NONE;
      break;

    case STHS34PF80_TMOS_INT_TSHOCK:
      *val = STHS34PF80_TMOS_INT_TSHOCK;
      break;

    case STHS34PF80_TMOS_INT_MOTION:
      *val = STHS34PF80_TMOS_INT_MOTION;
      break;

    case STHS34PF80_TMOS_INT_TSHOCK_MOTION:
      *val = STHS34PF80_TMOS_INT_TSHOCK_MOTION;
      break;

    case STHS34PF80_TMOS_INT_PRESENCE:
      *val = STHS34PF80_TMOS_INT_PRESENCE;
      break;

    case STHS34PF80_TMOS_INT_TSHOCK_PRESENCE:
      *val = STHS34PF80_TMOS_INT_TSHOCK_PRESENCE;
      break;

    case STHS34PF80_TMOS_INT_MOTION_PRESENCE:
      *val = STHS34PF80_TMOS_INT_MOTION_PRESENCE;
      break;

    case STHS34PF80_TMOS_INT_ALL:
      *val = STHS34PF80_TMOS_INT_ALL;
      break;

    default:
      *val = STHS34PF80_TMOS_INT_NONE;
      break;
  }
  return ret;
}

/**
  * @brief  Push-pull/open-drain selection on INT1 and INT2 pins.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      PUSH_PULL, OPEN_DRAIN,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_int_pin_mode_set(stmdev_ctx_t *ctx, sths34pf80_int_pin_mode_t val)
{
  sths34pf80_ctrl3_t ctrl3;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL3, (uint8_t *)&ctrl3, 1);

  if (ret == 0)
  {
    ctrl3.pp_od = ((uint8_t)val & 0x1);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_CTRL3, (uint8_t *)&ctrl3, 1);
  }

  return ret;
}

/**
  * @brief  Push-pull/open-drain selection on INT1 and INT2 pins.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      PUSH_PULL, OPEN_DRAIN,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_int_pin_mode_get(stmdev_ctx_t *ctx, sths34pf80_int_pin_mode_t *val)
{
  sths34pf80_ctrl3_t ctrl3;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL3, (uint8_t *)&ctrl3, 1);

  switch ((ctrl3.pp_od))
  {
    case STHS34PF80_PUSH_PULL:
      *val = STHS34PF80_PUSH_PULL;
      break;

    case STHS34PF80_OPEN_DRAIN:
      *val = STHS34PF80_OPEN_DRAIN;
      break;

    default:
      *val = STHS34PF80_PUSH_PULL;
      break;
  }
  return ret;
}

/**
  * @brief  Interrupt activation level.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      ACTIVE_HIGH, ACTIVE_LOW,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_pin_polarity_set(stmdev_ctx_t *ctx, sths34pf80_pin_polarity_t val)
{
  sths34pf80_ctrl3_t ctrl3;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL3, (uint8_t *)&ctrl3, 1);

  if (ret == 0)
  {
    ctrl3.int_h_l = ((uint8_t)val & 0x1);
    ret = sths34pf80_write_reg(ctx, STHS34PF80_CTRL3, (uint8_t *)&ctrl3, 1);
  }

  return ret;
}

/**
  * @brief  Interrupt activation level.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      ACTIVE_HIGH, ACTIVE_LOW,
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_pin_polarity_get(stmdev_ctx_t *ctx, sths34pf80_pin_polarity_t *val)
{
  sths34pf80_ctrl3_t ctrl3;
  int32_t ret;

  ret = sths34pf80_read_reg(ctx, STHS34PF80_CTRL3, (uint8_t *)&ctrl3, 1);

  switch ((ctrl3.int_h_l))
  {
    case STHS34PF80_ACTIVE_HIGH:
      *val = STHS34PF80_ACTIVE_HIGH;
      break;

    case STHS34PF80_ACTIVE_LOW:
      *val = STHS34PF80_ACTIVE_LOW;
      break;

    default:
      *val = STHS34PF80_ACTIVE_HIGH;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup Embedded
  * @brief    Embedded
  * @{/
  *
  */
/**
  * @brief  Function Configuration write
  *
  * @param  ctx      read / write interface definitions
  * @param  addr     embedded register address
  * @param  data     embedded register data
  * @param  len      embedded register data len
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_func_cfg_write(stmdev_ctx_t *ctx, uint8_t addr, uint8_t *data, uint8_t len)
{
  sths34pf80_page_rw_t page_rw = {0};
  int32_t ret;
  int32_t i;

  ret = sths34pf80_mem_bank_set(ctx, STHS34PF80_EMBED_FUNC_MEM_BANK);
  if (ret == 0)
  {
    /* Set func_cfg_write to 1 */
    page_rw.func_cfg_write = 1;
    ret = sths34pf80_write_reg(ctx, STHS34PF80_PAGE_RW, (uint8_t *)&page_rw, 1);
  }

  if (ret == 0)
  {
    /* write func_cfg_address (it will autoincrement when writing) */
    ret = sths34pf80_write_reg(ctx, STHS34PF80_FUNC_CFG_ADDR, &addr, 1);
  }

  if (ret == 0)
  {
    for (i = 0; i < len; i++)
    {
      /* write func_cfg_data */
      ret = sths34pf80_write_reg(ctx, STHS34PF80_FUNC_CFG_DATA, &data[i], 1);
    }
  }

  if (ret == 0)
  {
    /* Set func_cfg_write to 0 */
    page_rw.func_cfg_write = 0;
    ret = sths34pf80_write_reg(ctx, STHS34PF80_PAGE_RW, (uint8_t *)&page_rw, 1);
  }

  if (ret == 0)
  {
      ret = sths34pf80_mem_bank_set(ctx, STHS34PF80_MAIN_MEM_BANK);
  }

  return ret;
}

/**
  * @brief  Function Configuration read
  *
  * @param  ctx      read / write interface definitions
  * @param  addr     embedded register address
  * @param  data     embedded register data
  * @param  len      embedded register data len
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t sths34pf80_func_cfg_read(stmdev_ctx_t *ctx, uint8_t addr, uint8_t *data, uint8_t len)
{
  sths34pf80_page_rw_t page_rw = {0};
  uint8_t reg_addr;
  int32_t ret;
  int32_t i;

  ret = sths34pf80_mem_bank_set(ctx, STHS34PF80_EMBED_FUNC_MEM_BANK);
  if (ret == 0)
  {
    /* Set func_cfg_read to 1 */
    page_rw.func_cfg_read = 1;
    ret = sths34pf80_write_reg(ctx, STHS34PF80_PAGE_RW, (uint8_t *)&page_rw, 1);
  }

  if (ret == 0)
  {
    for (i = 0; i < len; i++)
    {
      /* write func_cfg_address */
      reg_addr = addr + i;
      ret = sths34pf80_write_reg(ctx, STHS34PF80_FUNC_CFG_ADDR, &reg_addr, 1);

      if (ret == 0)
      {
        /* read func_cfg_data */
        ret = sths34pf80_read_reg(ctx, STHS34PF80_FUNC_CFG_DATA, &data[i], 1);
      }
    }
  }

  if (ret == 0)
  {
    /* Set func_cfg_read to 0 */
    page_rw.func_cfg_read = 0;
    ret = sths34pf80_write_reg(ctx, STHS34PF80_PAGE_RW, (uint8_t *)&page_rw, 1);
  }

  if (ret == 0)
  {
      ret = sths34pf80_mem_bank_set(ctx, STHS34PF80_MAIN_MEM_BANK);
  }

  return ret;
}

/**
  * @}
  *
  */

