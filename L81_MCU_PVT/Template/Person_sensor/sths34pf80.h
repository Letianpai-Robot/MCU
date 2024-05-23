/**
 ******************************************************************************
 * @file    stts751.h
 * @author  MEMS Software Solutions Team
 * @brief   STTS751 header driver file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STHS34PF80_H
#define STHS34PF80_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "sths34pf80_reg.h"
#include <string.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @addtogroup STHS34PF80 STHS34PF80
 * @{
 */

/** @defgroup STHS34PF80_Exported_Types STHS34PF80 Exported Types
 * @{
 */

typedef int32_t (*STHS34PF80_Init_Func)(void);
typedef int32_t (*STHS34PF80_DeInit_Func)(void);
typedef int32_t (*STHS34PF80_GetTick_Func)(void);
typedef int32_t (*STHS34PF80_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*STHS34PF80_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  uint8_t data_status;
  uint8_t pres_flag;
  uint8_t mot_flag;
  uint8_t tamb_shock_flag;
  int16_t Tobj;
  int16_t Tobj_comp;
  int16_t Tamb;
  int16_t presence;
  int16_t motion;
} STHS34PF80_RawData_t;

typedef struct
{
  float Object;
  float Ambient;
  float Presence;
} STHS34PF80_Data_t;


typedef struct
{
  STHS34PF80_Init_Func          Init;
  STHS34PF80_DeInit_Func        DeInit;
  uint32_t                   BusType; /*0 means I2C */
  uint8_t                    Address;
  STHS34PF80_WriteReg_Func      WriteReg;
  STHS34PF80_ReadReg_Func       ReadReg;
  STHS34PF80_GetTick_Func       GetTick;
} STHS34PF80_IO_t;

typedef struct
{
  STHS34PF80_IO_t       IO;
  stmdev_ctx_t       Ctx;
  uint8_t            is_initialized;
  uint8_t            enabled;
  float              odr;
} STHS34PF80_Object_t;

typedef struct
{
  uint8_t tmos; 
  float   MaxOdr;
} STHS34PF80_Capabilities_t;

typedef struct
{
  int32_t (*Init)(STHS34PF80_Object_t *);
  int32_t (*DeInit)(STHS34PF80_Object_t *);
  int32_t (*ReadID)(STHS34PF80_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(STHS34PF80_Object_t *, STHS34PF80_Capabilities_t *);
} STHS34PF80_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(STHS34PF80_Object_t *);
  int32_t (*Disable)(STHS34PF80_Object_t *);
  int32_t (*GetSensitivity)(STHS34PF80_Object_t *, float *);
  int32_t (*GetOutputDataRate)(STHS34PF80_Object_t *, float *);
  int32_t (*SetOutputDataRate)(STHS34PF80_Object_t *, float);
  int32_t (*GetFullScale)(STHS34PF80_Object_t *, int32_t *);
  int32_t (*SetFullScale)(STHS34PF80_Object_t *, int32_t);
  int32_t (*GetAxes)(STHS34PF80_Object_t *, STHS34PF80_Data_t *);
  int32_t (*GetAxesRaw)(STHS34PF80_Object_t *, STHS34PF80_RawData_t *);
} STHS34PF80_Drv_t;

/**
 * @}
 */

/** @defgroup STHS34PF80_Exported_Constants STHS34PF80 Exported Constants
 * @{
 */
#define STHS34PF80_I2C_BUS           0U

/** STHS34PF80 error codes  **/
#define STHS34PF80_OK                 0
#define STHS34PF80_ERROR             -1

/**
 * @}
 */

/** @addtogroup STHS34PF80_Exported_Functions STHS34PF80 Exported Functions
 * @{
 */

int32_t STHS34PF80_RegisterBusIO(STHS34PF80_Object_t *pObj, STHS34PF80_IO_t *pIO);
int32_t STHS34PF80_Init(STHS34PF80_Object_t *pObj);
int32_t STHS34PF80_DeInit(STHS34PF80_Object_t *pObj);
int32_t STHS34PF80_ReadID(STHS34PF80_Object_t *pObj, uint8_t *Id);
int32_t STHS34PF80_GetCapabilities(STHS34PF80_Object_t *pObj, STHS34PF80_Capabilities_t *Capabilities);

int32_t STHS34PF80_Enable(STHS34PF80_Object_t *pObj);
int32_t STHS34PF80_Disable(STHS34PF80_Object_t *pObj);
int32_t STHS34PF80_GetOutputDataRate(STHS34PF80_Object_t *pObj, float *Odr);
int32_t STHS34PF80_SetOutputDataRate(STHS34PF80_Object_t *pObj, float Odr);
int32_t STHS34PF80_GetAxesRaw(STHS34PF80_Object_t *pObj, STHS34PF80_RawData_t *Value);
int32_t STHS34PF80_GetSensitivity(STHS34PF80_Object_t *pObj, float *Sensitivity);
int32_t STHS34PF80_GetFullScale(STHS34PF80_Object_t *pObj, int32_t *fs);
int32_t STHS34PF80_SetFullScale(STHS34PF80_Object_t *pObj, int32_t fs);
int32_t STHS34PF80_GetAxes(STHS34PF80_Object_t *pObj, STHS34PF80_Data_t * data);

void l81_factory_readid(void);


/**
 * @}
 */

/** @addtogroup STHS34PF80_Exported_Variables STHS34PF80 Exported Variables
 * @{
 */

extern STHS34PF80_CommonDrv_t STHS34PF80_COMMON_Driver;
extern STHS34PF80_Drv_t STHS34PF80_Driver;

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
