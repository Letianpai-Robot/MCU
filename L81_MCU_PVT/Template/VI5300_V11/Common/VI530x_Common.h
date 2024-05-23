/**
  ******************************************************************************
  * @file    VI430x_Common.h
  * @author  L
  * @version V1.0.0
  * @date    22-November-2021
  * @brief   .
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VI530x_COMMON_H
#define VI530x_COMMON_H

//#ifdef __cplusplus
// extern "C" {
//#endif

/* Includes ------------------------------------------------------------------*/
//#include "gd32e23x.h"
#include "gd32l23x.h"
#include "systick.h"
#include "Emu_IIC.h"
#include "EVB_Common.h"
#include "VI530x_User_Handle.h"
//#include "VI530x_System_Data.h"
/** @addtogroup StdPeriph_Driver
  * @{
  */

/** @addtogroup KEY
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/**
  * @}
  */

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
extern uint8_t Debug_Output_State;
extern uint8_t CG_Ratio;
//*VI530x XSHUT***********************************************************************/
//PA0
//#define VI530x_XSHUT_AHBClock_FUN()	__HAL_RCC_GPIOA_CLK_ENABLE();
#define VI530x_XSHUT_CLK			RCU_GPIOA
#define VI530x_XSHUT_PORT			GPIOA
#define VI530x_XSHUT_PIN			GPIO_PIN_0
#define VI530x_XSHUT_LOW()		{GPIO_BC(VI530x_XSHUT_PORT) = VI530x_XSHUT_PIN;}
#define VI530x_XSHUT_HIGH()		{GPIO_BOP(VI530x_XSHUT_PORT) = VI530x_XSHUT_PIN;}


//*CaliFlag AVDD_EN***********************************************************************/
//PA2
#define AVDD_EN_CLK				RCU_GPIOA
#define AVDD_EN_PORT			GPIOA
#define AVDD_EN_PIN				GPIO_PIN_2
#define AVDD_EN_LOW()			{GPIO_BC(AVDD_EN_PORT) = AVDD_EN_PIN;}
#define AVDD_EN_HIGH()		{GPIO_BOP(AVDD_EN_PORT) = AVDD_EN_PIN;}

//*VI530x GPIO1***********************************************************************/
//PA1
//#define VI530x_GPIO1_AHBClock_FUN()	__HAL_RCC_GPIOA_CLK_ENABLE()
#define VI530x_GPIO1_CLK			RCU_GPIOA
#define VI530x_GPIO1_PORT			GPIOA
#define VI530x_GPIO1_PIN			GPIO_PIN_1
#define VI530x_GPIO1_EXTI_LINE            EXTI_1
#define VI530x_GPIO1_EXTI_PORT_SOURCE     EXTI_SOURCE_GPIOA
#define VI530x_GPIO1_EXTI_PIN_SOURCE      EXTI_SOURCE_PIN1
#define VI530x_GPIO1_IRQ									EXTI0_1_IRQn


//*VI530x IIC_SDA***********************************************************************/
//PB6
//#define VI530x_IIC_AHBClock_FUN()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define VI530x_IIC_SCL_CLK			RCU_GPIOB
#define VI530x_IIC_SCL_PORT			GPIOB
#define VI530x_IIC_SCL_PIN			GPIO_PIN_8//GPIO_PIN_6
#define VI530x_IIC_SCL_LOW()		{GPIO_BC(VI530x_IIC_SCL_PORT) = VI530x_IIC_SCL_PIN;}
#define VI530x_IIC_SCL_HIGH()		{GPIO_BOP(VI530x_IIC_SCL_PORT) = VI530x_IIC_SCL_PIN;}

#define VI530x_IIC_SDA_CLK			RCU_GPIOB
#define VI530x_IIC_SDA_PORT			GPIOB
#define VI530x_IIC_SDA_PIN			GPIO_PIN_9
#define VI530x_IIC_SDA_LOW()		{GPIO_BC(VI530x_IIC_SDA_PORT) = VI530x_IIC_SDA_PIN;}
#define VI530x_IIC_SDA_HIGH()		{GPIO_BOP(VI530x_IIC_SDA_PORT) = VI530x_IIC_SDA_PIN;}
#define VI530x_IIC_SDA_INPUTMODE()	{GPIO_CTL(VI530x_IIC_SDA_PORT) &= ~(0x3U << (2U * (9)));\
																			GPIO_CTL(VI530x_IIC_SDA_PORT) |= ((uint32_t)((uint32_t)(GPIO_MODE_INPUT) << (2U * (9))));}																			
#define VI530x_IIC_SDA_OUTPUTMODE()	{GPIO_CTL(VI530x_IIC_SDA_PORT) &= ~(0x3U << (2U * (9)));\
																			GPIO_CTL(VI530x_IIC_SDA_PORT) |= ((uint32_t)((uint32_t)(GPIO_MODE_OUTPUT) << (2U * (9))));}	
#define VI530x_IIC_SDA_READ()				gpio_input_bit_get(VI530x_IIC_SDA_PORT, VI530x_IIC_SDA_PIN)

																	
																			
//*Flash***********************************************************************/
#define FLASH_PAGE_SIZE          ((uint16_t)0x400)	//0X200=0.5K
#define BANK1_START_ADDR        ((uint32_t)0x08000000)
#define FLASH_ADDR_XTALK  			(BANK1_START_ADDR + (FLASH_PAGE_SIZE * 60))
#define FLASH_ADDR_OFFSET  	  	(BANK1_START_ADDR + (FLASH_PAGE_SIZE * 61))
#define FLASH_ADDR_CALIFLAG  	  (BANK1_START_ADDR + (FLASH_PAGE_SIZE * 62))
																			
//#define VI530x_OFFSET_DISTANCE	100 //10cm
#define VI530x_OFFSET_DISTANCE	600 //60cm																		
																			
typedef struct 
{
	//VI530x CG_Pos
	int8_t Calibration_CG_Pos;
	//VI530x CG_Maxratio
	uint8_t Calibration_CG_Maxratio;
	//VI530x CG_Peak
	uint16_t Calibration_CG_Peak;
	//VI530x reftof标定值
	uint16_t Calibration_Reftof;	
	uint8_t NullBuffer[2];	//Flash读写32位对齐
	
} NVM_VI530x_XTALK_Calib_Data;

typedef struct 
{
	//VI530x offset标定值
	float Calibration_Offset;
	//uint8_t NullBuffer;	//Flash读写32位对齐
}NVM_VI530x_Offset_Calib_Data;
	
typedef struct 
{
	uint8_t CalibrationFlag;
	uint8_t NullBuffer[3];	//Flash读写32位对齐
}NVM_VI530x_Calibration_Flag;
																			
//#define IIC_Read_One_Byte(addr,value)				Emu_I2C_Read(VI530x_IIC_DEV_ADDR,addr,value,1)
#define IIC_Read_X_Bytes(addr,value,tlen)		Emu_I2C_Read(VI530x_IIC_Dev_Addr_Now,addr,value,tlen)
//#define IIC_Write_One_Byte(addr,value)			Emu_I2C_Write(VI530x_IIC_DEV_ADDR,addr,&value,1)
#define IIC_Write_X_Bytes(addr,pValue,tlen)	Emu_I2C_Write(VI530x_IIC_Dev_Addr_Now,addr,pValue,tlen)


/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
void VI530x_DelayMs(uint16_t timer_num);
uint8_t VI530x_Calibration_Read(void);
uint8_t VI530x_Calibration_Xtalk(void);
uint8_t VI530x_Calibration_Offset(void);
void VI530x_HAL_Init(void);
void VI530x_HAL_Start(void);
void VI530x_HAL_Stop(void);

void EVB_UART_Interrupt_Callback(void);
void EVB_UART_DMA(void);
																			
void FLASH_ReadByte(uint32_t fAddr, uint8_t *pBuffer, uint16_t NumToRead);
void FLASH_Write(uint32_t WriteAddrStart, uint8_t *pBuffer, uint16_t NumToWrite);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Microelectronics *****END OF FILE****/
