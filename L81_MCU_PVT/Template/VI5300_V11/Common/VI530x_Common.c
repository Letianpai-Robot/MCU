/***********************************************************************************************************************
Includes
***********************************************************************************************************************/


/* Start user code for include. Do not edit comment generated here */
#include "VI530x_Common.h"
#include "stdio.h"
#include <string.h>

/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
uint8_t Debug_Output_State = 0;
uint8_t CG_Ratio = 0;
/* End user code. Do not edit comment generated here */

   
/***********************************************************************************************************************
* Function Name: VI530x_DelayMs
* Description  : 延时函数
* Arguments    : timer_num: 延时时间，单位ms
* Return Value : None
***********************************************************************************************************************/
void VI530x_DelayMs(uint16_t timer_num)
{
	delay_1ms(timer_num);
}



//**VI530x XSHUT**//
/***********************************************************************************************************************
* Function Name: VI530x_XSHUT_Init
* Description  : VI530x XSHUT引脚初始化，输出，高电平有效
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void VI530x_XSHUT_Init(void)
{
	/* enable the GPIO clock */
	rcu_periph_clock_enable(VI530x_XSHUT_CLK);
	/* configure GPIO port */ 
	gpio_mode_set(VI530x_XSHUT_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, VI530x_XSHUT_PIN);
	gpio_output_options_set(VI530x_XSHUT_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, VI530x_XSHUT_PIN);

	VI530x_XSHUT_LOW();
}

//**AVDD_EN**//
/***********************************************************************************************************************
* Function Name: AVDD_EN_Init
* Description  : AVDD EN引脚初始化，输出，高电平有效
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void AVDD_EN_Init(void)
{
	/* enable the GPIO clock */
	rcu_periph_clock_enable(AVDD_EN_CLK);
	/* configure GPIO port */ 
	gpio_mode_set(AVDD_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, AVDD_EN_PIN);
	gpio_output_options_set(AVDD_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, AVDD_EN_PIN);

	AVDD_EN_HIGH();
}


//**VI530x GPIO0**//
/***********************************************************************************************************************
* Function Name: VI530x_GPIO0_Init
* Description  : 初始化配置MCU的GPIO0引脚为下降沿输入中断检测
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void VI530x_GPIO0_Init(void)
{
	rcu_periph_clock_enable(VI530x_GPIO1_CLK);
	//rcu_periph_clock_enable(RCU_CFGCMP);

	/* configure GPIO pin as input */
	gpio_mode_set(VI530x_GPIO1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, VI530x_GPIO1_PIN);

	/* enable and set GPIO EXTI interrupt to the lowest priority */
//	nvic_irq_enable(VI530x_GPIO1_IRQ, 2U);

	/* connect GPIO EXTI line to GPIO pin */
	syscfg_exti_line_config(VI530x_GPIO1_EXTI_PORT_SOURCE, VI530x_GPIO1_EXTI_PIN_SOURCE);

	/* configure GPIO EXTI line */
	exti_init(VI530x_GPIO1_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
	exti_interrupt_flag_clear(VI530x_GPIO1_EXTI_LINE);
}


//**IIC**//

/***********************************************************************************************************************
* Function Name: VI530x_IIC_Init
* Description  : 与VI530x的软件IIC初始化
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void VI530x_IIC_Init(void)
{
	/* enable the IIC GPIO clock */
	rcu_periph_clock_enable(VI530x_IIC_SCL_CLK);
	/* configure IIC GPIO port */ 
	gpio_mode_set(VI530x_IIC_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, VI530x_IIC_SCL_PIN);
	gpio_output_options_set(VI530x_IIC_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, VI530x_IIC_SCL_PIN);

	/* configure IIC GPIO port */ 
	gpio_mode_set(VI530x_IIC_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, VI530x_IIC_SDA_PIN);
	gpio_output_options_set(VI530x_IIC_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, VI530x_IIC_SDA_PIN);
	
	VI530x_IIC_SCL_HIGH();
	VI530x_IIC_SDA_HIGH();
}


/***********************************************************************************************************************
* Function Name: FLASH_ReadByte
* Description  : 写Flash
* Arguments    : fAddr：Flash读的开始地址
*								 pBuffer：Buffer数据指针
*								 NumToWrite：字节大小
* Return Value : None
***********************************************************************************************************************/
void FLASH_ReadByte(uint32_t fAddr, uint8_t *pBuffer, uint16_t NumToRead)   
{
    uint16_t i;
  
    for (i = 0; i < NumToRead; i++)
    {
        pBuffer[i] = *(uint8_t*)fAddr; 
        fAddr++; //偏移1个字节.	
    }
  
}



/***********************************************************************************************************************
* Function Name: FLASH_Write
* Description  : 写Flash
* Arguments    : WriteAddrStart：Flash写的开始地址
*								 pBuffer：写内容Buffer地址
*								 NumToWrite：字节大小
* Return Value : None
***********************************************************************************************************************/
void FLASH_Write(uint32_t WriteAddrStart, uint8_t *pBuffer, uint16_t NumToWrite)
{
//	FLASH_EraseInitTypeDef FlashEraseInit;
	uint8_t FlashStatus = 1;
	uint32_t SectorError=0;
	uint32_t *data_buffer_val = (uint32_t *)pBuffer;  
	//uint32_t PageAddr = BANK1_START_ADDR + (FLASH_PAGE_SIZE * WriteAddrStart); 
	uint32_t Address = WriteAddrStart;
//	
	/****解锁****/
//	HAL_FLASH_Unlock();             //解锁	
	
	/* unlock the flash program/erase controller */
	fmc_unlock();

	/* clear all pending flags */
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);

	/****擦除****/
	/* erase target page */
  fmc_page_erase(WriteAddrStart);
	if(0xFFFFFFFF != ( *(__IO uint32_t*) Address) )
	{
		FlashStatus = 0;
	}
//	FlashEraseInit.TypeErase=FLASH_TYPEERASE_PAGES;       //擦除类型，页擦除 
//	FlashEraseInit.Banks=FLASH_BANK_1;   									//要擦除的扇区
//	FlashEraseInit.PageAddress=WriteAddrStart;            //一次只擦除一个扇区
//	FlashEraseInit.NbPages=1;      												//一次只擦除一个页
//	FlashStatus = HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError);
	
	
	/****写****/
	//PageOfAddr = WriteAddrStart;
	Address = WriteAddrStart;
	while((Address < (WriteAddrStart + NumToWrite)) && (FlashStatus == 1))
	{
		//FlashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Address, *data_buffer_val++);
		/* program target address */
        fmc_word_program(Address, *data_buffer_val++);
		Address = Address + 4;
	}
	/****验证写****/
	Address = WriteAddrStart;
	data_buffer_val = (uint32_t *)pBuffer;
	while((Address < (WriteAddrStart + NumToWrite)) && (FlashStatus == 1))
	{
		if((*(__IO uint32_t*) Address) != *data_buffer_val++)
		{
			FlashStatus = 0;
		}
			Address += 4;
	}
		
	/****上锁****/
//	HAL_FLASH_Lock();           //上锁
	/* clear all pending flags */
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);

	/* lock the main FMC after the operation */
	fmc_lock();
}

/***********************************************************************************************************************
* Function Name: VI530x_Calibration_Read
* Description  : 标定值读取
* Arguments    : None
* Return Value : ret
***********************************************************************************************************************/
uint8_t VI530x_Calibration_Read(void)
{
	uint8_t ret = 0;
	NVM_VI530x_XTALK_Calib_Data XtalkCaliData;
	NVM_VI530x_Offset_Calib_Data OffsetCaliData;
	NVM_VI530x_Calibration_Flag CaliFlag;
	
	FLASH_ReadByte(FLASH_ADDR_CALIFLAG, (uint8_t*)&CaliFlag, 1);
	if(CaliFlag.CalibrationFlag == 0x03)
	{
		FLASH_ReadByte(FLASH_ADDR_XTALK, (uint8_t*)&XtalkCaliData, sizeof(XtalkCaliData));
		
		
		VI530x_Cali_Data.VI530x_Calibration_CG_Pos = XtalkCaliData.Calibration_CG_Pos;
		//printf("VI530x_Calibration_CG_Pos = %d\r\n",VI530x_Cali_Data.VI530x_Calibration_CG_Pos);
		
		VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio = XtalkCaliData.Calibration_CG_Maxratio;
		//printf("VI530x_Calibration_CG_Maxratio = %d\r\n",VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio);
		
		VI530x_Cali_Data.VI530x_Calibration_CG_peak = XtalkCaliData.Calibration_CG_Peak;
		//printf("VI530x_Calibration_CG_Peak = %d\r\n",VI530x_Cali_Data.VI530x_Calibration_CG_peak);
	
		VI530x_Cali_Data.VI530x_Calibration_Reftof = XtalkCaliData.Calibration_Reftof;
		//printf("VI530x_Calibration_Reftof = %d\r\n",VI530x_Cali_Data.VI530x_Calibration_Reftof);

		
		FLASH_ReadByte(FLASH_ADDR_OFFSET, (uint8_t*)&OffsetCaliData, sizeof(OffsetCaliData));
		VI530x_Cali_Data.VI530x_Calibration_Offset = OffsetCaliData.Calibration_Offset;
		//printf("VI530x_Calibration_Offset = %f\r\n",VI530x_Cali_Data.VI530x_Calibration_Offset);
	}
	else
	{
		//无标定使用缺省值
		VI530x_Cali_Data.VI530x_Calibration_Offset = 0;
		VI530x_Cali_Data.VI530x_Calibration_CG_Pos = 2;         //  0718  0->2
		VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio = 20;   //  0718  6->20
		//printf("VI530x_Calibration_Offset = %f\r\n",VI530x_Cali_Data.VI530x_Calibration_Offset);
		//printf("Without Calibration!\r\n");
	}
		
	return ret;
	
}

/***********************************************************************************************************************
* Function Name: VI530x_Calibration_Xtalk
* Description  : Xtalk标定
* Arguments    : None
* Return Value : ret
***********************************************************************************************************************/
uint8_t VI530x_Calibration_Xtalk(void)
{
	uint8_t ret = 0;

	NVM_VI530x_XTALK_Calib_Data XtalkCaliData;
	NVM_VI530x_Calibration_Flag CaliFlag;
	XtalkCaliData.Calibration_CG_Maxratio = 0;
	
	ret = VI530x_Stop_Continue_Ranging_Cmd();	
	//printf("Xtalk Calibration Start!\r\n");

	//Xtalk标定
	ret |= VI530x_Xtalk_Calibration();
	//printf("VI530x_Calibration_CG_Pos = %d\r\n",VI530x_Cali_Data.VI530x_Calibration_CG_Pos);
	XtalkCaliData.Calibration_CG_Pos = VI530x_Cali_Data.VI530x_Calibration_CG_Pos;

	//printf("VI530x_Calibration_CG_Maxratio = %d\r\n",VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio);
	XtalkCaliData.Calibration_CG_Maxratio = VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio;

	//printf("VI530x_Calibration_CG_Peak = %d\r\n",VI530x_Cali_Data.VI530x_Calibration_CG_peak);
	XtalkCaliData.Calibration_CG_Peak = VI530x_Cali_Data.VI530x_Calibration_CG_peak;

	//ref_tof标定
	ret |= VI530x_Reftof_Calibration();
	//printf("VI530x_Calibration_Reftof = %d\r\n",VI530x_Cali_Data.VI530x_Calibration_Reftof);
	XtalkCaliData.Calibration_Reftof = VI530x_Cali_Data.VI530x_Calibration_Reftof;
	
	if(XtalkCaliData.Calibration_CG_Maxratio>15)
	{
		CaliFlag.CalibrationFlag = 0x00;
		//printf("Xtalk is too large, Fail!\r\n");
	}
	else
	{
		CaliFlag.CalibrationFlag = 0x01;
		FLASH_Write(FLASH_ADDR_XTALK, (uint8_t*)&XtalkCaliData, sizeof(XtalkCaliData));//保存标定值
		FLASH_Write(FLASH_ADDR_CALIFLAG, (uint8_t*)&CaliFlag, 1);//保存校准标志位
		//printf("Xtalk Calibration Complete!\r\n");
	}
	//开关温度校准:0x01，0x00关
	ret |= VI530x_Set_Sys_Temperature_Enable(0x01);
	//开启连续测距
	ret = VI530x_Start_Continue_Ranging_Cmd();	
	return ret;
	
}

/***********************************************************************************************************************
* Function Name: VI530x_Calibration_Offset
* Description  : Offset标定
* Arguments    : None
* Return Value : ret
***********************************************************************************************************************/
uint8_t VI530x_Calibration_Offset(void)
{
	uint8_t ret = 0;
	uint8_t data_buff[10];

	NVM_VI530x_Offset_Calib_Data OffsetCaliData;
	NVM_VI530x_Calibration_Flag CaliFlag;
	
	ret = VI530x_Stop_Continue_Ranging_Cmd();
	//printf("%d mm Offset Calibration Start!\r\n",VI530x_OFFSET_DISTANCE);

	//Offset 标定
	//在10cm做offset标定,如果在其他位置,如3cm，则把第一个参数改成30
	ret |= VI530x_Offset_Calibration(VI530x_OFFSET_DISTANCE);
//	if(ret == 0)
//	{
//	//printf("VI530x_Calibration_Offsetf = %f\r\n",VI530x_Cali_Data.VI530x_Calibration_Offset);
//	}
//	else
//	{
//		VI530x_Cali_Data.VI530x_Calibration_Offset = 0;
//	//	printf("Offset Calibration Fail!\r\n");
//	}
		
	//customer save gCali_data   //保存标定参数
	OffsetCaliData.Calibration_Offset = VI530x_Cali_Data.VI530x_Calibration_Offset;
	FLASH_Write(FLASH_ADDR_OFFSET, (uint8_t*)&OffsetCaliData, sizeof(OffsetCaliData));
	
	FLASH_ReadByte(FLASH_ADDR_CALIFLAG, (uint8_t*)&CaliFlag, 1);//读取校准标志位
	CaliFlag.CalibrationFlag |= 0x02;
	FLASH_Write(FLASH_ADDR_CALIFLAG, (uint8_t*)&CaliFlag, 1);//保存校准标志位
			
//	printf("Offset Calibration Complete!\r\n");

	//开关温度校准:0x01，0x00关
	ret |= VI530x_Set_Sys_Temperature_Enable(0x01);
	//开启连续测距
	ret = VI530x_Start_Continue_Ranging_Cmd();	
	return ret;
}


/***********************************************************************************************************************
* Function Name: VI530x_HAL_Init
* Description  : This function initializes the HAL module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void VI530x_HAL_Init(void)
{
	AVDD_EN_Init();
	VI530x_XSHUT_Init();
	VI530x_GPIO0_Init();
	VI530x_IIC_Init();
//	EVB_LED_Init();
//	EVB_UART_Init(115200);
}




/***********************************************************************************************************************
* Function Name: VI530x_HAL_Start
* Description  : This function starts the HAL module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void VI530x_HAL_Start(void)
{

}

/***********************************************************************************************************************
* Function Name: VI530x_HAL_Stop
* Description  : This function stops the HAL module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void VI530x_HAL_Stop(void)
{

   
}






