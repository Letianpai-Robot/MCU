#include "VI530x_API.h"
#include "VI530x_System_Data.h"
#include "L81_inf.h"

uint8_t VI530x_GPIO_Interrupt_status = 0;

uint8_t VI530x_IIC_Dev_Addr_Now = VI530x_IIC_DEV_ADDR;

VI530x_Calibration_TypeDef VI530x_Cali_Data;
uint8_t VI530x_Chip_Version = 0x30;


void VI530x_Chip_PowerON(void)
{
  // Xshut pin
  VI530x_XSHUT_Enable(0); // set 0
  VI530x_Delay_Ms(10);											 // delay 10ms
  VI530x_XSHUT_Enable(1);	 // set 1
  VI530x_Delay_Ms(10);											 // delay 10ms
}

void VI530x_Chip_PowerOFF(void)
{
  VI530x_XSHUT_Enable(0);
}


uint8_t VI530x_Set_Digital_Clock_Dutycycle(void)
{
	uint8_t ret = 0;
	ret |= VI530x_IIC_Write_One_Byte(VAN_REG_PW_CTRL, 0x0F);

	ret |= VI530x_IIC_Write_One_Byte(VAN_REG_PW_CTRL, 0x0E);
	VI530x_Delay_Ms(5);
	return ret;
}

VI530x_Status VI530x_Wait_For_CPU_Ready(void)
{
  VI530x_Status Status = VI530x_OK;
  uint8_t stat;
  int retry = 0;

  do
  {
    VI530x_Delay_Ms(1); // delay 1ms
    Status = VI530x_IIC_Read_One_Byte(0x02, &stat);
  }
  while ((retry++ < 20) && (stat & 0x01));
  if (retry >= 20)
  {
		//printf("CPU Busy stat = %d\n", stat);
    return VI530x_BUSY;
  }

  return Status;
}

uint8_t VI530x_Clear_Interrupt(void)
{
	uint8_t ret = 0;
	VI530x_GPIO_Interrupt_status = 0;

	return VI530x_IIC_Read_One_Byte(VAN_RET_INT_STATUS, &ret);
}

uint8_t VI530x_Get_And_Clear_Interrupt(uint8_t *interrupt_status)
{
	uint8_t ret = 0, temp_status = 0;


	if(!VI530x_Cali_Data.VI530x_Interrupt_Mode_Status)
	{
		ret |= VI530x_IIC_Read_One_Byte(VAN_RET_INT_STATUS, &temp_status);
	}
	if(VI530x_GPIO_Interrupt_status || (temp_status & 0x01))
	{
		*interrupt_status = 0x01;
		VI530x_GPIO_Interrupt_status = 0;
	}
	else
	{
		*interrupt_status = 0x00;
	}
	return ret;
}

uint8_t VI530xWriteCommand(uint8_t cmd)
{
	return VI530x_IIC_Write_One_Byte(VAN_REG_CMD, cmd);
}

uint8_t VI530x_Start_Single_Ranging_Cmd(void)
{
	uint8_t ret = 0;
	ret |= VI530x_Clear_Interrupt();

	ret |= VI530x_Set_Digital_Clock_Dutycycle();

	ret |= VI530xWriteCommand(VAN_START_RANG_CMD);

	return ret;
}

uint8_t VI530x_Start_Continue_Ranging_Cmd(void)
{
	uint8_t ret = 0;

	ret |= VI530x_Clear_Interrupt();
	ret |= VI530x_Set_Digital_Clock_Dutycycle();

	ret |= VI530xWriteCommand(0x0F);

	return ret;
}

uint8_t VI530x_Stop_Continue_Ranging_Cmd(void)
{
	uint8_t ret = 0;

	ret |= VI530x_Clear_Interrupt();
	ret |= VI530x_Set_Digital_Clock_Dutycycle();
	if(VI530x_Cali_Data.VI530x_Power_Manage_Status)
	{

		ret |= VI530xWriteCommand(0x1F);
	}
	else
	{
		ret |= VI530xWriteCommand(0X00);
	}	
	
	VI530x_Delay_Ms(10);
	return ret;
}

uint8_t VI530x_Chip_Register_Init(uint8_t *chip_version)
{
	uint8_t ret = 0;
	uint8_t version_31_cnt = 5;
	uint8_t version_30_cnt = 5;

	do
	{
		ret |= VI530x_IIC_Read_One_Byte(0x38, chip_version);
		if (*chip_version == 0x30)
		{
			version_31_cnt--;
		}
		else
		{
			version_30_cnt--;
		}
	} while (version_31_cnt && version_30_cnt); // 0x30 v3.1, 0x00 v3.0

	if (version_31_cnt == 0)
	{
		*chip_version = 0x31;
	}
	else
	{
		*chip_version = 0x30;
	}

	/*****************************************************************/

	if(VI530x_Cali_Data.VI530x_Power_Manage_Status)
	{
		ret |= VI530x_IIC_Write_One_Byte(VAN_REG_SYS_CFG, 0x0C);
	}
	else
	{

		ret |= VI530x_IIC_Write_One_Byte(VAN_REG_SYS_CFG, 0x08);
	}
	
	ret |= VI530x_IIC_Write_One_Byte(0x07, 0x00); //閿熸枻鎷蜂綅PD閿熸枻鎷锋媷閿熸枻鎷烽敓锟?
	ret |= VI530x_IIC_Write_One_Byte(0x07, 0x01);
	ret |= VI530x_IIC_Write_One_Byte(0x07, 0x00); //閿熻?闈╂嫹A0閿熸枻鎷锋媷閿熸枻鎷烽敓琛楋拷
	ret |= VI530x_IIC_Write_One_Byte(0x04, 0x21);
	ret |= VI530x_IIC_Write_One_Byte(0x05, 0x0e);

	ret |= VI530x_IIC_Write_One_Byte(0x08, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x37, 0x80);
	ret |= VI530x_IIC_Write_One_Byte(0x38, 0x30); // v3.0 0x00
	ret |= VI530x_IIC_Write_One_Byte(0x39, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x3a, 0x30); // v3.0 0x00
	ret |= VI530x_IIC_Write_One_Byte(0x3b, 0x80);
	ret |= VI530x_IIC_Write_One_Byte(0x3c, 0x80);
	ret |= VI530x_IIC_Write_One_Byte(0x3d, 0x80);
	ret |= VI530x_IIC_Write_One_Byte(0x3e, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x3f, 0x00);
	ret |= VI530x_IIC_Write_One_Byte(0x07, 0x0e);
	ret |= VI530x_IIC_Write_One_Byte(0x07, 0x0f);

	return ret;
}




uint8_t VI530x_Xtalk_Calibration(void)
{
	uint8_t status = 0;
	uint8_t ret = 0;
	uint16_t time_out_cnt = 1000;
//	uint16_t time_out_cnt = 10;
	uint8_t xtalk_buff[10];

	ret |= VI530x_Clear_Interrupt();
	ret |= VI530x_Set_Digital_Clock_Dutycycle();
	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x0D);

	while (time_out_cnt--)
	{
		VI530x_Delay_Ms(10);
		ret |= VI530x_Get_And_Clear_Interrupt(&status);
		if (status)
		{
			VI530x_Delay_Ms(10);
			ret |= VI530x_IIC_Read_One_Byte(0x08, &status);
			if (status == 0xAA)
			{
				ret |= VI530x_IIC_Read_X_Bytes(0x0C, xtalk_buff, 5);

        VI530x_Cali_Data.VI530x_Calibration_CG_Pos = xtalk_buff[0];
        VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio = xtalk_buff[3];
				VI530x_Cali_Data.VI530x_Calibration_CG_peak = (uint16_t)((((uint16_t)xtalk_buff[2])<<8) |(( (uint16_t)xtalk_buff[1])));
        ret |= VI530x_Set_Sys_Xtalk_Position(VI530x_Cali_Data.VI530x_Calibration_CG_Pos);
        ret |= VI530x_Set_Sys_CG_Maxratio(VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio);
				break;
			}
			else
			{
				return Result_ERROR;
			}
		}
		if (time_out_cnt == 0)
		{
			return Result_ERROR;
		}		
	}
	return ret;
}


uint8_t VI530x_Get_CG_Maxratio(uint16_t cg_peak,uint8_t *maxratio)
{	

	uint8_t buff[10];
	uint8_t ret = 0;
	uint16_t EP = 0;
	uint64_t temp = 0;
	uint32_t intecounts = 0;
	EP = 11431;
	ret |= VI530x_Get_Sys_Integral_Time(&intecounts);
	ret |= VI530x_Get_Sys_Histogram_MA_Window_Data(buff);	
	
	temp = (intecounts * cg_peak / 65536/3 * buff[8] + EP/2)/ EP*4/3;

	*maxratio = temp + 2;
	ret |= VI530x_Set_Sys_CG_Maxratio(*maxratio);	
	
	return ret;
}

uint8_t VI530x_Offset_Calibration(uint16_t mili)
{
  uint8_t ret = 0;

  uint8_t get_data_cnt = 0;
  uint8_t confidence = 0;
  uint8_t databuff[32] = {0};
  uint8_t interrupt_status = 0;
  uint16_t noise = 0;
  uint16_t time_out_cnt = 0;

  int16_t raw_tof = 0;
  int32_t sum_tof = 0, sum_bias = 0;
  uint32_t bias = 0;
  uint32_t ref_peak = 0, peak1 = 0;
  uint32_t intecounts = 0;

  int16_t ret_offset = 0;

  uint32_t integral_times = 0;
  float offset_mili = 0.0;

  uint8_t get_data_total_times = 30;
  uint8_t start_get_data_times = 10;

	ret |= VI530x_Set_Sys_Temperature_Enable(0x00);
	ret |= VI530x_Clear_Interrupt();
	ret |= VI530x_Start_Continue_Ranging_Cmd();

  while(1)
  {
    VI530x_Delay_Ms(5);
    ret |= VI530x_Get_And_Clear_Interrupt(&interrupt_status);
    if (interrupt_status)
    {

      ret |= VI530x_IIC_Read_X_Bytes(0x0C, databuff, 32);
	  time_out_cnt = 0;
	  memcpy(&intecounts,&databuff[22],4);
	  intecounts = intecounts & 0x00FFFFFF;

	  memcpy(&noise,&databuff[26],2);
	  memcpy(&peak1,&databuff[28],4);
	  memcpy(&ref_peak,&databuff[8],4);
	  memcpy(&raw_tof,&databuff[12],2);	
	  bias = VI530x_V10_Calculate_Pileup_Bias(ref_peak,noise,intecounts);
	  confidence = VI530x_Calculate_Confidence(noise, peak1, intecounts); 
			
      if(get_data_cnt > start_get_data_times)
	  {
		sum_tof += raw_tof;
		sum_bias += bias;
	  }
			
      get_data_cnt++;
			//Debug_Mode
	  #ifdef Debug_Mode
			printf("Offset:raw_tof = %2d, ref_peak = %4d, peak = %4d, noise = %4d, bias = %2d, intecounts = %4d, cnt = %d\r\n",
					raw_tof,ref_peak,peak1,noise,bias,intecounts,get_data_cnt);
	  #endif
			if( confidence != 100)
			{	
				ret = VI530x_ERROR;
				break;
			}		
    }
    if(get_data_cnt > get_data_total_times)
    {
	 offset_mili = (float)(sum_tof+sum_bias ) / (get_data_total_times-start_get_data_times) - mili;
      break;
    }
	time_out_cnt++;
	if(time_out_cnt > 200)
	{
	  ret = VI530x_ERROR;
	  break;
	}
  }
  ret |= VI530x_Stop_Continue_Ranging_Cmd();
  VI530x_Cali_Data.VI530x_Calibration_Offset = offset_mili;
  return ret;
}

uint8_t VI530x_Reftof_Calibration(void)
{
    uint8_t ret = 0;
    uint8_t get_data_total_times = 0;
    uint8_t start_get_data_times = 0;
    uint8_t get_data_cnt = 0;
    uint8_t databuff[2];
    uint8_t interrupt_status = 0;
    uint16_t sum_reftof = 0;
    uint16_t abnormal_state_cnt = 0;

    get_data_total_times = 30;
    start_get_data_times = 10;
		
	
	ret |= VI530x_Set_Sys_Temperature_Enable(0x00);
	ret |= VI530x_Clear_Interrupt();
    ret |= VI530x_Start_Continue_Ranging_Cmd();

	ret |= VI530x_IIC_Read_X_Bytes(0x20, databuff, 2);

    sum_reftof += ((((int16_t)databuff[1]) << 8) | (((int16_t)databuff[0])));
    do
    {

        ret |= VI530x_Get_And_Clear_Interrupt(&interrupt_status);

        if (!interrupt_status)
        {
            
			VI530x_Delay_Ms(1);
            abnormal_state_cnt++;
            if (abnormal_state_cnt > 1000)
            {
            
                return Result_ERROR;
            }
            continue;
        }
        abnormal_state_cnt = 0;
        if (get_data_cnt >= start_get_data_times && get_data_cnt < get_data_total_times)
        {    
            ret |= VI530x_IIC_Read_X_Bytes(0x20, databuff, 2);

            sum_reftof += ((((int16_t)databuff[1]) << 8) | (((int16_t)databuff[0])));
        }

        if (get_data_cnt < get_data_total_times)
        {
            get_data_cnt++;
        }
    } while (get_data_cnt < get_data_total_times);

    ret |= VI530x_Stop_Continue_Ranging_Cmd();

    VI530x_Cali_Data.VI530x_Calibration_Reftof = (sum_reftof) / (get_data_total_times - start_get_data_times);
	ret |= VI530x_Set_Sys_Reftof(VI530x_Cali_Data.VI530x_Calibration_Reftof);
    return ret;
}


uint8_t VI530x_Get_Histogram_Data(uint8_t tdc, uint8_t *histogram_buff)
{
    uint8_t ret = 0;

    uint8_t i = 0;
    uint8_t reg_pw_ctrl = 0;
    uint8_t reg_sys_cfg = 0;
    uint16_t ram_addr_base = 0;

    if (tdc <= 9)
    {
        ram_addr_base = 0x1000 + 0x0400 * tdc;

    }
    else if (tdc >= 0xF1 && tdc <= 0xF4)
    {
        ram_addr_base = (tdc - 0xF1) * 0x0400;
    }

    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Read_One_Byte(REG_SYS_CFG, &reg_sys_cfg);
    ret |= VI530x_IIC_Write_One_Byte(REG_SYS_CFG, reg_sys_cfg | (0x01 << 0));
    ret |= VI530x_IIC_Read_One_Byte(REG_PW_CTRL, &reg_pw_ctrl);
    ret |= VI530x_IIC_Write_One_Byte(REG_PW_CTRL, reg_pw_ctrl | (0x01 << 3));               // set otp_ld_done
    ret |= VI530x_IIC_Write_One_Byte(REG_PW_CTRL, reg_pw_ctrl | (0x01 << 3) | (0x01 << 1)); // set otp_ld_done, pw_ctrl_lp
    ret |= VI530x_IIC_Write_One_Byte(REG_MCU_CFG, 0x03);                                    // 0x01 FOR VI530x V3.0
    ret |= VI530x_IIC_Write_One_Byte(REG_CMD, 0x01);
    ret |= VI530x_IIC_Write_One_Byte(REG_SIZE, 0x02);
    ret |= VI530x_IIC_Write_One_Byte(REG_SCRATCH_PAD_BASE + 0x00, *((uint8_t *)(&ram_addr_base)));
    ret |= VI530x_IIC_Write_One_Byte(REG_SCRATCH_PAD_BASE + 0x01, *((uint8_t *)(&ram_addr_base) + 1));

    VI530x_Delay_Ms(1);

    for (i = 0; i < 32; i++)
    {
        ret |= VI530x_IIC_Write_One_Byte(REG_CMD, 0x05);  
        ret |= VI530x_IIC_Write_One_Byte(REG_SIZE, 0x20);  
        VI530x_Delay_Ms(5);

        ret |= VI530x_IIC_Read_X_Bytes(0x0c, &histogram_buff[32 * i], 32);
    }
    ret |= VI530x_IIC_Write_One_Byte(REG_MCU_CFG, 0x07);                        
    ret |= VI530x_IIC_Write_One_Byte(REG_SYS_CFG, reg_sys_cfg & ~(0x01 << 0));  
    ret |= VI530x_IIC_Write_One_Byte(REG_PW_CTRL, reg_pw_ctrl | (0x01 << 1));   
    ret |= VI530x_IIC_Write_One_Byte(REG_PW_CTRL, reg_pw_ctrl);                 

    return ret;
}



//
uint8_t VI530x_Get_Measure_Data(VI530x_MEASURE_TypeDef *measure_data)
{
	uint8_t Interrupt_status = 0;
	uint8_t ret = 0;
	uint8_t data_buff[32] = {0};
	uint16_t time_out_cnt = 3000;
	
	int16_t raw_tof = 0;
	int16_t correction_tof = 0;	
	uint32_t intecounts = 0;
	uint32_t peak = 0;
	uint16_t noise = 0;	
	uint16_t xtalk_count = 0;
	int16_t ref_tof = 0;
	uint32_t ref_peak = 0;

	uint8_t confidence = 0;	
	float bias = 0;

	while (time_out_cnt--)
	{
		ret |= VI530x_Get_And_Clear_Interrupt(&Interrupt_status);
		if (Interrupt_status)
		{
			ret |= VI530x_IIC_Read_X_Bytes(0x0C, data_buff, 32);
			
			memcpy(&ref_peak,&data_buff[8],4);
			memcpy(&ref_tof,&data_buff[20],2);
			memcpy(&raw_tof,&data_buff[12],2);
			memcpy(&intecounts,&data_buff[22],4);
			intecounts = intecounts & 0x00FFFFFF;
			memcpy(&noise,&data_buff[26],2);
			memcpy(&peak,&data_buff[28],4);
			memcpy(&xtalk_count,&data_buff[14],2);

			bias = VI530x_V10_Calculate_Pileup_Bias(ref_peak,noise, intecounts);
			correction_tof  = raw_tof  + bias - VI530x_Cali_Data.VI530x_Calibration_Offset;
			if(correction_tof  < 0)
			{
				correction_tof  = 0;
			}

			confidence = VI530x_Calculate_Confidence(noise, peak, intecounts);
			if( (correction_tof  < 50)&& (peak <800000) )
			{
				confidence = 50;
			}
			measure_data->intecounts = intecounts;
			measure_data->correction_tof = correction_tof;
			measure_data->confidence = confidence;
			measure_data->peak = peak;	
			measure_data->noise = noise;	
			measure_data->xtalk_count = xtalk_count;
			break;
		}
		if (time_out_cnt == 0)
		{
			ret |= VI530x_ERROR;;
		}
		
		VI530x_Delay_Ms(1);
	}

#ifdef Debug_Mode
			printf("AT+RES,ACK\r\n");  
			printf("AT+RES,%3d\r\n", correction_tof);
			printf("AT+RES,end\r\n"); 
	printf("raw_tof = %2d, peak = %4d, ref_tof = %2d, ref_peak = %4d, noise = %4d, bias = %2f, cali_offset = %4f, intecounts = %4d, xtalk_count = %2d\r\n",
			  raw_tof,peak,ref_tof,ref_peak,noise,bias,
				 VI530x_Cali_Data.VI530x_Calibration_Offset,intecounts,xtalk_count);
				
#endif
	return ret;
}

VI530x_Status VI530x_Set_Californiation_Data(float cali_offset)
{
    uint8_t ret = VI530x_OK;

		ret |= VI530x_Set_Sys_Xtalk_Position(VI530x_Cali_Data.VI530x_Calibration_CG_Pos);
		ret |= VI530x_Set_Sys_CG_Maxratio(VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio);
		ret |= VI530x_Set_Sys_Reftof(VI530x_Cali_Data.VI530x_Calibration_Reftof);
		VI530x_Cali_Data.VI530x_Calibration_Offset = cali_offset;
    return ret;
}

VI530x_Status VI530x_Chip_Init(void)
{
	VI530x_Status Status = VI530x_OK;
	uint8_t IIC_ID = 0;

	VI530x_Cali_Data.VI530x_Power_Manage_Status = 0x01; 

  VI530x_Chip_PowerON();
	
	Status |= VI530x_IIC_Read_One_Byte(VAN_REG_IIC_DEV_ADDR, &IIC_ID);
	if(IIC_ID != VI530x_IIC_DEV_ADDR)
	{
		Status = VI530x_ERROR;
		//Debug
//	#ifdef Debug_Mode
//		printf("Check device ID is 0x%2x!\r\n",IIC_ID);
//	#endif
	}
	
#ifdef Change_IIC_Dev_Addr	
	Status |= VI530x_Set_ModelChangeAddr(VI530x_IIC_DEV_ADDR2);			
#endif
	
  Status |= VI530x_Chip_Register_Init(&VI530x_Chip_Version);
	
//#ifdef Debug_Mode
//	printf("VI530x chip version is %x\r\n",VI530x_Chip_Version);
//#endif

  Status |= VI530x_Wait_For_CPU_Ready();
	return Status;
}



static void IICDelay_1us(unsigned int num)
{
	unsigned int time_num;	 
	time_num = num;
    delay_4us(time_num);

}

//#if 0
//uint8_t l81_VI53_test_func(char params[])
//{
//  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
//  uint8_t param_num = 0;
//	 
//	
//	ATcmd_split_params(params, param, &param_num);
//	
//	uint32_t addr = StringToHex(param[0u]);   
//	if (param_num != 0U){
//		printf("AT+RES,ACK\r\n");
//		printf("AT+RES,Err,This command hava no params\r\n");
//		printf("AT+RES,end\r\n");
//		return 0U;
//	}
//	if (param_num == 0U){
//		printf("AT+RES,ACK\r\n");
//		printf("AT+RES,test\r\n");
//		printf("AT+RES,end\r\n");
//	} 
//	distance_measurement_result_test();

//	
//  return 1U;		
//}
//#endif

