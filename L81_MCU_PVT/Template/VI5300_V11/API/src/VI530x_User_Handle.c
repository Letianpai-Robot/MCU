#include "VI530x_User_Handle.h"

#include "VI530x_Common.h"
#include "L81_AT.h"
#include "L81_FMC.h"
#include "L81_inf.h"
#include "L81_infrared_task.h"

#define VI530x_XSHUT_GPIO_Port	GPIOA
#define VI530x_XSHUT_Pin	GPIO_PIN_0

typedef struct
{
	int16_t correction_tof;	
	uint8_t tof_flag;	
}ATTOF_TypeDef;
ATTOF_TypeDef  atresulttof;	

uint8_t auto_put = 0;
uint8_t VI530x_IIC_Read_One_Byte(uint8_t addr, uint8_t *value)
{		
	
	uint8_t ret = 0;
	
	ret = IIC_Read_X_Bytes(addr,value,1);

	if(ret != 0)
		ret =  VI530x_IIC_ERROR;
	return ret;

}

uint8_t VI530x_IIC_Read_X_Bytes(uint8_t addr, uint8_t *value, uint16_t tlen)
{
	uint8_t ret = 0;
	
	ret = IIC_Read_X_Bytes(addr,value,tlen);
	
	if(ret != 0)
		ret =  VI530x_IIC_ERROR;
	return ret;

}

uint8_t VI530x_IIC_Write_One_Byte(uint8_t addr, uint8_t value)
{	
	uint8_t ret = 0;
	
	ret = IIC_Write_X_Bytes(addr,&value,1);
	
	if(ret != 0)
		ret =  VI530x_IIC_ERROR;
	return ret;

}


uint8_t VI530x_IIC_Write_X_Bytes(uint8_t addr, uint8_t *pValue, uint16_t tlen)
{	
	uint8_t ret = 0;
	
	ret = IIC_Write_X_Bytes(addr,pValue,tlen);
	
	if(ret != 0)
		ret =  VI530x_IIC_ERROR;
	return ret;

}

void VI530x_Delay_Ms(uint16_t nMs)
{
	delay_1ms(nMs);
}


void VI530x_GPIO_Interrupt_Handle(void)
{
	if(VI530x_Cali_Data.VI530x_Interrupt_Mode_Status)
	{
		VI530x_GPIO_Interrupt_status = 1;
	}
}

void VI530x_XSHUT_Enable(uint8_t state)
{
	if(state)
	{
		gpio_bit_write(VI530x_XSHUT_GPIO_Port, VI530x_XSHUT_Pin, SET);
	}
	else
	{
		gpio_bit_write(VI530x_XSHUT_GPIO_Port, VI530x_XSHUT_Pin, RESET);
	}
}

void VI530x_init()
{
	uint8_t ret = 0;

	VI530x_Cali_Data.VI530x_Interrupt_Mode_Status = 0x00;		
	ret |= VI530x_Chip_Init();
    ret |= VI530x_Download_Firmware((uint8_t *)VI5300_M31_firmware_buff, FirmwareSize());
	
	fwdgt_counter_reload();
		//ret |= VI530x_Calibration_Read();
	VI530x_Cali_Data.VI530x_Calibration_Offset = 0;
		VI530x_Cali_Data.VI530x_Calibration_CG_Pos = 2;
		VI530x_Cali_Data.VI530x_Calibration_CG_Maxratio = 25;
	VI530x_Cali_Data.VI530x_Calibration_Reftof = 483;
	 fwdgt_counter_reload();
	ret |= VI530x_Set_Californiation_Data(VI530x_Cali_Data.VI530x_Calibration_Offset);
	fwdgt_counter_reload();
	ret |= VI530x_Set_Sys_Temperature_Enable(0x03);
	
}

void  distance_measurement_raw_result()
{
	uint8_t ret = 0,temp;
	VI530x_MEASURE_TypeDef result;
	ret |= VI530x_Xtalk_Calibration();
	ret |= VI530x_Reftof_Calibration();
	ret |= VI530x_Offset_Calibration(VI530x_OFFSET_DISTANCE);
	ret |= VI530x_Set_Sys_Temperature_Enable(0x03);    //0718 lsl+
	ret |= VI530x_Start_Continue_Ranging_Cmd();	

    temp  = 0;
	fwdgt_counter_reload();
	ret = VI530x_Get_Measure_Data(&result);
	atresulttof.correction_tof = result.correction_tof;
	atresulttof.tof_flag = 1;
}
  
 
 int16_t distance_measurement_result(void)   //0620-rjq  void -> int16_t
 {
	uint8_t ret = 0,temp;
	VI530x_MEASURE_TypeDef result;
//	 fwdgt_counter_reload();    //0718 lsl-
//	ret |= VI530x_Calibration_Read();
//	 fwdgt_counter_reload();
//	ret |= VI530x_Set_Californiation_Data(VI530x_Cali_Data.VI530x_Calibration_Offset);
	fwdgt_counter_reload();
//	 ret |= VI530x_Start_Continue_Ranging_Cmd();
	 ret |= VI530x_Start_Single_Ranging_Cmd();
	 VI530x_Delay_Ms(33); //33ms    //0718 lsl+
   	fwdgt_counter_reload();
	ret |= VI530x_Get_Measure_Data(&result);
	if(!ret)
	{
	temp = l81_fmc_read(TOF_ADDR);
	if(temp<100)
		{
			result.correction_tof = result.correction_tof + temp;
		}
	else {
 			temp = temp - 100;
			result.correction_tof = result.correction_tof- temp;
		}
	if(result.correction_tof>4000)
		 result.correction_tof = 3999;
	//if(!ret)
		return result.correction_tof;
//	if(!ret)
//		{
//			printf("AT+RES,ACK\r\n");  
//			printf("AT+RES,%d\r\n", result.correction_tof);
//			printf("AT+RES,end\r\n"); 
	}
	return -1000;
}
 

uint8_t l81_VI53_dem_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	 
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command hava no params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	if (auto_put == 1){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,auto_put is running\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	int16_t correction_tof = distance_measurement_result();   //0620-rjq  void -> int16_t
	if(correction_tof != -1000)
		{
			printf("AT+RES,ACK\r\n");  
			printf("AT+RES,%d\r\n", correction_tof);
			printf("AT+RES,end\r\n"); 
		}
	else
	{
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,get correction_tof err,plause retry\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
		
  return 1U;
}

uint8_t l81_VI53_cal_read_raw_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 1U){ 
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
	
	uint32_t cmd = String2Int(param[0]);
		
	if ((cmd < 1U) || (cmd > 2u)) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,err cmd value eg:[1~2]\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	switch (cmd)
	{
    case 1:
        printf("AT+RES,ACK\r\n");
        printf("AT+RES,end\r\n");
		atresulttof.tof_flag = 0;
		distance_measurement_raw_result();
    break;
    case 2:
//			if (auto_put == 1){
//				printf("AT+RES,ACK\r\n");
//				printf("AT+RES,Err,auto_put is running\r\n");
//				printf("AT+RES,end\r\n");
//				return 0U;
//			}
			if(atresulttof.tof_flag == 1)
			{
				printf("AT+RES,ACK\r\n");
				printf("AT+RES,%d\r\n", atresulttof.correction_tof);
				printf("AT+RES,end\r\n");
			}
			else{
			 printf("AT+RES,ACK\r\n");
			 printf("AT+RES,Err,not ready\r\n");
			 printf("AT+RES,end\r\n");
			}
      break;
    default:
      break;
  }
    
    return 1U;	
}

uint8_t l81_VI53_TofSet_func(char params[])   //0620-rjq add
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 2U){ 
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
	
	uint32_t cmd = String2Int(param[0]);
	uint32_t tofset_cmd = String2Int(param[1]);
	if (tofset_cmd < 50) tofset_cmd = 50;
	
	if ((cmd < 0U) || (cmd > 1u)) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,err cmd value eg:[0~1]\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	switch (cmd)
	{
    case 0:
			auto_put = 0;
			infrared_task_stop();
      printf("AT+RES,ACK\r\n");
      printf("AT+RES,end\r\n");
    break;
    case 1:
			auto_put = 1;
			infrared_task_init(tofset_cmd);
			printf("AT+RES,ACK\r\n");
			printf("AT+RES,end\r\n");
      break;
    default:
      break;
  }
    
    return 1U;	
}


/*!
    \brief      string switch to Hex
								ASCII 48->0, 57->9, 65->A, 70->F, 97->a, 102->f
    \param[in]  the string of number list, eg "0x123", must begin with "0x" or "0X"
    \param[out] none
    \retval     int
*/
int32_t StringToHex(const char *str)
{
    uint32_t res = 0u;
    const char *p = str;
	uint8_t i = 8u; 
    uint8_t  flag=0;
	if(*p==0x2D) {
		flag = 1;
		p++;
	}
    else if (!str)
			return 0u;

      while((*p>=48u && *p<=57u) || (*p>=65u && *p<=70u) || (*p>=97u && *p<=102u))
      {
			if (i == 0u)
				break;
			else
				i--;
			
       if(*p>=48u && *p<=57u)//      0~~9 
	   {
				 res = 16u*res + ((*p++)-48u);
	   }
       if(*p>=65u && *p<=70u)//A      ~        F 
         res = 16u*res + (((*p++)-65u) + 10u);
       if(*p>=97 && *p<=102)//a    ~            f
         res = 16u*res + (((*p++)-97u) + 10u);
      }

	if(flag)
	{

		res=res+100;

	}		
    return res;
}
uint8_t l81_VI53_cal_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	 
	
	ATcmd_split_params(params, param, &param_num);
	
	uint32_t addr = StringToHex(param[0u]);  //param 0 store write address
	if (param_num == 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command hava no params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	if (param_num == 1U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,end\r\n");
	} 
	l81_fmc_program(TOF_ADDR,addr);

	
  return 1U;		
}

#if 0
uint8_t l81_VI53_test_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	 
	
	ATcmd_split_params(params, param, &param_num);
	
	uint32_t addr = StringToHex(param[0u]);  
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command hava no params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	if (param_num == 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,test\r\n");
		printf("AT+RES,end\r\n");
	} 
	distance_measurement_result_test();	
  return 1U;		
}
#endif








