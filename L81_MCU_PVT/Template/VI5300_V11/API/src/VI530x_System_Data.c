#include "VI530x_System_Data.h"


uint8_t VI530x_Set_Sys_Integral_Time(uint32_t integral_time)
{
	uint8_t ret = 0;
	ret |= VI530x_Set_Digital_Clock_Dutycycle();
	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);

	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x03);

	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x01);

	//小端模式
	ret |= VI530x_IIC_Write_One_Byte(0x0F, (integral_time & 0xff));

	ret |= VI530x_IIC_Write_One_Byte(0x10, ((integral_time >> 8) & 0xff));

	ret |= VI530x_IIC_Write_One_Byte(0x11, ((integral_time >> 16) & 0xff));

	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);

	VI530x_Delay_Ms(5);
	return ret;
}

uint8_t VI530x_Get_Sys_Integral_Time(uint32_t *integral_time)
{
	uint8_t ret = 0;
	uint8_t buff[3];

	ret |= VI530x_Set_Digital_Clock_Dutycycle();

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);

	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x03);

	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x01);

	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);

	VI530x_Delay_Ms(5);
	ret |= VI530x_IIC_Read_X_Bytes(0x0C, buff,3); //小端模式

	*integral_time = (buff[2] << 16) + (buff[1] << 8) + (buff[0] << 0);

	return ret;
}

/**
 * @brief  测距后延时通过帧率计算写入
 * @param  delay_time：测距后延时时间
 * @retval uint8_t
 */

uint8_t VI530x_Set_Sys_DelayTime(uint16_t delay_time)
{
	uint8_t ret = 0;
	
	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);

	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x02);

	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x04);

	ret |= VI530x_IIC_Write_One_Byte(0x0F, (((uint16_t)delay_time >> 8) & 0xff));

	ret |= VI530x_IIC_Write_One_Byte(0x10, ((uint16_t)delay_time & 0xff));

	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);

	VI530x_Delay_Ms(5);

	return ret;
}



uint8_t VI530x_Get_Sys_DelayTime(uint16_t *delay_time)
{
	uint8_t ret = 0;
	uint8_t buff[2];

	ret |= VI530x_Set_Digital_Clock_Dutycycle();

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);

	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x02);

	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x04);

	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);

	VI530x_Delay_Ms(5);
	ret |= VI530x_IIC_Read_X_Bytes(0x0C, buff,2); //小端模式
	
	*delay_time = buff[1] + (buff[0]<<8);
	
	return ret;
}


uint8_t VI530x_Set_FPS(uint8_t data_fps)
{
	uint8_t ret = 0;
	
	double Post_pulse_time = 0;
	double Post_pulse_delay = 0;
	
	uint32_t integral_time = 0;
	
	if(data_fps == 0)
	{
		return Result_ERROR;
	}
	
	ret |= VI530x_Get_Sys_Integral_Time(&integral_time);

	Post_pulse_time = integral_time * 146.3 / 1000 / 1000 + 1.3;
	
	Post_pulse_delay = (1000 / data_fps - Post_pulse_time) * 10;
	
	if( Post_pulse_delay < 0 )
	{
		Post_pulse_delay = 0;
	}

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);

	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x02);

	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x04);

	ret |= VI530x_IIC_Write_One_Byte(0x0F, (((uint16_t)Post_pulse_delay >> 8) & 0xff));

	ret |= VI530x_IIC_Write_One_Byte(0x10, ((uint16_t)Post_pulse_delay & 0xff));

	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);

	VI530x_Delay_Ms(5);

	return ret;
}

uint8_t VI530x_Get_FPS(uint8_t *data_fps)
{
	uint8_t ret = 0;
	
	double Post_pulse_time = 0;
	double Post_pulse_delay = 0;
	
	uint32_t integral_time = 0;
	uint8_t buff[2];

	ret |= VI530x_Set_Digital_Clock_Dutycycle();

	ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);

	ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x02);

	ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x04);

	ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);

	VI530x_Delay_Ms(5);
	ret |= VI530x_IIC_Read_X_Bytes(0x0C, buff,2); //小端模式
	
	Post_pulse_delay = buff[1] + (buff[0]<<8);	
	
	ret |= VI530x_Get_Sys_Integral_Time(&integral_time);

	Post_pulse_time = integral_time * 146.3 / 1000 / 1000 + 1.3;	
	
	*data_fps = 1000 / (Post_pulse_delay / 10 + Post_pulse_time);
	
	return ret;
}

uint8_t VI530x_Set_Integralcounts_Frame(uint8_t fps, uint32_t intecoutns)
{
  uint8_t ret = 0;
	uint32_t inte_time = 0;
	uint32_t fps_time = 0;
	int32_t delay_time = 0;
	uint16_t delay_counts = 0;

	inte_time = intecoutns *1463/10;
	fps_time = 1000000000/fps;
	delay_time = fps_time - inte_time -3000000;//1600000
	if( delay_time <= 0 )
	{
		delay_counts = 0;
	}
	else
	{
		delay_counts = (uint16_t)(delay_time/40900);
	}

  ret = VI530x_Set_Sys_Integral_Time(intecoutns);
  ret = VI530x_Set_Sys_DelayTime(delay_counts);

  return ret;
}

    
uint8_t VI530x_Set_Sys_Reftof(uint16_t reftof)
{
    uint8_t ret = 0;
    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0D, 2);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x17);
	
		ret |= VI530x_IIC_Write_One_Byte(0x0F, ((reftof >> 8)& 0xFF));

		ret |= VI530x_IIC_Write_One_Byte(0x10, (reftof & 0xFF));
    
    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
    
		VI530x_Delay_Ms(5);
    return ret;
}

uint8_t VI530x_Get_Sys_Reftof(uint16_t *reftof)
{
    uint8_t ret = 0;

    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0D, 2);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x17);
	
    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
    
		VI530x_Delay_Ms(5);
	
		ret |= VI530x_IIC_Read_X_Bytes(0x0C,(uint8_t *)reftof,2);
	
    return ret;
}


uint8_t VI530x_Set_Sys_Histogram_MA_Window_Data(uint8_t *setting_buff)
{
    uint8_t ret = 0;
    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);
    ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x08);
    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x06);
    ret |= VI530x_IIC_Write_X_Bytes(0x0F, setting_buff, 8);
    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
		VI530x_Delay_Ms(5);
		
    return ret;
}


uint8_t VI530x_Get_Sys_Histogram_MA_Window_Data(uint8_t *getting_buff)
{
    uint8_t ret = 0,i = 0;
    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
    ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x08);
    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x06);
    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
    VI530x_Delay_Ms(5);
    ret |= VI530x_IIC_Read_X_Bytes(0x0C, getting_buff, 8);
	
		getting_buff[8] = 0;
	
		for(i = 0;i < 8; i++)
		{
			getting_buff[8] += ((getting_buff[i] & 0x0F)+((getting_buff[i] >> 4) & 0x0F));
		}	
		
    return ret;
}


uint8_t VI530x_Set_Sys_Temperature_Enable(uint8_t status)
{
    uint8_t ret = 0;
    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x0E);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0F, status);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
		VI530x_Delay_Ms(5);
    return ret;
}


uint8_t VI530x_Get_Sys_Temperature_Enable(uint8_t *status)
{
    uint8_t ret = 0;
    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x0E);
	
    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
		VI530x_Delay_Ms(5);
    ret |= VI530x_IIC_Read_One_Byte(0x0C, status);

    return ret;
}


uint8_t VI530x_Set_Sys_Limit_Distance(uint16_t distance)
{
    uint8_t ret = 0;

    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x02);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x0F);
    //大端模式
    ret |= VI530x_IIC_Write_One_Byte(0x0F, ((distance >> 8) & 0xFF));
		
    ret |= VI530x_IIC_Write_One_Byte(0x10, (distance & 0xFF));

    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	
    VI530x_Delay_Ms(5);

    return ret;
}


uint8_t VI530x_Get_Sys_Limit_Distance(uint16_t *distance)
{
    uint8_t ret = 0;
  	uint8_t buff[2];

    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x02);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x0F);

    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	
    VI530x_Delay_Ms(5);	
    //大端模式
    ret |= VI530x_IIC_Read_X_Bytes(0x0C,buff,2);
		
		*distance = (buff[1] << 8) + (buff[0] << 0);
    return ret;
}


uint8_t VI530x_Set_Sys_CG_Maxratio(uint8_t maxratio)
{
    uint8_t ret = 0;

    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x1A);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0F, maxratio);

    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
    VI530x_Delay_Ms(5);
    return ret;
}


uint8_t VI530x_Get_Sys_CG_Maxratio(uint8_t *maxratio)
{
    uint8_t ret = 0;

    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x1A);
	
    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
    VI530x_Delay_Ms(5);   
	
    ret |= VI530x_IIC_Read_One_Byte(0x0C, maxratio);


    return ret;
}

uint8_t VI530x_Set_Sys_CK(uint8_t ck)
{
    uint8_t ret = 0;

    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x1B);
  
    ret |= VI530x_IIC_Write_One_Byte(0x0F, ck);

    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
    VI530x_Delay_Ms(5);
    return ret;
}


uint8_t VI530x_Get_Sys_CK(uint8_t *ck)
{
    uint8_t ret = 0;

    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x1B);
	
    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
    VI530x_Delay_Ms(5);   
	
    ret |= VI530x_IIC_Read_One_Byte(0x0C, ck);

    return ret;
}


uint8_t VI530x_Set_Sys_Xtalk_Position(uint8_t xtalk_position)
{
    uint8_t ret = 0;

    ret |= VI530x_Set_Digital_Clock_Dutycycle();
    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);

    ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);

    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x00);

    ret |= VI530x_IIC_Write_One_Byte(0x0F, xtalk_position);

    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	
		VI530x_Delay_Ms(5);

    return ret;
}


uint8_t VI530x_Get_Sys_Xtalk_Position(uint8_t *xtalk_position)
{
    uint8_t ret = 0;

    ret |= VI530x_Set_Digital_Clock_Dutycycle();
    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);

    ret |= VI530x_IIC_Write_One_Byte(0x0D, 0x01);

    ret |= VI530x_IIC_Write_One_Byte(0x0E, 0x00);
	
    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
	
		VI530x_Delay_Ms(5);
	
    ret |= VI530x_IIC_Read_One_Byte(0x0C, xtalk_position);

    return ret;
}

uint8_t VI530x_Write_System_Data(uint8_t offset_addr, uint8_t *buff, uint8_t len)
{
    uint8_t ret = 0;
    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x01);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0D, len);
    
    
    ret |= VI530x_IIC_Write_One_Byte(0x0E, offset_addr);
    

    ret |= VI530x_IIC_Write_X_Bytes(0x0F, buff, len);


    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
    
		VI530x_Delay_Ms(5);
    return ret;
}


uint8_t VI530x_Read_System_Data(uint8_t offset_addr, uint8_t *buff, uint8_t len)
{
    uint8_t ret = 0;

    ret |= VI530x_Set_Digital_Clock_Dutycycle();

    ret |= VI530x_IIC_Write_One_Byte(0x0C, 0x00);
 
    ret |= VI530x_IIC_Write_One_Byte(0x0D, len);
    
    ret |= VI530x_IIC_Write_One_Byte(0x0E, offset_addr);
    

    ret |= VI530x_IIC_Write_One_Byte(0x0A, 0x09);
    
    VI530x_Delay_Ms(5);

    ret |= VI530x_IIC_Read_X_Bytes(0x0C, buff, len);
	
    return ret;
}




