/*!
    \file    L81_humiture_task.c
    \brief    
*/

/*
    Copyright (c) 2023, wxf 
*/


#include <stdio.h>
#include "L81_TimTask.h"
#include "L81_humiture_task.h"

#include "gxhtc3iic.h"


/************************************ brief*******************************************/
/* 

*/
/****************************************************************************************************/
#define HUMITURE_DEBUG_EN            0   //debug EN=1 DEN=0
#if HUMITURE_DEBUG_EN
#define HUMITURELog(...) printf(__VA_ARGS__)
#else
#define HUMITURELog(...) 
#endif

T_HUMITURE_PARAM_TYPDEF humiture = {0};
uint32_t num2_16 = 65536;  //  2^16    use to get tem value and hum value
uint16_t read_t = 1;  //  1 read tem ; 0 read hum

uint16_t read_tem()
{
  uint8_t buf[3]= {0};
  uint16_t cmd = GXHTC3_NORMAL_T_CMD;
  uint16_t tem = 0;
  uint8_t err = 0;
  err = gxhtc3_cmd_read_3B(cmd, buf);
  float t = 0;
  
  tem = buf[0];
  tem<<=8;
  tem |= buf[1];
  
  //HUMITURELog("TEM = %02X %02X  err=%d\r\n",tem, buf[2], err);
  if(tem == 0)
	{
		humiture.temperature_check_en = 0;
		return humiture.temperature_last;
	}
	humiture.temperature_last = humiture.temperature_cur;
	if(tem != 0)
	{
		humiture.temperature_check_en = 1;
		
		t = (float)tem;
		humiture.temperature_cur = t *175/num2_16 - 45;
		//HUMITURELog("TEM = %f \r\n",temperature);
	}
  return tem;
}

uint16_t read_hum()
{
  uint8_t buf[3]= {0};
  uint16_t cmd = GXHTC3_NORMAL_H_CMD;
  uint16_t hum = 0;
  uint8_t err = 0;
  err = gxhtc3_cmd_read_3B(cmd, buf);
  float h = 0;
  
  hum = buf[0];
  hum<<=8;
  hum |= buf[1];
  
  //HUMITURELog("HUM = %02X %02X  err=%d\r\n",hum, buf[2], err);
  if(hum == 0)
	{
		humiture.humidity_check_en = 0;
		return humiture.humidity_last;
	}
	humiture.humidity_last = humiture.humidity_cur;
	if(hum != 0)
	{
		humiture.humidity_check_en =1;
		humiture.humidity_new = 1;
		h = (float)hum;
		humiture.humidity_cur = h * 100 / num2_16;
		//HUMITURELog("HUM = %f \r\n",humidity);
	}
	
  return hum;
}
float get_tem()
{
	humiture.temperature_new = 0;
	return humiture.temperature_cur;
}

float get_hum()
{
	humiture.humidity_new = 0;
	return humiture.humidity_cur;
}
T_HUMITURE_PARAM_TYPDEF get_humiture()
{
	return humiture;
}

void humiture_task_task()
{
	if(read_t)
	{
		read_tem();
		read_hum();
		read_t = 0;
	}else{
		read_hum();
		read_tem();
		read_t = 1;
	}
}


void humiture_task_init()
{
  gxhtc3_iic_init();
	humiture.temperature_last = 0;
	humiture.temperature_cur = 0;
	humiture.temperature_check_en = 1;
	humiture.humidity_last = 0;
	humiture.humidity_cur = 0;
	humiture.humidity_check_en = 1;
	
	
  humiture_task_task();
  //task init
  l81_tim_task_creat(ID_HUMITURE, TIM_TASK_CYCLE_ALL, HUMITURE_CYCLE_TIM_CHECK, NULL, humiture_task_task);

  //start task
  l81_tim_task_en(ID_HUMITURE);

  HUMITURELog("humiture tast init\r\n");
}
