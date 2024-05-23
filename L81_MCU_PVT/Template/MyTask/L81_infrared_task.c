/*!
    \file    L81_humiture_task.c
    \brief    
*/

/*
    Copyright (c) 2023, wxf 
*/


#include <stdio.h>
#include "L81_TimTask.h"
#include "L81_infrared_task.h"
//#include "L81_inf.h"
#include "VI530x_User_Handle.h"

#include "gxhtc3iic.h"


/************************************ brief*******************************************/
/* 

*/
/****************************************************************************************************/
#define INFRARED_DEBUG_EN            0   //debug EN=1 DEN=0
#if INFRARED_DEBUG_EN
#define INFRAREDLog(...) printf(__VA_ARGS__)
#else
#define INFRAREDLog(...) 
#endif

void infrared_task(void *param)
{
	
	int16_t correction_tof = distance_measurement_result();   //0620-rjq  void -> int16_t
	if(correction_tof != -1000)
		printf("AT+INT,tof,%d\r\n",correction_tof);
}

void infrared_task_init(uint32_t tofset_cmd)
{
  //task init
  l81_tim_task_creat(ID_INFRARED, TIM_TASK_CYCLE_ALL, tofset_cmd, NULL, infrared_task);

  //start task
  l81_tim_task_en(ID_INFRARED);

  INFRAREDLog("infrared tast init\r\n");
}
void infrared_task_stop()
{
  //stop task
  l81_tim_task_den(ID_INFRARED);

  INFRAREDLog("infrared tast stop\r\n");
}
