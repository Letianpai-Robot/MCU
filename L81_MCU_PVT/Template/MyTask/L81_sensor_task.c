/*!
    \file    L81_sensor_task.c
    \brief    
*/

/*
    Copyright (c) 2023,  
*/


#include "L81_sensor_task.h"

/************************************motor operation brief*******************************************/
/* 

*/
/****************************************************************************************************/
#define SENSOR_DEBUG_EN            0     //debug EN=1 DEN=0
#if SENSOR_DEBUG_EN
#define SensorLog(...) printf(__VA_ARGS__)
#else
#define SensorLog(...) 
#endif

#define PERSON_SENSOR_EN  0


void sensor_light_task(void *param)
{
  STK_light_read();//read light
  
  if(t_stk3311_litht.check_flag)
  {
    t_stk3311_litht.check_flag = 0;
    printf("AT+INT,light,%d\r\n",t_stk3311_litht.light_value/10);  //lux=value/10
  }
}

#if PERSON_SENSOR_EN
void sensor_person_task(void *param)
{
  SHHS34pf80_person_flag_read();//read person 
  
  if(t_sths34pf80_flag.check_flag)
  {
    t_sths34pf80_flag.check_flag = 0;
    printf("AT+INT,person,%d\r\n",t_sths34pf80_flag.pres_flag);
  }
}
#endif

        
        

void sensor_task_init(void)
{
  //sensor init
  STK_init();         //light init
  #if PERSON_SENSOR_EN
  SHHS34pf80_init();  //person
  #endif
  
  //task init
  l81_tim_task_creat(ID_SENSOR_LIGHT, TIM_TASK_CYCLE_ALL, SENSOR_CYCLE_TIM_LIGHT, NULL, sensor_light_task);
  //start task
  l81_tim_task_en(ID_SENSOR_LIGHT);
  
  #if PERSON_SENSOR_EN
  l81_tim_task_creat(ID_SENSOR_PERSON, TIM_TASK_CYCLE_ALL, SENSOR_CYCLE_TIM_PERSON, NULL, sensor_person_task);
  l81_tim_task_en(ID_SENSOR_PERSON);
  #endif
}





