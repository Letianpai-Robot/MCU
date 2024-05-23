/*!
    \file    L81_main_task.c
    \brief    
*/

/*
    Copyright (c) 2023, wxf 
*/


#include "L81_main_task.h"
#include "L81_sensor_task.h" //for sensor control

//#include "L81_inf.h" //for read adc
//#include "L81_TimTask.h"
#include "L81_AT.h"

#include "L81_inf.h" //for led config
#include "L81_Motors.h" //for motor pwr config
#include "L81_cliff.h"  //for cliff led close 
#include "algorithm.h" //for gyro alg

/************************************motor operation brief*******************************************/
/* 

*/
/****************************************************************************************************/
#define MAIN_DEBUG_EN            0     //debug EN=1 DEN=0
#if MAIN_DEBUG_EN
#define MainLog(...) printf(__VA_ARGS__)
#else
#define MainLog(...) 
#endif

#define STATUS_ON   1U
#define STATUS_OFF  0U


typedef enum
{
  FUN_NONE = 0,
  FUN_SENSOR_LIGHT = 1,
  FUN_SENSOR_TOUCH = 2,
  FUN_MOTOR_PWR = 3,
  FUN_LED = 4,
  FUN_CLIFF_SUSPEND = 5,
  FUN_GYRO_ALG_TASK = 6,  //
  FUN_MOTOR_EARL_PWR = 7,
  FUN_MOTOR_EARR_PWR = 8,
  
  
  FUN_END,
}E_FUN_TYPDEF;

uint8_t fun_contrl(uint8_t cmd, uint8_t status)
{
  E_FUN_TYPDEF eFun = (E_FUN_TYPDEF)cmd;
  uint8_t err = 0;
  switch(eFun)
  {
    case FUN_SENSOR_LIGHT:
    {
      if(status == STATUS_ON)
      {
        l81_tim_task_en(ID_SENSOR_LIGHT);
      }
      else
      {
        l81_tim_task_den(ID_SENSOR_LIGHT);
      }
      err = STATUS_OFF;
    }
    break;
    case FUN_SENSOR_TOUCH:
    {
      //
    }
    break;
    case FUN_MOTOR_PWR:
    {
      if(status == STATUS_ON)
      {
        l81_motor_poweron();
      }
      else
      {
        l81_motor_poweroff();
      }
      err = STATUS_OFF;
    }
    break;
    case FUN_LED:
    {
      if(status == STATUS_ON)
      {
        
      }
      else
      {
        LED_On_set(LEDS_LEFT, LED_GRB_BLACK);
			  LED_On_set(LEDS_RIGHT, LED_GRB_BLACK);
      }
      err = STATUS_OFF;
    }
    break;
    case FUN_CLIFF_SUSPEND:
    {
    
      if(status == STATUS_ON)
      {
        timer_enable(TIMER1);
      }
      else
      {
        timer_disable(TIMER1);
        gpio_bit_write(GPIOB, GPIO_PIN_1 | GPIO_PIN_2, (uint8_t)cliff_Off);
      }
      
      err = STATUS_OFF;
			
			is_printf_cliff = 0;  //1124 rjq++
    }
    break;
    
    case FUN_GYRO_ALG_TASK:
    {
    
      if(status == STATUS_ON)
      {
        algorithm_task_start();
      }
      else
      {
        algorithm_task_stop();
      }
      
      err = STATUS_OFF;
    }
    break;
    
    case FUN_MOTOR_EARL_PWR:
    {
    
      if(status == STATUS_ON)
      {
        CTR_EARL_PWR_ON();
      }
      else
      {
        CTR_EARL_PWR_OFF();
      }
      
      err = STATUS_OFF;
    }
    break;
    
    case FUN_MOTOR_EARR_PWR:
    {
    
      if(status == STATUS_ON)
      {
        CTR_EARR_PWR_ON();
      }
      else
      {
        CTR_EARR_PWR_OFF();
      }
      
      err = STATUS_OFF;
    }
    break;

    
    default :break;
  }

  return err;
}


void main_task(void *param)
{
  
}




void main_task_init(void)
{
//  l81_tim_task_creat(ID_RUN_LED, TIM_TASK_CYCLE_ALL, 500, NULL, main_task);
//  l81_tim_task_en(ID_RUN_LED);

}



uint8_t l81_AT_contrl_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 2U){  //motor must has three paramters, (motor number, value type(angle or pulse), value)
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	uint32_t cmd = String2Int(param[0]);
	uint32_t status = String2Int(param[1]);

		
	if (cmd < 1U || cmd >= (uint8_t)FUN_END) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong fun comd %d, move cmd is[1,4]\r\n", cmd);
		printf("AT+RES,end\r\n");
		return 0U;
	}

	if (status < 0 ||  status > 1) 
  {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong value %d, status is[0,1]\r\n", status);
		printf("AT+RES,end\r\n");
		return 0U;
	}	
	
  fun_contrl(cmd, status);
  
  printf("AT+RES,ACK\r\n");
  printf("AT+RES,FunCtr,%d,%d\r\n",cmd, status);
  printf("AT+RES,end\r\n");
  

  return 1U;
}



