/*!
    \file    L81_led_task.c
    \brief    
*/

/*
    Copyright (c) 2023,  
*/


#include "L81_led_task.h"

/************************************motor operation brief*******************************************/
/* 

*/
/****************************************************************************************************/
#define LED_DEBUG_EN            0     //debug EN=1 DEN=0
#if LED_DEBUG_EN
#define LedLog(...) printf(__VA_ARGS__)
#else
#define LedLog(...) 
#endif




void run_led_task(void *param)
{
  gpio_bit_toggle(GPIOB, GPIO_PIN_6);
}

void status_led_task(void *param)
{
  gpio_bit_toggle(GPIOD, GPIO_PIN_4);
}




void led_task_init(void)
{
  l81_tim_task_creat(ID_RUN_LED, TIM_TASK_CYCLE_ALL, 500, NULL, run_led_task);
  l81_tim_task_creat(ID_STATUS_LED, TIM_TASK_CYCLE_ALL, 500, NULL, status_led_task);

  l81_tim_task_en(ID_RUN_LED);
  l81_tim_task_en(ID_STATUS_LED);
}





