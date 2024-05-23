/*!
    \file    L81_humiture_task.h
    \brief   the header for L81_humiture_task.c
*/

/*
    Copyright (c) 2023, 
*/

#ifndef L81_HUMITURE_TASK_H
#define L81_HUMITURE_TASK_H

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "gd32l23x.h"
#include "systick.h"
#include "main.h"
#include "L81_TimTask.h"


#define HUMITURE_CYCLE_TIM_CHECK        1000 //cycle time ms

typedef struct
{
  uint8_t temperature_check_en; //1 start,0 stop,
  uint8_t humidity_check_en; //1 start,0 stop,
  uint8_t temperature_new; //1 start,0 stop,
  uint8_t humidity_new; //1 start,0 stop,
  
  float temperature_last;
  float temperature_cur;
  float humidity_last;
  float humidity_cur;
  
  
}T_HUMITURE_PARAM_TYPDEF;


uint16_t read_tem();
uint16_t read_hum();
float get_tem();
float get_hum();
T_HUMITURE_PARAM_TYPDEF get_humiture();

void humiture_task_task();
void humiture_task_init();


#endif  //