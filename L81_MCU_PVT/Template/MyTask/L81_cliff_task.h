/*!
    \file    L81_cliff_task.h
    \brief   the header for L81_cliff_task.c
*/

/*
    Copyright (c) 2023, wxf
*/

#ifndef L81_CLIFF_TASK_H
#define L81_CLIFF_TASK_H

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "gd32l23x.h"
#include "systick.h"
#include "main.h"
#include "L81_TimTask.h"



#define CLIFF_CYCLE_TIM_CHECK				50 //cycle time ms
#define AW_CYCLE_TIM_CHECK					50 //cycle time ms
//#define CLIFF_DENGER_NUM					50
//volatile uint32_t CLIFF_DENGER_NUM;

#define FLAG_ERR    1
#define FLAG_OK     0

#define FLAG_ON     1
#define FLAG_OFF    0

typedef struct
{
  uint8_t cliff_check_en; //1 start,0 stop,
  
  uint8_t cliff_last_flag;
  uint8_t cliff_cur_flag;
  
  uint8_t suspend_last_flag;
  uint8_t suspend_cur_flag;
  
  uint8_t last_move_flag;
  
}T_CLIFF_PARAM_TYPDEF;


void cliff_param_init(void);
void cliff_check_en(void);
void cliff_check_den(void);

void cliff_task_init(void);
uint8_t l81_AT_CfgR_func_CLIFF_DENGER_NUM(void);
uint8_t l81_AT_CfgW_func_CLIFF_DENGER_NUM(uint32_t danger_num);


#endif  //