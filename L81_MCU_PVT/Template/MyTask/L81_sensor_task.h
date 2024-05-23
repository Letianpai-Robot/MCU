/*!
    \file    L81_sensor_task.h
    \brief   the header for L81_sensor_task.c
*/

/*
    Copyright (c) 2023, 
*/

#ifndef L81_SENSOR_TASK_H
#define L81_SENSOR_TASK_H

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "gd32l23x.h"
#include "systick.h"
#include "main.h"
#include "L81_TimTask.h"
#include "stk3311iic.h"
#include "sths34pf80_iic.h"


#define SENSOR_CYCLE_TIM_LIGHT        2000 //cycle time ms
#define SENSOR_CYCLE_TIM_PERSON       2000 //cycle time ms






void sensor_task_init(void);


#endif  //