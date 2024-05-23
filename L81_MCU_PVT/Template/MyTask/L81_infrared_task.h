/*!
    \file    L81_cliff_task.h
    \brief   the header for L81_cliff_task.c
*/

/*
    Copyright (c) 2023, wxf
*/

#ifndef L81_INFRARED_TASK_H
#define L81_INFRARED_TASK_H

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "gd32l23x.h"
#include "systick.h"
#include "main.h"
#include "L81_TimTask.h"


void infrared_task(void *param);
void infrared_task_init(uint32_t tofset_cmd);
void infrared_task_stop();


#endif  //