/*!
    \file    L81_TimTask.h
    \brief   the header for L81_TimTask.c
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#ifndef L81_TIM_TASK_H
#define L81_TIM_TASK_H

#include "gd32l23x.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"


#define TIM_TASK_USE      1U
#define TIM_TASK_UNUSE    0U

#define TIM_TASK_ERR      1U
#define TIM_TASK_OK       0U


#define TIM_TASK_CYCLE_ONCE 1U
#define TIM_TASK_CYCLE_ALL  2U

#define TIM_TASK_PERIOD   10U  //10ms

#define TIM_TASK_BUF_LEN  18    //  0713  rjq 15->18  



//
typedef void(*funTimTask)(void *param);

typedef struct
{
  uint8_t  tim_use;
  uint8_t  tim_type;
  uint32_t tim_delay;
  uint32_t tim_count;
  void     *param;
  funTimTask FunTimTask; 
}T_TIM_TASK_TYPDEF;

extern uint8_t tim_task_flag;
extern T_TIM_TASK_TYPDEF tTimTask[TIM_TASK_BUF_LEN];

//LED
#define ID_RUN_LED          0
#define ID_STATUS_LED       1 

//MOTOR
#define ID_EAR              2
#define ID_MOVE             3
#define ID_CALIBRATION      4
#define ID_MOVE_SAMPE       5


//sensor
#define ID_SENSOR_LIGHT     6
#define ID_SENSOR_PERSON    7

//gyro
#define ID_GYRO_DEAL        8
#define ID_GYRO_AVG         9
#define ID_SUSPEND          10
#define ID_WOBBLE           11

//cliff
#define ID_CLIFF            12

//humiture
#define ID_HUMITURE            13
//infrared
#define ID_INFRARED            14  //ID buff = 15
//aw
#define ID_AWset            15  //ID buff = 15
#define ID_AW               16  //ID buff = 18


void tim_call(void); //for tim call
//
void l81_tim_task_init();
uint8_t l81_tim_task_creat(uint8_t tim_id, uint8_t tim_type, uint32_t tim_delay, void *param, funTimTask fun);
uint8_t  l81_tim_task_en(uint8_t tim_id);
uint8_t  l81_tim_task_den(uint8_t tim_id);
uint8_t  l81_tim_task_reset_param(uint8_t tim_id, void *param);
uint8_t  l81_tim_task_reset_delay(uint8_t tim_id, uint32_t tim_delay);
void l81_tim_task_serve(void);

#endif  //L81_MOTORS_H