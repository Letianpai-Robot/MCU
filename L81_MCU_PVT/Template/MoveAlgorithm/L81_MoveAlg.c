/*!
    \file    L81_MoveAlg.c
    \brief    
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

#include "gd32l23x.h"
#include "systick.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "L81_Motors.h"
#include "L81_MoveAlg.h"

#include "L81_inf.h" //for read adc
#include "L81_TimTask.h"
#include "L81_AT.h"
#include "gyro_oula.h"             // Component selection
#include "algorithm.h"
/************************************motor operation brief*******************************************/
/* 

*/
/****************************************************************************************************/
#define MOVE_DEBUG_EN            0     //debug EN=1 DEN=0
#if MOVE_DEBUG_EN
#define MoveLog(...) printf(__VA_ARGS__)
#else
#define MoveLog(...) 
#endif



#define READ_FOOT_L()     l81_adc_channel_get_sample(ADC_CHANNEL_0)
#define READ_FOOT_R()     l81_adc_channel_get_sample(ADC_CHANNEL_1)
#define READ_LEG_L()      l81_adc_channel_get_sample(ADC_CHANNEL_3)
#define READ_LEG_R()      l81_adc_channel_get_sample(ADC_CHANNEL_2)
#define READ_EAR_L()      l81_adc_channel_get_sample(ADC_CHANNEL_5)
#define READ_EAR_R()      l81_adc_channel_get_sample(ADC_CHANNEL_4)

#define MOVE_ONE_PERIOD_TIM_US   20000U  //20ms periodic time us
#define MOVE_ONE_PERIOD_TIM_MS   30U      //20ms periodic time ms

#define MOVE_UNLOCK_VALUE     2750u   //pwm value
#define MOVE_UNLOCK_PERIOD    1u      //pwm num

#define MOVE_STOR_VALUE       3400u   //pwm value
#define MOVE_STOR_PERIOD      4u      //pwm num



T_MOVE_TYPDEF t_move = {0};
T_MOVE_STEP_TYPDEF t_move_step = {0};
T_MOVE_EAR_TYPDEF  t_move_ear_step = {0};

void move_sampe_task(void *param);
static void move_body_init(void);
static void move_ear_init(void);
float yaw_start;
float yaw_d;
float yaw_now;
float yaw_last;
int last_move = 0;
int cur_move = 0;

int is_in_turn = 0;

float Kleg = 3;  

void l81_move_init()
{
  #if 1
  t_move.tLegL.zero_value = 1500;
  t_move.tFootL.zero_value = 1500;
  
  t_move.tLegR.zero_value = 1500;
  t_move.tFootR.zero_value = 1500;
  
  t_move.tEarL.zero_value = 1500;
  t_move.tEarR.zero_value = 1500;
  #else
  t_move.tLegL.zero_value = 1100;
  t_move.tFootL.zero_value = 1500;
  
  t_move.tLegR.zero_value = 1800;
  t_move.tFootR.zero_value = 1500;
  
  t_move.tEarL.zero_value = 1500;
  t_move.tEarR.zero_value = 1500;
  #endif
//  //motor tim task init
  
//  l81_tim_task_creat(ID_FOOT_L, TIM_TASK_CYCLE_ONCE, 20, (void *)&t_ctrl, l81_move_step);
//  l81_tim_task_creat(ID_FOOT_R, TIM_TASK_CYCLE_ONCE, 20, (void *)&t_ctrl, l81_move_step);
//  l81_tim_task_creat(ID_LEG_L, TIM_TASK_CYCLE_ONCE, 20, (void *)&t_ctrl, l81_move_step);
//  l81_tim_task_creat(ID_LEG_R, TIM_TASK_CYCLE_ONCE, 20, (void *)&t_ctrl, l81_move_step);
////  l81_tim_task_creat(ID_FOOT_L, TIM_TASK_CYCLE_ONCE, 20, (void *)&t_ctrl_foot_l, l81_move_step);
//  
//  l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, l81_move_walk_step);
////  
////  l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, l81_move_left_step);


//  t_ctrl_foot_l.eBody = MOVE_FOOT_L;
//  t_ctrl_foot_l.eDir  = MOVE_R;
//  t_ctrl_foot_l.angel = 0;
//  
//  t_ctrl_foot_r.eBody = MOVE_FOOT_R;
//  t_ctrl_foot_r.eDir  = MOVE_R;
//  t_ctrl_foot_r.angel = 0;
//  
//  t_ctrl_leg_l.eBody = MOVE_LEG_L;
//  t_ctrl_leg_l.eDir  = MOVE_R;
//  t_ctrl_leg_l.angel = 0;
//  
//  t_ctrl_leg_r.eBody = MOVE_LEG_R;
//  t_ctrl_leg_r.eDir  = MOVE_R;
//  t_ctrl_leg_r.angel = 0;
//  
//  t_ctrl_ear_l.eBody = MOVE_EAR_L;
//  t_ctrl_ear_l.eDir  = MOVE_R;
//  t_ctrl_ear_l.angel = 0;
//  
//  t_ctrl_ear_r.eBody = MOVE_EAR_R;
//  t_ctrl_ear_r.eDir  = MOVE_R;
//  t_ctrl_ear_r.angel = 0;

  motor_set_pulse(1, 1500);
  motor_set_pulse(2, 1500);
  motor_set_pulse(3, 1500);
  motor_set_pulse(4, 1500);
  
  motor_set_pulse(5, 1500);
  motor_set_pulse(6, 1500);
  
  move_body_init();
  
  move_ear_init();
  
  
//  int32_t set_step[4]={step,step,step,step};
//  int32_t set_angle[4]={20,20,20,20};
//  int32_t set_delay[4]={delay,delay,delay,delay};
  
//  move_stop_en();
//  move_body_init();
//  move_body_step(set_step);
//  move_body_angle(set_angle);
//  move_body_delay(set_delay);
//  move_body_cycl_num();
  l81_tim_task_creat(ID_MOVE_SAMPE, TIM_TASK_CYCLE_ALL, MOVE_TIME_SAMPE, NULL, move_sampe_task);
  l81_tim_task_en(ID_MOVE_SAMPE);
}



static void move_ear_init(void)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_l   = &t_move.tEarL;
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_r   = &t_move.tEarR;

  
  
   pt_ear_l->stop = MOVE_ON;
   pt_ear_l->e_body = MOVE_EAR_L;
   pt_ear_l->e_shock = MOVE_SHOCK_1; 
   pt_ear_l->zero_value = 1500;   ///??
   pt_ear_l->delay_value = 1000;
   pt_ear_l->tim_count = 0;
   pt_ear_l->cycle_num = 0;  // delay_value/MOVE_TIME_SAMPE
   pt_ear_l->cycle_count = 0;
   pt_ear_l->step_num = 1;
   pt_ear_l->step_count = 0;
   pt_ear_l->last_set_angle = 0;
   pt_ear_l->last_set_value = 0;
   pt_ear_l->last_read_value = 0;
   pt_ear_l->cur_set_angle = 0;
   pt_ear_l->cur_set_value = 0;
   pt_ear_l->cur_read_value = 0;
   pt_ear_l->target_angle = 0;    //
  
   pt_ear_r->stop = MOVE_ON;
   pt_ear_r->e_body = MOVE_EAR_R;
   pt_ear_r->e_shock = MOVE_SHOCK_1; 
   pt_ear_r->zero_value = 1500;   ///??
   pt_ear_r->delay_value = 1000;
   pt_ear_r->tim_count = 0;
   pt_ear_r->cycle_num = 0;  // delay_value/MOVE_TIME_SAMPE
   pt_ear_r->cycle_count = 0;
   pt_ear_r->step_num = 1;
   pt_ear_r->step_count = 0;
   pt_ear_r->last_set_angle = 0;
   pt_ear_r->last_set_value = 0;
   pt_ear_r->last_read_value = 0;
   pt_ear_r->cur_set_angle = 0;
   pt_ear_r->cur_set_value = 0;
   pt_ear_r->cur_read_value = 0;
   pt_ear_r->target_angle = 0;    //

}

static void move_refesh_ear_en(void)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_l   = &t_move.tEarL;
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_r   = &t_move.tEarR;
  
  pt_ear_l->stop  = MOVE_OFF;
  pt_ear_r->stop  = MOVE_OFF;

}

static void move_refesh_ear_den(void)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_l   = &t_move.tEarL;
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_r   = &t_move.tEarR;
  
  pt_ear_l->stop  = MOVE_ON;
  pt_ear_r->stop  = MOVE_ON;
  
}








static void move_body_init(void)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  
   pt_leg_l->stop = MOVE_ON;
   pt_leg_l->e_body = MOVE_LEG_L;
   pt_leg_l->e_shock = MOVE_SHOCK_1; 
   pt_leg_l->zero_value = 1500;   ///??
   pt_leg_l->delay_value = 1000;
   pt_leg_l->tim_count = 0;
   pt_leg_l->cycle_num = 0;  // delay_value/MOVE_TIME_SAMPE
   pt_leg_l->cycle_count = 0;
   pt_leg_l->step_num = 1;
   pt_leg_l->step_count = 0;
   pt_leg_l->last_set_angle = 0;
   pt_leg_l->last_set_value = 0;
   pt_leg_l->last_read_value = 0;
   pt_leg_l->cur_set_angle = 0;
   pt_leg_l->cur_set_value = 0;
   pt_leg_l->cur_read_value = 0;
   pt_leg_l->target_angle = 0;    //
  
   pt_leg_r->stop = MOVE_ON;
   pt_leg_r->e_body = MOVE_LEG_R;
   pt_leg_r->e_shock = MOVE_SHOCK_1; 
   pt_leg_r->zero_value = 1500;   ///??
   pt_leg_r->delay_value = 1000;
   pt_leg_r->tim_count = 0;
   pt_leg_r->cycle_num = 0;  // delay_value/MOVE_TIME_SAMPE
   pt_leg_r->cycle_count = 0;
   pt_leg_r->step_num = 1;
   pt_leg_r->step_count = 0;
   pt_leg_r->last_set_angle = 0;
   pt_leg_r->last_set_value = 0;
   pt_leg_r->last_read_value = 0;
   pt_leg_r->cur_set_angle = 0;
   pt_leg_r->cur_set_value = 0;
   pt_leg_r->cur_read_value = 0;
   pt_leg_r->target_angle = 0;    //
   
   pt_foot_l->stop = MOVE_ON;
   pt_foot_l->e_body = MOVE_FOOT_L;
   pt_foot_l->e_shock = MOVE_SHOCK_1; 
   pt_foot_l->zero_value = 1500;   ///??
   pt_foot_l->delay_value = 1000;
   pt_foot_l->tim_count = 0;
   pt_foot_l->cycle_num = 0;  // delay_value/MOVE_TIME_SAMPE
   pt_foot_l->cycle_count = 0;
   pt_foot_l->step_num = 1;
   pt_foot_l->step_count = 0;
   pt_foot_l->last_set_angle = 0;
   pt_foot_l->last_set_value = 0;
   pt_foot_l->last_read_value = 0;
   pt_foot_l->cur_set_angle = 0;
   pt_foot_l->cur_set_value = 0;
   pt_foot_l->cur_read_value = 0;
   pt_foot_l->target_angle = 0;    //
   
   pt_foot_r->stop = MOVE_ON;
   pt_foot_r->e_body = MOVE_FOOT_R;
   pt_foot_r->e_shock = MOVE_SHOCK_1; 
   pt_foot_r->zero_value = 1500;   ///??
   pt_foot_r->delay_value = 1000;
   pt_foot_r->tim_count = 0;
   pt_foot_r->cycle_num = 0;  // delay_value/MOVE_TIME_SAMPE
   pt_foot_r->cycle_count = 0;
   pt_foot_r->step_num = 1;
   pt_foot_r->step_count = 0;
   pt_foot_r->last_set_angle = 0;
   pt_foot_r->last_set_value = 0;
   pt_foot_r->last_read_value = 0;
   pt_foot_r->cur_set_angle = 0;
   pt_foot_r->cur_set_value = 0;
   pt_foot_r->cur_read_value = 0;
   pt_foot_r->target_angle = 0;    //
  
  
}

static void move_stop_en(void)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  pt_leg_l->stop  = MOVE_ON;
  pt_leg_r->stop  = MOVE_ON;
  pt_foot_l->stop = MOVE_ON;
  pt_foot_r->stop = MOVE_ON;
  
}

static void move_stop_den(void)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  pt_leg_l->stop  = MOVE_OFF;
  pt_leg_r->stop  = MOVE_OFF;
  pt_foot_l->stop = MOVE_OFF;
  pt_foot_r->stop = MOVE_OFF;
  
}

static void move_body_angle(int32_t *p_angle)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  pt_leg_l->target_angle  = p_angle[0];
  pt_leg_r->target_angle  = p_angle[1];
  pt_foot_l->target_angle = p_angle[2];
  pt_foot_r->target_angle = p_angle[3];
  
}

static void move_body_delay(int32_t *p_delay)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  pt_leg_l->delay_value  = p_delay[0];
  pt_leg_r->delay_value  = p_delay[1];
  pt_foot_l->delay_value = p_delay[2];
  pt_foot_r->delay_value = p_delay[3];
  
}

static void move_body_step(int32_t *p_step)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  pt_leg_l->step_num  = p_step[0];
  pt_leg_r->step_num  = p_step[1];
  pt_foot_l->step_num = p_step[2];
  pt_foot_r->step_num = p_step[3];
  
}

static void move_body_cycl_num(void)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  pt_leg_l->cycle_num  = pt_leg_l->delay_value/MOVE_TIME_SAMPE;
  pt_leg_r->cycle_num  = pt_leg_r->delay_value/MOVE_TIME_SAMPE;
  pt_foot_l->cycle_num = pt_foot_l->delay_value/MOVE_TIME_SAMPE;
  pt_foot_r->cycle_num = pt_foot_r->delay_value/MOVE_TIME_SAMPE;
  
}
/**
    \brief      angle turn pwm value
    \param[in]  e_body
    \param[in]  e_dir
    \param[in]  angel
    \retval     pwm value
  */
int32_t l81_move_angel_to_value(E_MOVE_BODY_TYPDEF e_body, E_MOVE_DIRECTION_TYPDEF e_dir, uint32_t angel)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_move_dat = NULL;
  int32_t pwm_value = (uint32_t)((MOVE_MAX_VALUE-MOVE_MIN_VALUE)/(MOVE_MAX_ANGLE - MOVE_MIN_ANGLE)) * angel;
  
  switch(e_body)
  {
    case MOVE_FOOT_L:
    {
      pt_move_dat = &t_move.tFootL;
      if(MOVE_L == e_dir)
      {
        pwm_value = 0 - pwm_value;
      }
    }
    break;
    
    case MOVE_FOOT_R:
    {
      pt_move_dat = &t_move.tFootR;
      if(MOVE_L == e_dir)
      {
        pwm_value = 0 - pwm_value;
      }
    }
    break;
    case MOVE_LEG_L:
    {
      pt_move_dat = &t_move.tLegL;
      if(MOVE_R == e_dir)
      {
        pwm_value = 0 - pwm_value;
      }
    }
    break;
    case MOVE_LEG_R:
    {
      pt_move_dat = &t_move.tLegR;
      if(MOVE_R == e_dir)
      {
        pwm_value = 0 - pwm_value;
      }
    }
    break;
    
    //left view
    case MOVE_EAR_L:
    {
      pt_move_dat = &t_move.tEarL;
      if(MOVE_R == e_dir)
      {
        pwm_value = 0 - pwm_value;
      }
    }
    break;
    
    //left view
    case MOVE_EAR_R:
    {
      pt_move_dat = &t_move.tEarR;
      if(MOVE_L == e_dir)
      {
        pwm_value = 0 - pwm_value;
      }
    }
    break;
    default :break;
  }
    
    return pwm_value;
    
}


/**
    \brief      set body turn angle
    \param[in]  none
    \param[out] none
    \retval     none
  */
uint8_t l81_move_body_write(E_MOVE_BODY_TYPDEF e_body, E_MOVE_DIRECTION_TYPDEF e_dir, uint32_t angle)
{
  uint8_t err = MOVE_ERR;
  uint8_t body = e_body;
  T_MOVE_BODY_DATA_TYPDEF *pt_move_dat = NULL;
  
  int32_t pwm_angle = 0;
  int32_t pwm_set = 0;
  
  
  MOTOR_NUM_TYPE num;
  switch(e_body)
  {
    case MOVE_FOOT_L:
    {
      pt_move_dat = &t_move.tFootL;
      num = MOTOR_NUM1;
    }
    break;
    
    case MOVE_FOOT_R:
    {
      pt_move_dat = &t_move.tFootR;
      num = MOTOR_NUM2;
    }
    break;
    case MOVE_LEG_L:
    {
      pt_move_dat = &t_move.tLegL;
      num = MOTOR_NUM3;
    }
    break;
    case MOVE_LEG_R:
    {
      pt_move_dat = &t_move.tLegR;
      num = MOTOR_NUM4;
    }
    break;
    
    case MOVE_EAR_L:
    {
      pt_move_dat = &t_move.tEarL;
      num = MOTOR_NUM5;
    }
    break;
    
    case MOVE_EAR_R:
    {
      pt_move_dat = &t_move.tEarR;
      num = MOTOR_NUM6;
    }
    break;
    default :return MOVE_ERR;
  }
  
  if(pt_move_dat == NULL)
  {
    return err = MOVE_ERR;
  }
  
  //angle chenge pwm value
  pwm_angle = l81_move_angel_to_value(e_body, e_dir, angle);
  
  
  pwm_set = pt_move_dat->zero_value;
  
  pwm_set = pwm_set + pwm_angle;
  
  if((pwm_set < MOVE_MIN_VALUE) || (pwm_set > MOVE_MAX_VALUE))
  {
    MoveLog("err pwm_value=%d\r\n",pwm_set);
    return err = MOVE_ERR;
  }
  
  pt_move_dat->cur_set_value = (uint32_t)pwm_set;
//  printf("num=%d value=%d\r\n",num,pt_move_dat->cur_set_value);
  //
	motor_set_pulse(num, pt_move_dat->cur_set_value);
 
//  MoveLog("num=%d value=%d\r\n",num,pt_move_dat->cur_set_value);


  return err;
}



void refresh(void *p)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_body = (T_MOVE_BODY_DATA_TYPDEF *)p;
  E_MOVE_BODY_TYPDEF e_body = pt_body->e_body;
  E_MOVE_DIRECTION_TYPDEF e_dir;
  uint32_t angle = 0;
  uint8_t err = MOVE_ERR;
  
  if(pt_body->stop)
  {
    return;
  }
  
  pt_body->cur_set_angle = (MOVE_TIME_SAMPE * pt_body->target_angle * pt_body->cycle_count)/pt_body->delay_value;
  
  switch(pt_body->e_shock)
  {
    case MOVE_SHOCK_1:
      e_dir = MOVE_L;
    
      angle = pt_body->cur_set_angle;
      if(pt_body->cycle_count == 0)
      {
        angle = 0;
      }
      else if(pt_body->cycle_count >= pt_body->cycle_num)
      {
//        pt_body->cycle_count = 0;
        pt_body->cur_set_angle = pt_body->target_angle;
        angle = pt_body->target_angle;
        
        pt_body->run_flag = MOVE_OFF;
      }
    

      break;
    case MOVE_SHOCK_2:
      e_dir = MOVE_L;
      angle = pt_body->target_angle - pt_body->cur_set_angle;
      if(pt_body->cycle_count == 0)
      {
        angle = pt_body->target_angle;
      }
      else if(pt_body->cycle_count >= pt_body->cycle_num)
      {
//        pt_body->cycle_count = 0;
        pt_body->cur_set_angle = pt_body->target_angle;
        angle = 0;
        
        pt_body->run_flag = MOVE_OFF;
      }
      

      break;
    case MOVE_SHOCK_3:
      e_dir = MOVE_R;
      angle = pt_body->cur_set_angle;
      if(pt_body->cycle_count == 0)
      {
        angle = 0;
      }
      else if(pt_body->cycle_count >= pt_body->cycle_num)
      {
//        pt_body->cycle_count = 0;
        pt_body->cur_set_angle = pt_body->target_angle;
        angle = pt_body->target_angle;
        
        pt_body->run_flag = MOVE_OFF;
      }
      

      break;
    case MOVE_SHOCK_4:
      e_dir = MOVE_R;
      angle = pt_body->target_angle - pt_body->cur_set_angle;
      if(pt_body->cycle_count == 0)
      {
        angle = pt_body->target_angle;
      }
      else if(pt_body->cycle_count >= pt_body->cycle_num)
      {
//        pt_body->cycle_count = 0;
        pt_body->cur_set_angle = pt_body->target_angle;
        angle = 0;
        
        pt_body->run_flag = MOVE_OFF;
      }

      break;
    
    default :
      e_dir = MOVE_L;
    break;
  }
  
  
  e_body = pt_body->e_body;
  
  
  err = l81_move_body_write(e_body, e_dir, angle);
  
  pt_body->cycle_count ++;
  if(pt_body->cycle_count > pt_body->cycle_num)
  {
    pt_body->stop = MOVE_ON;
    pt_body->run_flag = MOVE_ON;
  }
  
//  pt_body->tim_count += MOVE_TIME_SAMPE;
//  if(pt_body->tim_count > pt_body->delay_value)
//  {
//    pt_body->tim_count = 0;
//    pt_body->cycle_count = 0;
//    pt_body->step_count ++;
////    pt_body->e_shock ++;
//    if(pt_body->e_shock > MOVE_SHOCK_4)
//    {
//      pt_body->e_shock = MOVE_SHOCK_1;
////      pt_body->cur_set_angle= 0;
//    }
//  }
  
  if(err == MOVE_OK)
  {
    pt_body->last_set_angle = pt_body->cur_set_angle;
  }
  
  if(pt_body->step_count/4 > pt_body->step_num)
  {
    pt_body->stop = MOVE_ON;
  }

}

void move_sampe_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_l  = &t_move.tEarL;
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_r  = &t_move.tEarR;

  refresh(pt_leg_l);
  refresh(pt_leg_r);
  refresh(pt_foot_l);
  refresh(pt_foot_r);
  
  refresh(pt_ear_l);
  refresh(pt_ear_r);
  
}

void move_step_set(E_MOVE_BODY_TYPDEF e_body, E_MOVE_SHOCK_TYPDEF e_shock, uint32_t delay, uint32_t angle)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  switch(e_body)
  {
    case MOVE_FOOT_L  : 
    {
      pt_foot_l->e_shock = e_shock;
      pt_foot_l->delay_value  = delay;
      pt_foot_l->target_angle = angle;
      pt_foot_l->cycle_num  = pt_foot_l->delay_value/MOVE_TIME_SAMPE;
      pt_foot_l->cycle_count = 0;
      pt_foot_l->tim_count  = 0;
      
    }break;
    case MOVE_FOOT_R  : 
    {
      pt_foot_r->e_shock = e_shock;
      pt_foot_r->delay_value  = delay;
      pt_foot_r->target_angle = angle;
      pt_foot_r->cycle_num  = pt_foot_r->delay_value/MOVE_TIME_SAMPE;
      pt_foot_r->cycle_count = 0;
      pt_foot_r->tim_count  = 0;
    }break;
    case MOVE_LEG_L   : 
    {
      pt_leg_l->e_shock = e_shock;
      pt_leg_l->delay_value  = delay;
      pt_leg_l->target_angle = angle;
      pt_leg_l->cycle_num  = pt_leg_l->delay_value/MOVE_TIME_SAMPE;
      pt_leg_l->cycle_count = 0;
      pt_leg_l->tim_count  = 0;
    }break;
    case MOVE_LEG_R   : 
    {
      pt_leg_r->e_shock = e_shock;
      pt_leg_r->delay_value  = delay;
      pt_leg_r->target_angle = angle;
      pt_leg_r->cycle_num  = pt_leg_r->delay_value/MOVE_TIME_SAMPE;
      pt_leg_r->cycle_count = 0;
      pt_leg_r->tim_count  = 0;
    }break;
    default :break;
  }
}

void move_step_en(E_MOVE_BODY_TYPDEF e_body)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  switch(e_body)
  {
    case MOVE_FOOT_L  : 
    {
      pt_foot_l->stop = MOVE_OFF;
      pt_foot_l->run_flag = MOVE_OFF;
      
    }break;
    case MOVE_FOOT_R  : 
    {
      pt_foot_r->stop = MOVE_OFF;
      pt_foot_r->run_flag = MOVE_OFF;
    }break;
    case MOVE_LEG_L   : 
    {
      pt_leg_l->stop = MOVE_OFF;
      pt_leg_l->run_flag = MOVE_OFF;
    }break;
    case MOVE_LEG_R   : 
    {
      pt_leg_r->stop = MOVE_OFF;
      pt_leg_r->run_flag = MOVE_OFF;
    }break;
    default :break;
  }
  
}

void move_body_set(E_MOVE_BODY_TYPDEF e_body, E_MOVE_SHOCK_TYPDEF e_shock, uint32_t delay, uint32_t angle)
{
	move_step_set(e_body,e_shock,delay,angle);
  move_step_en(e_body);
}

uint8_t move_get_use_flag(E_MOVE_BODY_TYPDEF e_body)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  switch(e_body)
  {
    case MOVE_FOOT_L  : 
    {
      return pt_foot_l->run_flag;
    }break;
    case MOVE_FOOT_R  : 
    {
      return pt_foot_r->run_flag;
    }break;
    case MOVE_LEG_L   : 
    {
      return pt_leg_l->run_flag;
    }break;
    case MOVE_LEG_R   : 
    {
      return pt_leg_r->run_flag;
    }break;
    default :break;
  }
  return 0;
}





/*-body start-************/
void l18_move_zero(void)
{
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  l81_move_body_write(MOVE_FOOT_L, MOVE_L, 0);
  l81_move_body_write(MOVE_FOOT_R, MOVE_L, 0);
  l81_move_body_write(MOVE_LEG_L, MOVE_L, 0);
  l81_move_body_write(MOVE_LEG_R, MOVE_L, 0);
  
//  printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
}

void move_work_fleft_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = 18;
  uint32_t foot_l_gangl = 2;
  uint32_t foot_r_gangl = 25;

  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); 
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); 
      move_step_en(MOVE_LEG_R);
      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_l_gangl); 
      move_step_en(MOVE_FOOT_L);
      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_r_gangl); 
      move_step_en(MOVE_FOOT_R);
      
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }
  
  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag)
      {

        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_l_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_r_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag)
      {
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_l_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_r_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag)
      {  
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_l_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_r_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_l_gangl); //ritht -
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_r_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_l_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_r_gangl); //ritht +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        pt_move->step_count++;
      }
    }break;
    default : break; 
  } 

}

void move_work_fright_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = 18;
  uint32_t foot_l_gangl = 30;
  uint32_t foot_r_gangl = 2;

  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); 
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); 
      move_step_en(MOVE_LEG_R);
      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_l_gangl); 
      move_step_en(MOVE_FOOT_L);
      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_r_gangl); 
      move_step_en(MOVE_FOOT_R);
      
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }
  
  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag)
      {

        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_l_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_r_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag)
      {
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_l_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_r_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag)
      {  
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_l_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_r_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_l_gangl); //ritht -
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_r_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_l_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_r_gangl); //ritht +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        pt_move->step_count++;
      }
    }break;
    default : break; 
  } 

}




void move_work_bleft_task(void *param)
{
  
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = 15;
  
  uint32_t foot_gangl = 10;
  uint32_t foot_angle_r = 30;

  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); 
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); 
      move_step_en(MOVE_LEG_R);
      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); 
      move_step_en(MOVE_FOOT_L);
      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_angle_r); 
      move_step_en(MOVE_FOOT_R);
      
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }
  
  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag)
      {

        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_angle_r); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag)
      {
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_angle_r); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag)
      {  
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_angle_r); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_angle_r); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_angle_r); //ritht +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        pt_move->step_count++;
      }
    }break;
    default : break; 
  } 

}


void move_work_bleft2_task(void *param)
{
  
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = 15;
  
  uint32_t foot_gangl = 30;
  uint32_t foot_angle_r = 10;

  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); 
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); 
      move_step_en(MOVE_LEG_R);
      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); 
      move_step_en(MOVE_FOOT_L);
      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_angle_r); 
      move_step_en(MOVE_FOOT_R);
      
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }
  
  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag)
      {

        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_angle_r); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag)
      {
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_angle_r); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag)
      {  
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_angle_r); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_angle_r); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_angle_r); //ritht +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        pt_move->step_count++;   
      }
    }break;
    default : break; 
  } 

}


void move_work_bright_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = 15;
  uint32_t foot_l_gangl = 5;
  uint32_t foot_r_gangl = 25;

  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {

    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); 
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); 
      move_step_en(MOVE_LEG_R);
      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_l_gangl); 
      move_step_en(MOVE_FOOT_L);
      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_r_gangl); 
      move_step_en(MOVE_FOOT_R);
      
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }
  
  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag)
      {

        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_l_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_r_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag)
      {
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_l_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_r_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag)
      {  
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_l_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_r_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_l_gangl); //ritht -
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_r_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_l_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_r_gangl); //ritht +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        pt_move->step_count++;   
      }
    }break;
    default : break; 
  } 
}

void move_work_backward_task(void *param)
{
  
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = 15;
  uint32_t foot_gangl = 30;

  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }
  
  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag)
      {

        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag)
      {
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag)
      {  
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag)
      {
				if(pt_move->step_count == pt_move->step_num -1)
				{
					t_move_step.u_cur_step.e_work_step = WORK_OVER;
					t_move_step.u_last_step.e_work_step = WORK_FOOT5;
					break;
				}
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        pt_move->step_count++;   
      }
      
    }break;
    default : break; 
  } 

}

void move_work_forward_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

//  uint32_t leg_gangl = 15;
//  uint32_t foot_gangl = 30;
	uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
//	uint32_t leg_l_angle = t_move_step.leg_l_angle;
//  uint32_t leg_r_angle = t_move_step.leg_r_angle;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag)
      {

        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag)
      {
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag)
      {  
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_leg_l->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag)
      {
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag)
      {
				if(pt_move->step_count == pt_move->step_num -1)
				{
					t_move_step.u_cur_step.e_work_step = WORK_OVER;
					t_move_step.u_last_step.e_work_step = WORK_FOOT5;
					break;
				}
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}



void move_work_left_task(void *param) 
{
  
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

	uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
//  uint32_t leg_gangl = 15;
//  uint32_t foot_gangl = 10;
  uint32_t foot_angle_r = 30;

  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
//  pt_move->leg_tim = 300;
	
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);

      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }
  
  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_LEG;

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      if(pt_leg_r->run_flag)
      {
      }
    }break;
    
    case WORK_OVER:
    {
      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG;
        pt_move->step_count++;
      }
    }break;
    default : break; 
  } 

}


void move_work_right_task(void *param) 
{
  
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

	uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
//  uint32_t leg_gangl = 15;
//  uint32_t foot_gangl = 10;
  uint32_t foot_angle_r = 30;

  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
//  pt_move->leg_tim = 300;
	
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);

      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }
  
  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_LEG;

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
//        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
//        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
//        move_step_en(MOVE_LEG_R);
      }
      if(pt_leg_r->run_flag)
      {
      }
    }break;
    
    case WORK_OVER:
    {
      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG;
        pt_move->step_count++;
      }
    }break;
    default : break; 
  } 

}


void set_yaw_d()
{
	if(yaw_d >330) yaw_d-=360;
	if(yaw_d <-330) yaw_d+=360;
	if(yaw_d >40 ) yaw_d = 0;
	if(yaw_d < -40 ) yaw_d = 0;
	if(yaw_d >20 ) yaw_d = 20;
	if(yaw_d < -20 ) yaw_d = -20;
}
void move_work_forward_gyro_task(void *param) // 63  gyro algorithm 91
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

//  uint32_t leg_gangl = 15;
  uint32_t leg_gangl = t_move_step.leg_l_angle;
//  uint32_t foot_gangl = 30;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
//  uint32_t foot_gangl = foot_gangl_start;
	float Kp = 0.2;
	float Kp_l = 0.1;
//	float Kp = 0;
//	float Kp_l = 0;
//	float Kleg = 3;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
			yaw_start = 0;
			yaw_now = 0;
			yaw_last = 0;
			yaw_d = 0;
      
      //l18_move_zero();
			if (cur_move == 91)
				return;
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg - yaw_d*Kp_l); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				yaw_start = 0;
				yaw_start = yaw_dat;
				yaw_now = yaw_start;
				yaw_last = yaw_now;
				yaw_d = yaw_start - yaw_now;
				set_yaw_d();
				//printf("AT+WORK_FOOT,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_now,yaw_d);
				
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {

				yaw_d = yaw_start - yaw_last;
				set_yaw_d();
				if (cur_move == 91)
				{
					yaw_d = 0;
				}
				//printf("AT+WORK_LEG2,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg - yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
			if (cur_move == 91)
				{
					yaw_d = 0;
				}
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				//printf("AT+WORK_LEG3,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_LEG3,yaw_start,yaw_now,yaw_last,yaw_d,    %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl + yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg + yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				yaw_d = yaw_start - yaw_last;
				set_yaw_d();
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
				if (cur_move == 91)
				{
					yaw_d = 0;
					t_move_step.u_cur_step.e_work_step = WORK_LEG4;
				}
				//printf("AT+WORK_FOOT2,yaw_start,yaw_now,yaw_d,  %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_FOOT2,yaw_start,yaw_now,yaw_last,yaw_d,   %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
				
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {  
				yaw_last = yaw_now;
				yaw_now = yaw_dat;
				yaw_d = yaw_start - yaw_now;
				set_yaw_d();
				//printf("AT+WORK_FOOT3,yaw_start,yaw_now,yaw_d,  %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_now,yaw_d);
				//printf("AT+WORK_FOOT3,yaw_start,yaw_now,yaw_last,yaw_d,   %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_now,yaw_last,yaw_d);
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				yaw_d = yaw_start - yaw_last;
				set_yaw_d();
				//printf("AT+WORK_LEG4,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_LEG4,yaw_start,yaw_now,yaw_last,yaw_d,    %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
				
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
				if (cur_move == 91)
				{
					yaw_d = 0;
					t_move_step.u_cur_step.e_work_step = WORK_OVER;
				}
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl + yaw_d*Kp_l); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg + yaw_d*Kp_l); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				//printf("AT+WORK_LEG5,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_LEG5,yaw_start,yaw_now,yaw_last,yaw_d,    %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg - yaw_d*Kp_l); //left +
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				//printf("AT+WORK_FOOT4,yaw_start,yaw_now,yaw_d,  %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_FOOT4,yaw_start,yaw_now,yaw_last,yaw_d,   %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      } 
    }break;

    case WORK_FOOT5:
    {
			if(pt_move->step_count == pt_move->step_num -1)
			{
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
				break;
			}
				
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				yaw_last = yaw_now;
				yaw_now = yaw_dat;
				yaw_d = yaw_start - yaw_now;
				set_yaw_d();
				//printf("AT+WORK_FOOT5,yaw_start,yaw_now,yaw_d,  %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_now,yaw_d);
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //left +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_R);
      } 
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      pt_move->step_count ++;
			
			if (cur_move == 91)
				t_move_step.u_cur_step.e_work_step = WORK_LEG;
			
    }break;
    default : break; 
  } 

}
void move_work_backward_gyro_task(void *param) // 64  gyro algorithm 92 
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = 15;
//  uint32_t foot_gangl = 30;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
//  uint32_t foot_gangl = foot_gangl_start;
	float Kp = 0;
	float Kp_l = 0;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
			l81_tim_task_den(ID_MOVE);
			yaw_start = 0;
			yaw_now = 0;
			yaw_last = 0;
			yaw_d = 0;
      
//      l18_move_zero();
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
			if (cur_move == 92)
				return;
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg  - yaw_d*Kp_l); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht +
//      move_step_en(MOVE_FOOT_R);
      
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg ); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {

      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
			{
				yaw_start = 0;
				yaw_start = yaw_dat;
				yaw_now = yaw_start;
				yaw_last = yaw_now;
				yaw_d = yaw_start - yaw_now;
				if(yaw_d >30 || yaw_now > 330) yaw_d = 20;
				if(yaw_d < -30 ) yaw_d = -20;
				//printf("AT+MOVEW,yaw_start,yaw_now,yaw_last,yaw_d  ,%f,%f,%f,%f\r\n",yaw_start,yaw_now,yaw_last,yaw_d);
				
				if (cur_move == 92)
				{
					yaw_d = 0;
				}
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {

				yaw_d = yaw_start - yaw_last;
				if (cur_move == 92)
				{
					yaw_d = 0;
				}
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg  - yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl + yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg  + yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				yaw_d = yaw_start - yaw_last;
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
				if (cur_move == 92)
				{
					yaw_d = 0;
					t_move_step.u_cur_step.e_work_step = WORK_LEG4;
				}
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {  
				yaw_last = yaw_now;
				yaw_now = yaw_dat;
				yaw_d = yaw_start - yaw_now;
			if(yaw_d >20 || yaw_now > 330) yaw_d = 20;
			if(yaw_d < -20 ) yaw_d = -20;
			if(yaw_d >30 || yaw_now > 340) yaw_d = 0;
			if(yaw_d < -30 ) yaw_d = 0;
				//printf("AT+MOVEW,yaw_start,yaw_now,yaw_last,yaw_d  ,%f,%f,%f,%f\r\n",yaw_start,yaw_now,yaw_last,yaw_d);
        
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				yaw_d = yaw_start - yaw_last;
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				if (cur_move == 92)
				{
					yaw_d = 0;
					t_move_step.u_cur_step.e_work_step = WORK_OVER;
				}
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl + yaw_d*Kp_l); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg  + yaw_d*Kp_l); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg  - yaw_d*Kp_l); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;

    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				if(pt_move->step_count == pt_move->step_num -1)
				{
					t_move_step.u_cur_step.e_work_step = WORK_OVER;
					t_move_step.u_last_step.e_work_step = WORK_FOOT5;
					break;
				}
//				yaw_last = yaw_now;
//				yaw_now = yaw_dat;
//				yaw_d = yaw_start - yaw_now;
//			if(yaw_d >20 || yaw_now > 330) yaw_d = 20;
//			if(yaw_d < -20 ) yaw_d = -20;
//			if(yaw_d >30 || yaw_now > 340) yaw_d = 0;
//			if(yaw_d < -30 ) yaw_d = 0;
				//printf("AT+MOVEW,yaw_start,yaw_now,yaw_last,yaw_d  ,%f,%f,%f,%f\r\n",yaw_start,yaw_now,yaw_last,yaw_d);
        
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      pt_move->step_count ++;
			if (cur_move == 92)
				t_move_step.u_cur_step.e_work_step = WORK_LEG;
    }break;
    default : break; 
  } 

}
void move_work_forward_gyro_task_left(void *param) // 94
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

//  uint32_t leg_gangl = t_move_step.leg_l_angle;
  uint32_t leg_gangl = 15;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      //l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht +
//      move_step_en(MOVE_FOOT_R);
      
			//printf("AT+WORK_OVER,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
				
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_OVER:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
			{
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
			}
    }break;
    default : break; 
  } 

}
void move_work_backward_gyro_task_left(void *param) // 95 
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = 15;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
			l81_tim_task_den(ID_MOVE);
//      l18_move_zero();
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg  - yaw_d*Kp_l); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht +
//      move_step_en(MOVE_FOOT_R);
      
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl ); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {

      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
			{
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_OVER:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				t_move_step.u_cur_step.e_work_step = WORK_LEG;
				pt_move->step_count ++;
      }
    }break;
    default : break; 
  } 

}
void move_turn_left_up_foor_task(void *param) // 71  turn left foot
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

//  uint32_t leg_gangl = 15;
  uint32_t leg_gangl = t_move_step.leg_l_angle;
//  uint32_t foot_gangl = 30;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
//        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
//        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
//        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}

void move_turn_left2_up_foor_task(void *param) // 73  turn right foot
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

//  uint32_t leg_gangl = 15;
  uint32_t leg_gangl = t_move_step.leg_l_angle;
//  uint32_t foot_gangl = 30;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
				move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}

void move_turn_right_up_foor_task(void *param) // 72  
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

//  uint32_t leg_gangl = 15;
  uint32_t leg_gangl = t_move_step.leg_l_angle;
//  uint32_t foot_gangl = 30;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      } 
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

				yaw_d = yaw_start - yaw_last;
				set_yaw_d();
				
				//printf("AT+WORK_LEG2,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}

void move_work_forward2_gyro_task(void *param) // 63  gyro algorithm
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = 15;
//  uint32_t foot_gangl = 30;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
//  uint32_t foot_gangl = foot_gangl_start;
	float Kp = 0.2;
	float Kp_l = 0.1;
//	float Kp = 0;
//	float Kp_l = 0;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      //l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg - yaw_d*Kp_l); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht +
//      move_step_en(MOVE_FOOT_R);
      
			//printf("AT+WORK_OVER,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
			//printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
		yaw_start = 0;
		yaw_now = 0;
		yaw_last = 0;
		yaw_d = 0;
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
				yaw_start = 0;
				yaw_start = yaw_dat;
				yaw_now = yaw_start;
				yaw_last = yaw_now;
				yaw_d = yaw_start - yaw_now;
				set_yaw_d();
				//printf("AT+WORK_FOOT,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_now,yaw_d);
				
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {

				yaw_d = yaw_start - yaw_last;
				set_yaw_d();
				
				//printf("AT+WORK_LEG2,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg - yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				//printf("AT+WORK_LEG3,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_LEG3,yaw_start,yaw_now,yaw_last,yaw_d,    %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl + yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg + yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
				yaw_d = yaw_start - yaw_last;
				set_yaw_d();
				//printf("AT+WORK_FOOT2,yaw_start,yaw_now,yaw_d,  %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_FOOT2,yaw_start,yaw_now,yaw_last,yaw_d,   %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {  
				yaw_last = yaw_now;
				yaw_now = yaw_dat;
				yaw_d = yaw_start - yaw_now;
				set_yaw_d();
				//printf("AT+WORK_FOOT3,yaw_start,yaw_now,yaw_d,  %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_now,yaw_d);
				//printf("AT+WORK_FOOT3,yaw_start,yaw_now,yaw_last,yaw_d,   %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_now,yaw_last,yaw_d);
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				yaw_d = yaw_start - yaw_last;
				set_yaw_d();
				//printf("AT+WORK_LEG4,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_LEG4,yaw_start,yaw_now,yaw_last,yaw_d,    %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl + yaw_d*Kp_l); //right -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg + yaw_d*Kp_l); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				//printf("AT+WORK_LEG5,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_LEG5,yaw_start,yaw_now,yaw_last,yaw_d,    %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg - yaw_d*Kp_l); //left +
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
				//printf("AT+WORK_FOOT4,yaw_start,yaw_now,yaw_d,  %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_FOOT4,yaw_start,yaw_now,yaw_last,yaw_d,   %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      } 
    }break;

    case WORK_FOOT5:
    {
			if(pt_move->step_count == pt_move->step_num -1)
			{
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
				break;
			}
				
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
				yaw_last = yaw_now;
				yaw_now = yaw_dat;
				yaw_d = yaw_start - yaw_now;
				set_yaw_d();
				//printf("AT+WORK_FOOT5,yaw_start,yaw_now,yaw_d,  %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_now,yaw_d);
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //left +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_R);
      } 
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}
void move_turn_left3_up_foor_task(void *param) // 74  turn left foot
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 2;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
			is_in_turn = 0;
      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
//        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
//        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
//        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}

void move_turn_left4_up_foor_task(void *param) // 75  turn left foot
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 1.5;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
			is_in_turn = 0;
      
      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);

    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_R);
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
				move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}

void move_turn_right2_up_foor_task(void *param) // 94  turn right foot (-74)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 2;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
			is_in_turn = 0;
      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}


void move_work_forward_gyro2_task(void *param) // 63  gyro algorithm
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kp = 0.2;
	float Kp_l = 0.1;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
			yaw_start = 0;
			yaw_now = 0;
			yaw_last = 0;
			yaw_d = 0;
      
      //l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg - yaw_d*Kp_l); //left +
//      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        

//        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left +
//        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				yaw_start = 0;
				yaw_start = yaw_dat;
				yaw_now = yaw_start;
				yaw_last = yaw_now;
				yaw_d = yaw_start - yaw_now;
				set_yaw_d();
				//printf("AT+WORK_FOOT,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_now,yaw_d);
				
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {

				yaw_d = yaw_start - yaw_last;
				set_yaw_d();
				//printf("AT+WORK_LEG2,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
//				move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg - yaw_d*Kp_l); //left -
//        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				//printf("AT+WORK_LEG3,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_LEG3,yaw_start,yaw_now,yaw_last,yaw_d,    %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl + yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_L);
        
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg + yaw_d*Kp_l); //left -
//        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				yaw_d = yaw_start - yaw_last;
				set_yaw_d();
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
				//printf("AT+WORK_FOOT2,yaw_start,yaw_now,yaw_d,  %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_FOOT2,yaw_start,yaw_now,yaw_last,yaw_d,   %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
				
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {  
				yaw_last = yaw_now;
				yaw_now = yaw_dat;
				yaw_d = yaw_start - yaw_now;
				set_yaw_d();
				//printf("AT+WORK_FOOT3,yaw_start,yaw_now,yaw_d,  %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_now,yaw_d);
				//printf("AT+WORK_FOOT3,yaw_start,yaw_now,yaw_last,yaw_d,   %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_now,yaw_last,yaw_d);
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				yaw_d = yaw_start - yaw_last;
				set_yaw_d();
				//printf("AT+WORK_LEG4,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_LEG4,yaw_start,yaw_now,yaw_last,yaw_d,    %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
				
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
				if (cur_move == 91)
				{
					yaw_d = 0;
					t_move_step.u_cur_step.e_work_step = WORK_OVER;
				}
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl + yaw_d*Kp_l); //right -
        move_step_en(MOVE_LEG_L);
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg + yaw_d*Kp_l); //right -
//        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				//printf("AT+WORK_LEG5,yaw_start,yaw_now,yaw_d,   %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_LEG5,yaw_start,yaw_now,yaw_last,yaw_d,    %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
//        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg - yaw_d*Kp_l); //left +
//        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl - yaw_d*Kp_l); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				//printf("AT+WORK_FOOT4,yaw_start,yaw_now,yaw_d,  %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_d);
				//printf("AT+WORK_FOOT4,yaw_start,yaw_now,yaw_last,yaw_d,   %f    ,    %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_dat,yaw_last,yaw_d);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl - yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      } 
    }break;

    case WORK_FOOT5:
    {
			if(pt_move->step_count == pt_move->step_num -1)
			{
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
				break;
			}
				
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				yaw_last = yaw_now;
				yaw_now = yaw_dat;
				yaw_d = yaw_start - yaw_now;
				set_yaw_d();
				//printf("AT+WORK_FOOT5,yaw_start,yaw_now,yaw_d,  %f    ,    %f    ,    %f     \r\n",yaw_start,yaw_now,yaw_d);
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //left +
        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl + yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_R);
      } 
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      pt_move->step_count ++;
			
			if (cur_move == 91)
				t_move_step.u_cur_step.e_work_step = WORK_LEG;
			
    }break;
    default : break; 
  } 

}
void move_work_forward_gyro3_task(void *param) // 98 99  no gyro algorithm R 
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      //l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
//			if(pt_move->e_act_set == 98)
//      printf("AT+INT,MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
				
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {  
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      } 
    }break;

    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
				if(pt_move->step_count == pt_move->step_num -1)
					break;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_R);
      } 
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}
void move_work_forward_gyro4_task(void *param) // 98 100  no gyro algorithm L 
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      //l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
//			if(pt_move->e_act_set == 98)
//      printf("AT+INT,MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
				
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {  
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      } 
    }break;

    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
				if(pt_move->step_count == pt_move->step_num -1)
					break;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_R);
      } 
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}
void move_work_backward_r_task(void *param) // 97 101 no gyro algorithm R
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
			l81_tim_task_den(ID_MOVE);
//      l18_move_zero();
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left +
      move_step_en(MOVE_LEG_R);
			if(pt_move->e_act_set == 97)
      printf("AT+INT,MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg ); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {

      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
			{
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {  
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg ); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl ); //ritht -
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;

    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				if(pt_move->step_count == pt_move->step_num -1)
				{
					t_move_step.u_cur_step.e_work_step = WORK_OVER;
					t_move_step.u_last_step.e_work_step = WORK_FOOT5;
					break;
				}
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}
void move_work_backward_l_task(void *param) // 97 102 no gyro algorithm L
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
			l81_tim_task_den(ID_MOVE);
//      l18_move_zero();
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left +
      move_step_en(MOVE_LEG_R);
			if(pt_move->e_act_set == 97)
      printf("AT+INT,MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl ); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {

      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
			{
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {  
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl ); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl ); //ritht -
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;

    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				if(pt_move->step_count == pt_move->step_num -1)
				{
					t_move_step.u_cur_step.e_work_step = WORK_OVER;
					t_move_step.u_last_step.e_work_step = WORK_FOOT5;
					break;
				}
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}
void move_turn_right_2_task(void *param) // 103  turn right foot f
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 1.5;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
			is_in_turn = 0;
      
      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);

    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim*Kleg, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        
//        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
				move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
//        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
//        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}
void move_turn_left_2_task(void *param) // 104  turn left foot f
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 1.5;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
			is_in_turn = 0;
      
      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);

    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_R);
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim*Kleg, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
				move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}
void move_work_goforward_up1_task(void *param) // 69 70 goforward_up
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
//	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      //l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
//			if(pt_move->e_act_set == 98)
//      printf("AT+INT,MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
				
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {  
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      } 
    }break;

    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
				if(pt_move->step_count == pt_move->step_num -1)
					break;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_R);
      } 
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_work_goforward_up2_task(void *param) // goforward_up_1 105 106(115) 107  
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
  uint32_t leg_gangl_min = t_move_step.leg_r_angle;
	uint32_t foot_gangl_min = t_move_step.foot_r_angle;
//	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      //l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl_min); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
//			if(pt_move->e_act_set == 98)
//      printf("AT+INT,MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl_min); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl_min); //left -
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl_min); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
				
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {  
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl_min); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl_min); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      } 
    }break;

    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        
				if(pt_move->step_count == pt_move->step_num -1)
					break;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //ritht +
        move_step_en(MOVE_FOOT_R);
      } 
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_turn_left_up_foor_2_task(void *param) // 108  turn left foot
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
  uint32_t leg_gangl_min = t_move_step.leg_r_angle;
	uint32_t foot_gangl_min = t_move_step.foot_r_angle;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//				move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl_min); //left -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl_min); //right -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_OVER:
    {
      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG;
        pt_move->step_count ++;
      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//      t_move_step.u_cur_step.e_work_step = WORK_LEG;
//      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}

void move_turn_left2_up_foor_2_task(void *param) // 110  turn right foot
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
  uint32_t leg_gangl_min = t_move_step.leg_r_angle;
	uint32_t foot_gangl_min = t_move_step.foot_r_angle;
//	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
				move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_OVER:
    {
      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG;
        pt_move->step_count ++;
      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//      t_move_step.u_cur_step.e_work_step = WORK_LEG;
//      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}

void move_turn_right_up_foor_2_task(void *param) // 109  
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
  uint32_t leg_gangl_min = t_move_step.leg_r_angle;
	uint32_t foot_gangl_min = t_move_step.foot_r_angle;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//				move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl_min); //left -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl_min); //right -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_OVER:
    {
      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG;
        pt_move->step_count ++;
      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//      t_move_step.u_cur_step.e_work_step = WORK_LEG;
//      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}

void move_turn_left2_up_foor_111_task(void *param) // 111  turn right foot
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
  uint32_t leg_gangl_min = t_move_step.leg_r_angle;
	uint32_t foot_gangl_min = t_move_step.foot_r_angle;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
				move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, 150, foot_gangl); //ritht -
//        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_OVER:
    {
      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG;
        pt_move->step_count ++;
      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//      t_move_step.u_cur_step.e_work_step = WORK_LEG;
//      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}

void move_turn_left2_up_foor_112_task(void *param) // 112  turn right foot
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
  uint32_t leg_gangl_min = t_move_step.leg_r_angle;
	uint32_t foot_gangl_min = t_move_step.foot_r_angle;
	float Kleg = 3;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {

//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl/Kleg); //left +
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      
    }break;
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
				move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl); //left -
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //left -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//        move_step_en(MOVE_FOOT_L);
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, 100, foot_gangl); //ritht -
//        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //right -
//        move_step_en(MOVE_LEG_R);
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //right -
        move_step_en(MOVE_LEG_L);
      }
    }break;
    case WORK_OVER:
    {
      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG;
        pt_move->step_count ++;
      }
//      t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//      t_move_step.u_cur_step.e_work_step = WORK_LEG;
//      pt_move->step_count ++;
			
    }break;
    default : break; 
  } 

}

void move_set(E_MOVE_WORK_DIR_TYPDEF e_dir, int32_t step, int32_t delay)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
#if 0 
  int32_t set_step[4]={step,step,step,step};
  int32_t set_angle[4]={20,20,20,20};
  int32_t set_delay[4]={delay,delay,delay,delay};
  
  move_stop_en();
  move_body_init();
  move_body_step(set_step);
  move_body_angle(set_angle);
  move_body_delay(set_delay);
  move_body_cycl_num();
//  move_stop_den();
  
  l81_tim_task_creat(ID_MOVE_SAMPE, TIM_TASK_CYCLE_ALL, MOVE_TIME_SAMPE, NULL, move_sampe_task);
  l81_tim_task_en(ID_MOVE_SAMPE);
  
  t_move_step.u_cur_step.e_work_step = WORK_LEG;
  pt_leg_l->run_flag = 1;
  pt_leg_r->run_flag = 1;
  pt_foot_l->run_flag = 1;
  pt_foot_r->run_flag = 1;
  
//  l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_task);
//  l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_backward_task);
//  l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_bleft_task);
//  l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_bright_task);

//  l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_fleft_task);

//  l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_fright_task);
  
  
  l81_tim_task_en(ID_MOVE);
  #endif
  
  move_stop_en();
  l81_tim_task_den(ID_MOVE);
  
//  t_move_step.u_cur_step.e_work_step = WORK_LEG;
  
  pt_leg_l->run_flag = 1;
  pt_leg_r->run_flag = 1;
  pt_foot_l->run_flag = 1;
  pt_foot_r->run_flag = 1;
  
  t_move_step.u_cur_step.e_work_step = WORK_LEG;
  
  t_move_step.step_num = step;
  t_move_step.step_count = 0;
  t_move_step.foot_tim = delay;
  t_move_step.leg_tim  = delay;
  switch(e_dir)
  {
    case WORK_STOP:
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      l18_move_zero();
    }
    break;
    case WORK_DIR_F:
    {
			yaw_start = 0;
			yaw_start = yaw_dat;
			yaw_now = yaw_start;
			t_move_step.foot_l_angle = 30;
			t_move_step.foot_r_angle = 30;
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_task);
      //l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_gyro3_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    case WORK_DIR_B:
    {
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_backward_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    case WORK_DIR_L: 
    {
      //not good: move_work_fleft_task
      //good :move_work_bleft_task
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_fleft_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    case WORK_DIR_R:
    {
      //not good:move_work_fright_task move_work_bright_task
      //good :
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_fright_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    default :break;
  }
  
}


/*-body Action Unit start-************/
//#if 1 //add wxf-0329

void move_crab_left_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  uint32_t leg_l_angle = 20;
  uint32_t leg_r_angle = 14;
//  uint32_t foot_l_angle = 0;
//  uint32_t foot_r_angle = 0;
  uint32_t delay1=pt_move->leg_tim;
  uint32_t delay2=pt_move->leg_tim;
  
//  uint32_t tim_delay = 500; //ms
//  static uint32_t tim_count = 0;
  
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_leg_l->run_flag)&&(pt_leg_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
//      tim_count += 10;
//      if(tim_count == tim_delay)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_2;
//        tim_count = 0;
//      }
    }break;
    
    case STEP_1:
    {
      if((pt_leg_l->run_flag)&&(pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, delay1, leg_l_angle); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, delay2, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case STEP_2:
    {
      
      if((pt_leg_l->run_flag)&&(pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_2;

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, delay1/2, leg_l_angle); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, delay2, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    
    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 
}

void move_crab_right_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  uint32_t leg_l_angle = 14;
  uint32_t leg_r_angle = 20;
//  uint32_t foot_l_angle = 0;
//  uint32_t foot_r_angle = 0;
  uint32_t delay1=pt_move->leg_tim;
  uint32_t delay2=pt_move->leg_tim;
  
//  uint32_t tim_delay = 500; //ms
//  static uint32_t tim_count = 0;
  
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_leg_l->run_flag)&&(pt_leg_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
//      tim_count += 10;
//      if(tim_count == tim_delay)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_2;
//        tim_count = 0;
//      }
    }break;
    
    case STEP_1:
    {
      if((pt_leg_l->run_flag)&&(pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, delay1, leg_l_angle); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, delay2, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case STEP_2:
    {
      
      if((pt_leg_l->run_flag)&&(pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_2;

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, delay1, leg_l_angle); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, delay2/2, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    
    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  }
}

void move_crab_left1_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  uint32_t leg_l_angle = 10;
  uint32_t leg_r_angle = 45;

  uint32_t delay1=pt_move->leg_tim;
  uint32_t delay2=300;
  
  
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, delay1, 25); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_NONE;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, delay1, leg_l_angle); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, delay1, leg_l_angle); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;

    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, delay1, leg_l_angle); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;

    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}











void move_shake_leg_left_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
	
  uint32_t leg_l_angle = t_move_step.leg_l_angle;
//  uint32_t leg_l_angle = 25;
//  uint32_t leg_r_angle = 45;

  uint32_t delay1=pt_move->leg_tim;
  uint32_t delay2=pt_move->foot_tim;
  uint32_t foot_delay = delay2;
//  uint32_t foot_delay = 30;
  uint32_t foot_l_angle = 5;
  
  uint8_t shake_num = 2;
  static uint8_t shake_count = 0;
  
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag) && (pt_leg_l->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     
    }break;
    case STEP_1: //leg
    {
      if(pt_leg_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, delay1, leg_l_angle); //left +
        move_step_en(MOVE_LEG_L);
      }

    }break;
    case STEP_2: //foot
    {
      if((pt_foot_l->run_flag) && (pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
    }break;
    case STEP_3: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
    }break;
    case STEP_4: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_5;
        t_move_step.u_last_step.e_step_num = STEP_4;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
    }break;
    case STEP_5: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_5;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
        
        shake_count ++;
        if(shake_count > shake_num)
        {
          t_move_step.u_cur_step.e_step_num = STEP_OVER;
        }
      }
    }break;

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      shake_count = 0;
      t_move_step.u_cur_step.e_step_num = STEP_2;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_shake_leg_right_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
	uint32_t leg_r_angle = t_move_step.leg_r_angle;
  uint32_t leg_l_angle = t_move_step.leg_l_angle;
//  uint32_t leg_r_angle = 25;

  uint32_t delay1=pt_move->leg_tim;
  uint32_t delay2=pt_move->foot_tim;
  uint32_t foot_delay = delay2;
//  uint32_t foot_delay = 30;
  uint32_t foot_r_angle = 5;
  
  uint8_t shake_num = 2;
  static uint8_t shake_count = 0;
  
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_r->run_flag) && (pt_leg_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_r_angle); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     
    }break;
    case STEP_1: //leg
    {
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }

    }break;
    case STEP_2: //foot
    {
      if((pt_foot_r->run_flag) && (pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_3: //foot
    {
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_4: //foot
    {
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_5;
        t_move_step.u_last_step.e_step_num = STEP_4;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_5: //foot
    {
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_5;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
        
        shake_count ++;
        if(shake_count > shake_num)
        {
          t_move_step.u_cur_step.e_step_num = STEP_OVER;
        }
      }
    }break;

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      shake_count = 0;
      t_move_step.u_cur_step.e_step_num = STEP_2;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}



void move_shake_foot_left_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_l_angle = 25;
//  uint32_t leg_r_angle = 45;

  uint32_t delay1=pt_move->leg_tim;
  uint32_t foot_delay = 50;
  uint32_t foot_l_angle = 15;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     
    }break;
    case STEP_1: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }

    }break;
    case STEP_2: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
    }break;
    case STEP_3: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
    }break;
    case STEP_4: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_4;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
    }break;


    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_shake_foot_right_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_l_angle = 25;
//  uint32_t leg_r_angle = 45;

  uint32_t delay1=pt_move->leg_tim;
  uint32_t foot_delay = 50;
  uint32_t foot_r_angle = 15;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     
    }break;
    case STEP_1: //foot
    {
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }

    }break;
    case STEP_2: //foot
    {
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_3: //foot
    {
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_4: //foot
    {
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_4;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;


    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_cross_legs_left_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  uint32_t leg_l_angle = 20;
//  uint32_t leg_r_angle = 5;

  uint32_t delay1=pt_move->leg_tim;
  uint32_t foot_delay = 100;
//  uint32_t foot_l_angle = 45;
  uint32_t foot_r_angle = 20;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag)
    {
//      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     
    }break;
    case STEP_1: //foot
    {
      if((pt_leg_l->run_flag) && (pt_foot_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, foot_delay, leg_l_angle); //left +
        move_step_en(MOVE_LEG_L);
        
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }
    break;

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count = pt_move->step_num;
    }break;
    default : break; 
  } 
}

void move_cross_legs_right_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_l_angle = 20;
  uint32_t leg_r_angle = 20;

  uint32_t delay1=pt_move->leg_tim;
  uint32_t foot_delay = 100;
  uint32_t foot_l_angle = 20;
//  uint32_t foot_r_angle = 45;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag)
    {
//      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     
    }break;
    case STEP_1: //foot
    {
      if((pt_leg_r->run_flag) && (pt_foot_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, foot_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
        
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
    }
    break;

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count = pt_move->step_num;
    }break;
    default : break; 
  }
}

void move_lean_left_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_r_angle = 10;
//  uint32_t leg_l_angle = 45;
	uint32_t leg_r_angle = t_move_step.leg_r_angle;
  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t tim_delay = 2000; //ms
  static uint32_t tim_count = 0;

  uint32_t delay1=pt_move->leg_tim;
//  uint32_t foot_delay = 100;
//  uint32_t foot_l_angle = 45;
//  uint32_t foot_r_angle = 45;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag)
    {
//      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     tim_count += 10;
      if(tim_count == tim_delay)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        tim_count = 0;
      }
      
    }break;
    case STEP_1: //foot
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_NONE;
        t_move_step.u_last_step.e_step_num = STEP_1;
        tim_count = 0;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, delay1, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
      }
    }
    break;
    case STEP_2: //foot
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, delay1, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
      }
    }
    break;
    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count = pt_move->step_num;
    }break;
    default : break; 
  }
}
void move_lean_right_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_r_angle = 45;
//  uint32_t leg_l_angle = 10;
	uint32_t leg_r_angle = t_move_step.leg_r_angle;
  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t tim_delay = 2000; //ms
  static uint32_t tim_count = 0;

  uint32_t delay1=pt_move->leg_tim;
//  uint32_t foot_delay = 100;
//  uint32_t foot_l_angle = 45;
//  uint32_t foot_r_angle = 45;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag)
    {
//      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
      tim_count += 10;
      if(tim_count == tim_delay)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        tim_count = 0;
      }
    }break;
    case STEP_1: //foot
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_NONE;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, delay1, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
      }
    }
    break;
    case STEP_2: //foot
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, delay1, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
      }
    }
    break;

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count = pt_move->step_num;
    }break;
    default : break; 
  }
}


void move_stomp_left_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  uint32_t leg_r_angle = 5;
  uint32_t leg_l_angle = 25;
//  uint32_t tim_delay = 2000; //ms
//  static uint32_t tim_count = 0;

  uint32_t delay1=pt_move->leg_tim; //300 OK
//  uint32_t foot_delay = 100;
//  uint32_t foot_l_angle = 45;
//  uint32_t foot_r_angle = 45;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
//      tim_count += 10;
//      if(tim_count == tim_delay)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_2;
//        tim_count = 0;
//      }
    }break;
    case STEP_1: //foot
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, delay1, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
       
      }
    }
    break;
    case STEP_2: //foot
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, 30, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, delay1, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
      }
    }
    break;

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  }
}
void move_stomp_right_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  uint32_t leg_r_angle = 25;
  uint32_t leg_l_angle = 5;
//  uint32_t tim_delay = 2000; //ms
//  static uint32_t tim_count = 0;

  uint32_t delay1=pt_move->leg_tim; //300 OK
//  uint32_t foot_delay = 100;
//  uint32_t foot_l_angle = 45;
//  uint32_t foot_r_angle = 45;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
//      tim_count += 10;
//      if(tim_count == tim_delay)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_2;
//        tim_count = 0;
//      }
    }break;
    case STEP_1: //foot
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, delay1, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
       
      }
    }
    break;
    case STEP_2: //foot
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, 30, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
      }
    }
    break;

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  }
}

void move_shake_body_up_down_task(void *param)  //17    shuangtui tongxiang niu*2
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;
//  uint32_t leg_r_angle = 15;
//  uint32_t leg_l_angle = 15;
//  uint32_t tim_delay = 2000; //ms
//  static uint32_t tim_count = 0;

  uint32_t delay1=pt_move->leg_tim; //pt_move->leg_tim;
//  uint32_t delay1=30; //pt_move->leg_tim; 
//  uint32_t foot_delay = 100;
//  uint32_t foot_l_angle = 45;
//  uint32_t foot_r_angle = 45;
  
  //if(pt_move->step_count >= 3)
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
//      tim_count += 10;
//      if(tim_count == tim_delay)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_2;
//        tim_count = 0;
//      }
    }break;
    case STEP_1: //foot
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, delay1, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
       
      }
    }
    break;
    case STEP_2: //foot
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, delay1, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
      }
    }
    break;
    case STEP_3: //foot
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);            
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, delay1, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
       
      }
    }
    break;
    case STEP_4: //foot
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_4;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, delay1, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
      }
    }
    break;

    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  }
}
void move_shake_body_left_right_task(void *param)  //18 shuangjiao fanxiang xiangwai niu
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_r_angle = 12;
//  uint32_t leg_l_angle = 12;
//  uint32_t tim_delay = 2000; //ms
//  static uint32_t tim_count = 0;

  uint32_t delay1=pt_move->leg_tim;
//  uint32_t delay1=60; //
//  uint32_t foot_delay = 100;
//  uint32_t foot_l_angle = 15;
//  uint32_t foot_r_angle = 15;
//  uint32_t leg_l_angle = t_move_step.leg_l_angle;
//  uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
  
  //if(pt_move->step_count >= 3)
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
//      tim_count += 10;
//      if(tim_count == tim_delay)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_2;
//        tim_count = 0;
//      }
    }break;
    case STEP_1: //foot
    {
      if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, delay1, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, delay1, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_2: //foot
    {
      if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, delay1, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, delay1, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  }
}

void move_shake_head_left_right_task(void *param)  //28 19 shuangjiao tongxiang niu*2
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_r_angle = 12;
//  uint32_t leg_l_angle = 12;
//  uint32_t tim_delay = 2000; //ms
//  static uint32_t tim_count = 0;

//  uint32_t delay1=80;
//  uint32_t foot_delay = 100;
//  uint32_t foot_l_angle = 12;
//  uint32_t foot_r_angle = 12;//
  uint32_t delay1 = pt_move->leg_tim;
  uint32_t foot_l_angle = t_move_step.foot_r_angle;
  uint32_t foot_r_angle = t_move_step.foot_r_angle;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
//      tim_count += 10;
//      if(tim_count == tim_delay)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_2;
//        tim_count = 0;
//      }
    }break;
    case STEP_1: //foot
    {
      if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, delay1, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, delay1, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_2: //foot
    {
      if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, delay1, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, delay1, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_3: //foot
    {
      if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, delay1, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, delay1, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_4: //foot
    {
      if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_4;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, delay1, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, delay1, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  }
}


void move_stand_ease_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  uint32_t leg_r_angle = 10;
//  uint32_t leg_l_angle = 12;
//  uint32_t tim_delay = 2000; //ms
//  static uint32_t tim_count = 0;

  uint32_t delay1=100; //
//  uint32_t foot_delay = 100;
  uint32_t foot_l_angle = 10;
  uint32_t foot_r_angle = 25;
  
  if(pt_move->step_count >= 1)
  {
    if((pt_leg_r->run_flag) && (pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
//      tim_count += 10;
//      if(tim_count == tim_delay)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_2;
//        tim_count = 0;
//      }
    }break;
    case STEP_1: //foot
    {
      if((pt_leg_r->run_flag) && (pt_foot_r->run_flag) && (pt_foot_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, delay1, leg_r_angle); // 
        move_step_en(MOVE_LEG_R);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, delay1, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, delay1, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
       
      }
    }
    break;
    

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  }
}

void move_turn_left_task(void *param)  //21
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_r_angle = 10;
//  uint32_t leg_l_angle = 12;
  uint32_t tim_delay1 = 300; //ms
  uint32_t tim_delay2 = 600; //ms
  uint32_t tim_delay3 = 900; //ms
  static uint32_t tim_count = 0;

  uint32_t delay1 = pt_move->leg_tim;
  uint32_t delay2 = 2*pt_move->leg_tim;
  //uint32_t delay1=300; 
  //uint32_t delay2 = 300;
//  uint32_t foot_l_angle = 45; //
//  uint32_t foot_r_angle = 45; //
	uint32_t foot_l_angle = t_move_step.foot_l_angle;//45
	uint32_t foot_r_angle = t_move_step.foot_r_angle;//45
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      l81_move_body_write(MOVE_LEG_L, MOVE_L, 0);
			l81_move_body_write(MOVE_LEG_R, MOVE_L, 0);
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
      tim_count += 10;
      if(tim_count == tim_delay1)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
//        tim_count = 0;
      }
      else if(tim_count == tim_delay2)
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
      }
      else if(tim_count == tim_delay3)
      {
        t_move_step.u_cur_step.e_step_num = STEP_1;
        tim_count = 0;
      }
      
    }break;
    case STEP_1: //foot
    {
      if(pt_foot_l->run_flag)
      {
        tim_count = 0;
        t_move_step.u_cur_step.e_step_num = STEP_NONE;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, delay1, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
      }
    }
    break;
    case STEP_2: //foot
    {
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_NONE;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, delay1, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_3: //foot
    {
      if((pt_foot_r->run_flag) && (pt_foot_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_3;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, delay2, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, delay2, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_NONE;
      pt_move->step_count ++;
    }break;
    default : break; 
  }
}
void move_turn_right_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_r_angle = 10;
//  uint32_t leg_l_angle = 12;
  uint32_t tim_delay1 = 300; //ms
  uint32_t tim_delay2 = 600; //ms
  uint32_t tim_delay3 = 900; //ms
  static uint32_t tim_count = 0;

  uint32_t delay1 = pt_move->leg_tim;
  uint32_t delay2 = 2*pt_move->leg_tim;
//  uint32_t delay1=300; //
//  uint32_t delay2 = 300;
//  uint32_t foot_l_angle = 45;//45
//  uint32_t foot_r_angle = 45;//45
	uint32_t foot_l_angle = t_move_step.foot_l_angle;//45
	uint32_t foot_r_angle = t_move_step.foot_r_angle;//45
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      l81_move_body_write(MOVE_LEG_L, MOVE_L, 0);
			l81_move_body_write(MOVE_LEG_R, MOVE_L, 0);
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
      tim_count += 10;
      if(tim_count == tim_delay1)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
//        tim_count = 0;
      }
      else if(tim_count == tim_delay2)
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
      }
      else if(tim_count == tim_delay3)
      {
        t_move_step.u_cur_step.e_step_num = STEP_1;
        tim_count = 0;
      }
      
    }break;
    case STEP_1: //foot
    {
      if(pt_foot_r->run_flag)
      {
        tim_count = 0;
        t_move_step.u_cur_step.e_step_num = STEP_NONE;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, delay1, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
      }
    }
    break;
    case STEP_2: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_NONE;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, delay1, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
       
      }
    }
    break;
    case STEP_3: //foot
    {
      if((pt_foot_r->run_flag) && (pt_foot_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_3;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, delay2, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, delay2, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_NONE;
      pt_move->step_count ++;
    }break;
    default : break; 
  }
}

void move_clamp_leg_task(void *param)  //23
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

  uint32_t leg_gangl = 20;
  uint32_t foot_gangl = 25;
  uint32_t delay=100; 
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, delay, leg_gangl); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, delay, leg_gangl); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, delay, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, delay, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag)
      {

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, delay, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, delay, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_shake_foot_right_tempo_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_l_angle = 25;
//  uint32_t leg_r_angle = 45;

  uint32_t delay1=pt_move->leg_tim;
  uint32_t foot_delay1 = 350;
  uint32_t foot_delay2 = 300;
  uint32_t foot_r_angle = t_move_step.foot_r_angle;
//  uint32_t foot_r_angle = 15;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     
    }break;
    case STEP_1: //foot
    {
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_r_angle); //left +
//        move_step_en(MOVE_FOOT_R);
      }

    }break;
    case STEP_2: //foot
    {
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay2, foot_r_angle); //left +
//        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_3: //foot
    {
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_4: //foot
    {
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_4;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay2, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;


    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}
void move_sway_task(void *param)  //27
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  uint32_t leg_l_angle = 15;
  uint32_t leg_r_angle = 15;

  uint32_t delay1=pt_move->leg_tim;
  uint32_t foot_delay = 300;
  uint32_t foot_r_angle = 5;
  
  uint8_t shake_num = 2;
  static uint8_t shake_count = 0;
	
	
  uint32_t tim_delay1 = 600; //ms
  uint32_t tim_delay2 = 600; //ms
  static uint32_t tim_count = 0;
  
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_r->run_flag) && (pt_leg_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, 500, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     tim_count += 10;
//      if(tim_count == tim_delay1)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_OVER;
////        tim_count = 0;
//      }
      if(tim_count == tim_delay2)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        tim_count = 0;
      }
      
    }break;
    case STEP_1: //leg
    {
			
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
    }break;
    case STEP_2: //foot
    {
      if((pt_leg_l->run_flag))
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
      if(pt_leg_r->run_flag)
      {
//        t_move_step.u_cur_step.e_step_num = STEP_NONE;
				t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }

    }break;
    case STEP_3: //foot
    {
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
    }break;
    case STEP_4: //leg
    {
      if((pt_leg_l->run_flag))
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_4;

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
//    case STEP_5: //foot
//    {
//      if(pt_foot_r->run_flag)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_2;
//        t_move_step.u_last_step.e_step_num = STEP_5;
//        
//        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_r_angle); //left +
//        move_step_en(MOVE_FOOT_R);
//        
//        shake_count ++;
//        if(shake_count > shake_num)
//        {
//          t_move_step.u_cur_step.e_step_num = STEP_OVER;
//        }
//      }
//    }break;

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      shake_count = 0;
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}



void move_shake_foot_in_half_task(void *param) //34 shuangjiao fanxiang xiangnei yiban
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_r_angle = 12;
//  uint32_t leg_l_angle = 12;
//  uint32_t tim_delay = 2000; //ms
//  static uint32_t tim_count = 0;

//  uint32_t delay1=60; 
	uint32_t delay1=pt_move->leg_tim;
//  uint32_t foot_delay = 100;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
  uint32_t foot_r_angle = t_move_step.foot_r_angle;
//  uint32_t foot_l_angle = 15;
//  uint32_t foot_r_angle = 15;
  
  if(pt_move->step_count >= 3)
  {
    if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
//      tim_count += 10;
//      if(tim_count == tim_delay)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_2;
//        tim_count = 0;
//      }
    }break;
    case STEP_1: //foot
    {
      if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, delay1, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, delay1, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_2: //foot
    {
      if((pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, delay1, foot_l_angle); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, delay1, foot_r_angle); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  }
}

void move_shake_foot_task(void *param)  //28   //shuangjiao fanxiang xiangwai niu*2
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_l_angle = 25;
//  uint32_t leg_r_angle = 45;

  uint32_t foot_delay=pt_move->leg_tim;
//  uint32_t foot_delay = 50;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
//  uint32_t foot_l_angle = 15;
//  uint32_t foot_r_angle = 15;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     
    }break;
    case STEP_1: //foot
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
				
				if(foot_l_angle)
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_l_angle);
				if(foot_r_angle)
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_r_angle);
      }

    }break;
    case STEP_2: //foot
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
				if(foot_l_angle)
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_4, foot_delay, foot_l_angle);
				if(foot_r_angle)
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay, foot_r_angle);
      } 
    }break;
    case STEP_3: //foot
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;
				if(foot_l_angle)
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_l_angle);
				if(foot_r_angle)
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_r_angle);
      } 
    }break;
    case STEP_4: //foot
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_4;
				if(foot_l_angle)
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_l_angle);
				if(foot_r_angle)
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_r_angle);
      } 
    }break;


    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_incline_left_task(void *param) //29 zuoqing
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_l_angle = 25;
//  uint32_t leg_r_angle = 25;
	uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;
	

  uint32_t delay1=pt_move->leg_tim;
  uint32_t foot_delay = 300;
  uint32_t foot_r_angle = 5;
  
  uint8_t shake_num = 2;
  static uint8_t shake_count = 0;
	
	
  uint32_t tim_delay1 = 600; //ms
  uint32_t tim_delay2 = 600; //ms
  static uint32_t tim_count = 0;
  
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_r->run_flag) && (pt_leg_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, 500, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     tim_count += 10;
//      if(tim_count == tim_delay1)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_OVER;
////        tim_count = 0;
//      }
      if(tim_count == tim_delay2)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        tim_count = 0;
      }
      
    }break;
    case STEP_1: //leg
    {
			
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
    }break;
    case STEP_2: //foot
    {
      if((pt_leg_l->run_flag))
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
      if(pt_leg_r->run_flag)
      {
//        t_move_step.u_cur_step.e_step_num = STEP_NONE;
				t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_2;

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }

    }break;
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      shake_count = 0;
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}



void move_incline_right_task(void *param) //30 youqing
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
//  uint32_t leg_l_angle = 30;
//  uint32_t leg_r_angle = 30;
	uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;

  uint32_t delay1=pt_move->leg_tim;
  uint32_t foot_delay = 300;
  uint32_t foot_r_angle = 5;
  
  uint8_t shake_num = 2;
  static uint8_t shake_count = 0;
	
	
  uint32_t tim_delay1 = 600; //ms
  uint32_t tim_delay2 = 600; //ms
  static uint32_t tim_count = 0;
  
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_r->run_flag) && (pt_leg_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, 500, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     tim_count += 10;
//      if(tim_count == tim_delay1)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_OVER;
////        tim_count = 0;
//      }
      if(tim_count == tim_delay2)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        tim_count = 0;
      }
      
    }break;
    case STEP_1: //foot
    {
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
    }break;
    case STEP_2: //leg
    {
      if((pt_leg_l->run_flag))
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, delay1, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
      if(pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_2;

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, delay1, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
//    case STEP_5: //foot
//    {
//      if(pt_foot_r->run_flag)
//      {
//        t_move_step.u_cur_step.e_step_num = STEP_2;
//        t_move_step.u_last_step.e_step_num = STEP_5;
//        
//        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_r_angle); //left +
//        move_step_en(MOVE_FOOT_R);
//        
//        shake_count ++;
//        if(shake_count > shake_num)
//        {
//          t_move_step.u_cur_step.e_step_num = STEP_OVER;
//        }
//      }
//    }break;

    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      shake_count = 0;
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}



void move_clamp_leg_shake_foot_in_task(void *param)  //59   //bingjia shuangjiao tongxiang xiangnei peng*2
{ //ACT_CLAMP_LEG_SHAKE_FOOT_IN_LR
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;

  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t foot_delay=pt_move->leg_tim;
//  uint32_t foot_delay = 50;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
//  uint32_t foot_l_angle = 15;
//  uint32_t foot_r_angle = 15;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      //l18_move_zero();
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_l_angle); //left +
      move_step_en(MOVE_LEG_L);
      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     
    }break;
    case STEP_1: //leg
    {
			
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
    }break;
    case STEP_2: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_6;
        t_move_step.u_last_step.e_step_num = STEP_2;
				
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_6;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }

    }break;
    case STEP_6: //leg
    {
			
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_7;
        t_move_step.u_last_step.e_step_num = STEP_6;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
    }break;
    case STEP_7: //leg
    {
			
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_7;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
    }break;
    case STEP_3: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_8;
        t_move_step.u_last_step.e_step_num = STEP_3;
				
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_8;
        t_move_step.u_last_step.e_step_num = STEP_3;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_8: //leg
    {
			
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_9;
        t_move_step.u_last_step.e_step_num = STEP_8;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
    }break;
    case STEP_9: //leg
    {
			
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_9;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
    }break;
    case STEP_4: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_10;
        t_move_step.u_last_step.e_step_num = STEP_4;
				
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_10;
        t_move_step.u_last_step.e_step_num = STEP_4;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_10: //leg
    {
			
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_11;
        t_move_step.u_last_step.e_step_num = STEP_10;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
    }break;
    case STEP_11: //leg
    {
			
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_5;
        t_move_step.u_last_step.e_step_num = STEP_11;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
    }break;
    case STEP_5: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_5;
				
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_5;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;


    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_2;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_clamp_leg_shake_foot_task(void *param)  //60   //bingjia shuangjiao tongxiang xiangnei peng
{ //ACT_CLAMP_LEG_SHAKE_FOOT_LR
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;

  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t foot_delay=pt_move->leg_tim;
//  uint32_t foot_delay = 50;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
//  uint32_t foot_l_angle = 15;
//  uint32_t foot_r_angle = 15;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      //l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim, leg_l_angle); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_r_angle); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
     
    }break;
    case STEP_1: //leg
    {
			
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
      if(pt_foot_l->run_flag)
      {
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
				
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_2: //foot
    {
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
      if(pt_foot_l->run_flag)
      {
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }

    }break;
    case STEP_3: //foot
    {
      if(pt_leg_r->run_flag)
      {

        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
      if(pt_foot_l->run_flag)
      {
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_4: //foot
    {
      if(pt_leg_r->run_flag)
      {
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
      }
      if((pt_leg_l->run_flag))
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);
        
      }
      if(pt_foot_l->run_flag)
      {
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
      }
      if(pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_4;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_open_leg_task(void *param)  //61 
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

//  uint32_t leg_r_angle = 12;
//  uint32_t leg_l_angle = 12;
	uint32_t leg_s_angle = 0;
  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
//  uint32_t leg_gangl = 15;
//  uint32_t foot_gangl = 25;
  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t foot_delay=pt_move->foot_tim;
  uint32_t delay=leg_delay* leg_s_angle/leg_l_angle ; 
//  uint32_t leg_delay=60;
  
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_gangl); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, leg_delay, leg_gangl); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
				if(leg_l_angle)
				move_body_set(MOVE_LEG_L, MOVE_SHOCK_1, leg_delay, leg_l_angle);
				if(leg_r_angle)
				move_body_set(MOVE_LEG_R, MOVE_SHOCK_3, leg_delay, leg_r_angle);
				if(foot_l_angle)
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_l_angle);
				if(foot_r_angle)
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_r_angle);
//				if(leg_l_angle){
//        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, leg_delay, leg_l_angle); //left +
//        move_step_en(MOVE_LEG_L);
//        }
//				if(leg_l_angle){
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, leg_delay, leg_r_angle); //left +
//        move_step_en(MOVE_LEG_R);
//				}
//				if(foot_l_angle)
//				{
//					move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_l_angle); //left +
//					move_step_en(MOVE_FOOT_L);
//				}
//				if(foot_r_angle)
//				{
//					move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_r_angle); //left +
//					move_step_en(MOVE_FOOT_R);
//				}
			}
    }break;
    
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
				if(leg_s_angle == 0)
					t_move_step.u_cur_step.e_work_step = WORK_OVER;
				if(leg_l_angle)
				move_body_set(MOVE_LEG_L, MOVE_SHOCK_2, leg_delay, leg_l_angle);
				if(leg_r_angle)
				move_body_set(MOVE_LEG_R, MOVE_SHOCK_4, leg_delay, leg_r_angle);
				if(foot_l_angle)
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_4, foot_delay, foot_l_angle);
				if(foot_r_angle)
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay, foot_r_angle);
				
//				if(leg_l_angle){
//        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, leg_delay, leg_l_angle); //left +
//        move_step_en(MOVE_LEG_L);
//        }
//				if(leg_l_angle){
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, leg_delay, leg_r_angle); //left +
//        move_step_en(MOVE_LEG_R);
//				}
//				if(foot_l_angle)
//				{
//					move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, foot_delay, foot_l_angle); //left +
//					move_step_en(MOVE_FOOT_L);
//				}
//				if(foot_r_angle)
//				{
//					move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay, foot_r_angle); //left +
//					move_step_en(MOVE_FOOT_R);
//				}
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
				if(leg_l_angle)
				move_body_set(MOVE_LEG_L, MOVE_SHOCK_3, leg_delay, leg_l_angle);
				if(leg_r_angle)
				move_body_set(MOVE_LEG_R, MOVE_SHOCK_1, leg_delay, leg_r_angle);
				if(foot_l_angle)
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_l_angle);
				if(foot_r_angle)
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_r_angle);
				
//				if(leg_l_angle){
//        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, leg_delay, leg_l_angle); //left +
//        move_step_en(MOVE_LEG_L);
//        }
//				if(leg_l_angle){
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, leg_delay, leg_r_angle); //left +
//        move_step_en(MOVE_LEG_R);
//				}
//				if(foot_l_angle)
//				{
//					move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_l_angle); //left +
//					move_step_en(MOVE_FOOT_L);
//				}
//				if(foot_r_angle)
//				{
//					move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_r_angle); //left +
//					move_step_en(MOVE_FOOT_R);
//				}

//        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, leg_delay, leg_s_angle); //left -
//        move_step_en(MOVE_LEG_L);
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, leg_delay, leg_s_angle); //left -
//        move_step_en(MOVE_LEG_R);
//				
//				if(foot_l_angle || foot_r_angle)
//				{
//					move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, leg_delay, foot_l_angle); //left +
//					move_step_en(MOVE_FOOT_L);
//				}
//					move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_r_angle); //left +
//					move_step_en(MOVE_FOOT_R);
//					printf("foot_r_angle,%d \r\n",foot_r_angle);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				if(leg_l_angle)
				move_body_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_l_angle);
				if(leg_r_angle)
				move_body_set(MOVE_LEG_R, MOVE_SHOCK_2, leg_delay, leg_r_angle);
				if(foot_l_angle)
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_l_angle);
				if(foot_r_angle)
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_r_angle);
//				
//				if(leg_l_angle){
//        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_l_angle); //left +
//        move_step_en(MOVE_LEG_L);
//        }
//				if(leg_l_angle){
//        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, leg_delay, leg_r_angle); //left +
//        move_step_en(MOVE_LEG_R);
//				}
//				if(foot_l_angle)
//				{
//					move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_l_angle); //left +
//					move_step_en(MOVE_FOOT_L);
//				}
//				if(foot_r_angle)
//				{
//					move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_r_angle); //left +
//					move_step_en(MOVE_FOOT_R);
//				}

      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_open_leg_shake_task(void *param)  //62
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

//  uint32_t leg_r_angle = 12;
//  uint32_t leg_l_angle = 12;
  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
//  uint32_t leg_gangl = 15;
//  uint32_t foot_gangl = 25;
  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t foot_delay=pt_move->foot_tim;
  uint32_t delay=leg_delay/leg_l_angle * leg_r_angle; 
//  uint32_t leg_delay=60;
  
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_gangl); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, leg_delay, leg_gangl); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
				printf("leg_l_angle  %d\r\n",leg_l_angle);
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, leg_delay, leg_l_angle); //left +
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
			}
    }break;
    case WORK_FOOT:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;

				
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
			}
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;

				
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;

        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, leg_delay, leg_l_angle); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, leg_delay, leg_r_angle); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;

				
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;

        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_r_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_open_leg2_task(void *param)  //70
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

//  uint32_t leg_r_angle = 12;
//  uint32_t leg_l_angle = 12;
	uint32_t leg_s_angle = 0;
  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
//  uint32_t leg_gangl = 15;
//  uint32_t foot_gangl = 25;
  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t foot_delay=pt_move->foot_tim;
  uint32_t delay=leg_delay* leg_s_angle/leg_l_angle ; 
//  uint32_t leg_delay=60;
  
  static uint32_t i = 0;
  static uint32_t rt = 0;
  uint32_t rt_max = leg_delay/100 +1;
  uint32_t all_num = 16;
  uint32_t s_num = 36;
	
	
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      //l18_move_zero();
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
			if(rt++<rt_max)
				break;
			rt = 0;
			//motor_set_pulse(4, 1500-s_num*i);
			motor_set_pulse(3, 1500+s_num*i);
			if(++i>all_num)
			{
				i = 0;
        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
			}
    }break;
    
    case WORK_LEG2:
    {
			if(rt++<rt_max)
				break;
			rt = 0;
			//motor_set_pulse(4, (1500-s_num*all_num)+s_num*i);
			motor_set_pulse(3, (1500+s_num*all_num)-s_num*i);
			if(++i>all_num/2)
			{
				i = 0;
        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
			}
    }break;
    case WORK_LEG3:
    {
			if(rt++<rt_max)
				break;
			rt = 0;
			//motor_set_pulse(4, (1500-s_num*all_num/2)+s_num*i);
			motor_set_pulse(3, (1500+s_num*all_num/2)+s_num*i);
			if(++i>all_num/4)
			{
				i = 0;
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
			}
    }break;
    case WORK_LEG4:
    {
			if(rt++<rt_max)
				break;
			rt = 0;
			//motor_set_pulse(4, (1500-s_num*all_num/2)+s_num*i);
			motor_set_pulse(3, (1500+s_num*all_num/2+s_num*all_num/4)-s_num*i);
			if(++i>all_num/4)
			{
				i = 0;
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
			}
    }break;
    case WORK_LEG5:
    {
			if(rt++<rt_max)
				break;
			rt = 0;
			//motor_set_pulse(4, (1500-s_num*all_num/2)+s_num*i);
			motor_set_pulse(3, (1500+s_num*all_num/2)+s_num*i);
			if(++i>all_num/4)
			{
				i = 0;
        t_move_step.u_cur_step.e_work_step = WORK_LEG6;
        t_move_step.u_last_step.e_work_step = WORK_LEG5;
			}
    }break;
    case WORK_LEG6:
    {
			if(rt++<rt_max)
				break;
			rt = 0;
			//motor_set_pulse(4, (1500-s_num*all_num/2+s_num*all_num/4)+s_num*i);
			motor_set_pulse(3, (1500+s_num*all_num/2+s_num*all_num/4)-s_num*i);
			if(++i>all_num/2)
			{
				i = 0;
        t_move_step.u_cur_step.e_work_step = WORK_LEG7;
        t_move_step.u_last_step.e_work_step = WORK_LEG6;
			}
    }break;
    case WORK_LEG7:
    {
			if(rt++<rt_max)
				break;
			rt = 0;
			//motor_set_pulse(4, (1500-s_num*all_num)+s_num*i);
			motor_set_pulse(3, (1500+s_num*all_num/4)-s_num*i);
			if(++i>all_num/4)
			{
				i = 0;
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_LEG7;
			}
    }break;
    
    case WORK_OVER:
    {
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_open_leg_big_angle_task(void *param)  //70
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t foot_delay=pt_move->foot_tim;
  uint32_t delay=leg_delay/leg_l_angle * leg_r_angle; 
  uint32_t tim_delay1 = 100; //ms
  uint32_t tim_delay2 = 200; //ms
  uint32_t tim_delay3 = 3000; //ms
  uint32_t tim_delay4 = 4000; //ms
  static uint32_t tim_count = 0;
  
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_leg_l->run_flag && pt_leg_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      //l18_move_zero(); 
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }


  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
			tim_count += 10;
			//printf("tim_count,%d\r\n",tim_count);
      if(tim_count == tim_delay1)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        //tim_count = 0;
      }else if(tim_count >= tim_delay2)
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        tim_count = 0;
      }
     
    }break;
    case STEP_1:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1; 
        move_body_set(MOVE_LEG_L, MOVE_SHOCK_1, leg_delay, leg_l_angle); 
        move_body_set(MOVE_LEG_R, MOVE_SHOCK_3, leg_delay, leg_l_angle); 
			}
    }break;
    case STEP_2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, leg_delay, leg_l_angle); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, leg_delay, leg_l_angle); //left +
        move_step_en(MOVE_LEG_R);
//        move_body_set(MOVE_LEG_L, MOVE_SHOCK_2, leg_delay, leg_r_angle); 
//        move_body_set(MOVE_LEG_R, MOVE_SHOCK_4, leg_delay, leg_r_angle);  
      }
    }break;
    case STEP_3:
    {
//				printf("STEP_3 leg_delay,%d\r\n",leg_delay);
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
//        move_body_set(MOVE_LEG_L, MOVE_SHOCK_1, leg_delay, leg_r_angle); 
//        move_body_set(MOVE_LEG_R, MOVE_SHOCK_3, leg_delay, leg_r_angle); 
			}
    }break;
    case STEP_4:
    {
//				printf("STEP_4 leg_delay,%d\r\n",leg_delay);
      if(pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_4;
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, leg_delay, leg_r_angle); //left +
        move_step_en(MOVE_LEG_R);
//        move_body_set(MOVE_LEG_L, MOVE_SHOCK_2, leg_delay, leg_l_angle); 
//        move_body_set(MOVE_LEG_R, MOVE_SHOCK_4, leg_delay, leg_l_angle); 
      }
    }break;
    
    case STEP_OVER:
    { 
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_gesture_task1(void *param)  //76 gave a nod  or  77 fist bump
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  //uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	//uint32_t foot_r_angle = t_move_step.foot_r_angle;

  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t delay1=pt_move->foot_tim;
  static uint32_t tim_count = 0;
	
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_foot_l->run_flag && (pt_leg_l->run_flag && pt_leg_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();
      //tim_count = 0;
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }

  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
      tim_count += 10;
      if(tim_count >= delay1)
      {
				tim_count = 0;
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_NONE;
				
      }
    }break;
    case STEP_1: //leg
    {
			
      if(pt_foot_l->run_flag && pt_leg_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_NONE;
        t_move_step.u_last_step.e_step_num = STEP_1;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);

        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, leg_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, leg_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case STEP_2: //foot
    {
      if(pt_foot_l->run_flag)
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_2;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, leg_delay, leg_l_angle); // 
        move_step_en(MOVE_LEG_L);

        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, leg_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, leg_delay, foot_l_angle); //left +
        move_step_en(MOVE_FOOT_R);
			}
    }break;
    case STEP_OVER:
    {
      t_move_step.u_cur_step.e_step_num = STEP_1;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}
void move_excited_task(void *param)  //78
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

  uint32_t leg_gangl = t_move_step.leg_l_angle;
  //uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
//  uint32_t leg_gangl = 20;
//  uint32_t foot_gangl = 25;
  uint32_t delay=pt_move->leg_tim;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, delay, leg_gangl); //left +
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, delay, leg_gangl); //left +
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //left +
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_gangl); //ritht +
//      move_step_en(MOVE_FOOT_R);
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }
  switch(t_move_step.u_cur_step.e_step_num)
  {
    case WORK_NONE:
    {
     
    }break;
    case STEP_1:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, delay, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, delay, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case STEP_2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, delay, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, delay, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case STEP_3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {

        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, delay, leg_gangl); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, delay, leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      } 
    }break;
    
    case STEP_4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_5;
        t_move_step.u_last_step.e_step_num = STEP_4;
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, delay, leg_gangl); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, delay, leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
		case STEP_5: //foot
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_6;
        t_move_step.u_last_step.e_step_num = STEP_5;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_6: //foot
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_7;
        t_move_step.u_last_step.e_step_num = STEP_6;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_7: //foot
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_8;
        t_move_step.u_last_step.e_step_num = STEP_7;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_8: //foot
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_9;
        t_move_step.u_last_step.e_step_num = STEP_8;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
		case STEP_9: //foot
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_10;
        t_move_step.u_last_step.e_step_num = STEP_9;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_10: //foot
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_11;
        t_move_step.u_last_step.e_step_num = STEP_10;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_11: //foot
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_12;
        t_move_step.u_last_step.e_step_num = STEP_11;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_12: //foot
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_12;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}
void move_Rock_task(void *param)  //79  76-79 for Gesture movements
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

  uint32_t leg_gangl = t_move_step.leg_l_angle;
  //uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
  uint32_t delay=pt_move->leg_tim;
  
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }
  switch(t_move_step.u_cur_step.e_step_num)
  {
    case WORK_NONE:
    {
     
    }break;
    case STEP_1:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_2;
        t_move_step.u_last_step.e_step_num = STEP_1;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
      }
    }break;
    
    case STEP_2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
      }
    }break;
    case STEP_3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {

        t_move_step.u_cur_step.e_step_num = STEP_4;
        t_move_step.u_last_step.e_step_num = STEP_3;

        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
      } 
    }break;
    
    case STEP_4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_5;
        t_move_step.u_last_step.e_step_num = STEP_4;
        
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
      }
    }break;
		case STEP_5: //foot
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_6;
        t_move_step.u_last_step.e_step_num = STEP_5;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_6: //foot
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_7;
        t_move_step.u_last_step.e_step_num = STEP_6;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_7: //foot
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_8;
        t_move_step.u_last_step.e_step_num = STEP_7;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    case STEP_8: //foot
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && (pt_foot_l->run_flag) && (pt_foot_r->run_flag))
      {
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_8;
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, delay, foot_gangl); // 
        move_step_en(MOVE_FOOT_R);
       
      }
    }
    break;
    
    case STEP_OVER:
    {
//      if((pt_foot_l->run_flag)&&(pt_foot_r->run_flag))
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
//        pt_move->step_count ++;
//      }
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_dance_task(void *param) // 80 dance   
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  
//  uint32_t leg_gangl = 18;
//  uint32_t foot_l_gangl = 30;
//  uint32_t foot_r_gangl = 30;
  

  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
//  pt_move->leg_tim = 300; //wxf-test
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      
//      l18_move_zero();
//      move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); 
//      move_step_en(MOVE_LEG_L);
//      move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); 
//      move_step_en(MOVE_LEG_R);
//      move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim, foot_l_gangl); 
//      move_step_en(MOVE_FOOT_L);
//      move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim, foot_r_gangl); 
//      move_step_en(MOVE_FOOT_R);
      
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }
  
  switch(t_move_step.u_cur_step.e_step_num)
  {
    case STEP_NONE:
    {
      
    }break;
    
    case STEP_1:
    {
      
      move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, pt_move->leg_l_angle); 
      move_step_en(MOVE_LEG_L);
    
      t_move_step.u_cur_step.e_step_num = STEP_2;
      t_move_step.u_last_step.e_step_num = STEP_1;

    }break;
    case STEP_2:
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, pt_move->leg_l_angle); 
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, pt_move->leg_l_angle); 
        move_step_en(MOVE_LEG_R);
        
        t_move_step.u_cur_step.e_step_num = STEP_3;
        t_move_step.u_last_step.e_step_num = STEP_2;
//        tim1_count = 0;
      }
      
      

    }break;
    
    case STEP_3:
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, pt_move->leg_l_angle); 
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, pt_move->leg_l_angle); 
        move_step_en(MOVE_LEG_R);
      
        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_3;
      }
      
    }break;
    case STEP_4:
    {
      if((pt_leg_l->run_flag) && (pt_leg_r->run_flag))
      {
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, pt_move->leg_l_angle); 
        move_step_en(MOVE_LEG_L);

        t_move_step.u_cur_step.e_step_num = STEP_OVER;
        t_move_step.u_last_step.e_step_num = STEP_4;
      }
      
    }break;
    
    
    case STEP_OVER:
    {
      if((pt_leg_r->run_flag)&&(pt_leg_r->run_flag))
      {
        
        t_move_step.u_cur_step.e_step_num = STEP_2;
        pt_move->step_count++;
        
        
        if((pt_move->step_count == pt_move->step_num) && (t_move_step.u_last_step.e_step_num != STEP_4))
        {
          t_move_step.u_cur_step.e_step_num = STEP_4;
          pt_move->step_count --;
        }
      }
      
      
    }break;
    default : break; 
  } 
}


void move_step_lf_task(void *param)  //81 Step lf
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t foot_delay=pt_move->foot_tim;
	float Kleg = 3;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);

      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }

  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
				if(last_move != 82)
				{
					move_body_set(MOVE_LEG_L, MOVE_SHOCK_3, leg_delay, leg_l_angle);
					move_body_set(MOVE_LEG_R, MOVE_SHOCK_3, leg_delay, leg_r_angle/Kleg);
				}
				else{
					move_body_set(MOVE_FOOT_L, MOVE_SHOCK_4, foot_delay, foot_l_angle);
					move_body_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_r_angle);
				}
			}
    }break;
    
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
				if(last_move != 82)
				{
					move_body_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_l_angle);
					move_body_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_r_angle);
				}
				else{
					move_body_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_l_angle);
					move_body_set(MOVE_LEG_R, MOVE_SHOCK_4, leg_delay, leg_r_angle/Kleg);
				}
      }
    }break;
    
    case WORK_OVER:
    {
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_step_lb_task(void *param)  //82 Step lb
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t foot_delay=pt_move->foot_tim;
	float Kleg = 3;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);

      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }

  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
				if(last_move != 81)
				{
					move_body_set(MOVE_LEG_L, MOVE_SHOCK_3, leg_delay, leg_l_angle);
					move_body_set(MOVE_LEG_R, MOVE_SHOCK_3, leg_delay, leg_r_angle/Kleg);
				}
				else{
					move_body_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_l_angle);
					move_body_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay, foot_r_angle);
				}
			}
    }break;
    
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
				if(last_move != 81)
				{
					move_body_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_l_angle);
					move_body_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_r_angle);
				}
				else{
					move_body_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_l_angle);
					move_body_set(MOVE_LEG_R, MOVE_SHOCK_4, leg_delay, leg_r_angle/Kleg);
				}
      }
    }break;
    
    case WORK_OVER:
    {
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}
void move_step_rf_task(void *param)  //83 Step rf
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t foot_delay=pt_move->foot_tim;
	float Kleg = 3;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);

      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }

  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
				if(last_move != 84)
				{
					move_body_set(MOVE_LEG_L, MOVE_SHOCK_1, leg_delay, leg_l_angle/Kleg);
					move_body_set(MOVE_LEG_R, MOVE_SHOCK_1, leg_delay, leg_r_angle);
				}
				else{
					move_body_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_r_angle);
					move_body_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay, foot_r_angle);
				}
			}
    }break;
    
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
				if(last_move != 84)
				{
					move_body_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_l_angle);
					move_body_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_l_angle);
				}
				else{
					move_body_set(MOVE_LEG_L, MOVE_SHOCK_2, leg_delay, leg_l_angle/Kleg);
					move_body_set(MOVE_LEG_R, MOVE_SHOCK_2, leg_delay, leg_r_angle);
				}
      }
    }break;
    
    case WORK_OVER:
    {
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}
void move_step_rb_task(void *param)  //84 Step rb
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t foot_delay=pt_move->foot_tim;
	float Kleg = 3;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);

      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }

  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
				if(last_move != 83)
				{
					move_body_set(MOVE_LEG_L, MOVE_SHOCK_1, leg_delay, leg_l_angle/Kleg);
					move_body_set(MOVE_LEG_R, MOVE_SHOCK_1, leg_delay, leg_r_angle);
				}
				else{
					move_body_set(MOVE_FOOT_L, MOVE_SHOCK_4, foot_delay, foot_l_angle);
					move_body_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_l_angle);
				}
			}
    }break;
    
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
				if(last_move != 83)
				{
					move_body_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_r_angle);
					move_body_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_r_angle);
				}
				else{
					move_body_set(MOVE_LEG_L, MOVE_SHOCK_2, leg_delay, leg_l_angle/Kleg);
					move_body_set(MOVE_LEG_R, MOVE_SHOCK_2, leg_delay, leg_r_angle);
				}
      }
    }break;
    
    case WORK_OVER:
    {
      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}


void move_step_l_task(void *param)  //85 87 89 Step l
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t foot_delay=pt_move->foot_tim;
	float Kleg = 3;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);

      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
				move_body_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_l_angle);
				move_body_set(MOVE_LEG_R, MOVE_SHOCK_4, leg_delay, leg_r_angle/Kleg);
			}
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }

  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
				move_body_set(MOVE_LEG_L, MOVE_SHOCK_3, leg_delay, leg_l_angle);
				move_body_set(MOVE_LEG_R, MOVE_SHOCK_3, leg_delay, leg_r_angle/Kleg);
			}
    }break;
    
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_l_angle);
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_l_angle);
				
				if (cur_move == 89)
					t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
      }
    }break;
    case WORK_FOOT5:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT6;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_l_angle);
      }
    }break;
    case WORK_FOOT6:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT7;
        t_move_step.u_last_step.e_work_step = WORK_FOOT6;
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_l_angle);
      }
    }break;
    case WORK_FOOT7:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT8;
        t_move_step.u_last_step.e_work_step = WORK_FOOT7;
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_l_angle);
      }
    }break;
    case WORK_FOOT8:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT8;
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_l_angle);
      }
    }break;
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_l_angle);
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay, foot_l_angle);
				
				if (cur_move == 85 || cur_move == 89)
					t_move_step.u_cur_step.e_work_step = WORK_OVER;
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_r_angle);
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_r_angle);
      }
    }break;
    case WORK_FOOT4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_4, foot_delay, foot_r_angle);
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_r_angle);
      }
    }break;
//    case WORK_LEG4:
//    {
//      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_OVER;
//        t_move_step.u_last_step.e_work_step = WORK_LEG4;
//				move_body_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_l_angle);
//				move_body_set(MOVE_LEG_R, MOVE_SHOCK_4, leg_delay, leg_r_angle/Kleg);
//			}
//    }break;
    
    case WORK_OVER:
    {
      t_move_step.u_cur_step.e_work_step = WORK_FOOT;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}

void move_step_r_task(void *param)  //86 88 90 Step r
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

  uint32_t leg_l_angle = t_move_step.leg_l_angle;
  uint32_t leg_r_angle = t_move_step.leg_r_angle;
	uint32_t foot_l_angle = t_move_step.foot_l_angle;
	uint32_t foot_r_angle = t_move_step.foot_r_angle;
  uint32_t leg_delay=pt_move->leg_tim;
  uint32_t foot_delay=pt_move->foot_tim;
	float Kleg = 3;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);

      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
				move_body_set(MOVE_LEG_L, MOVE_SHOCK_2, leg_delay, leg_l_angle/Kleg);
				move_body_set(MOVE_LEG_R, MOVE_SHOCK_2, leg_delay, leg_r_angle);
			}
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);
    }
    return;
  }

  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
     
    }break;
    case WORK_LEG:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
				move_body_set(MOVE_LEG_L, MOVE_SHOCK_1, leg_delay, leg_l_angle/Kleg);
				move_body_set(MOVE_LEG_R, MOVE_SHOCK_1, leg_delay, leg_r_angle);
			}
    }break;
    
    case WORK_FOOT:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_3, foot_delay, foot_l_angle);
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_l_angle);
				if (cur_move == 90)
					t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
      }
    }break;
    case WORK_FOOT5:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT6;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_l_angle);
      }
    }break;
    case WORK_FOOT6:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT7;
        t_move_step.u_last_step.e_work_step = WORK_FOOT6;
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_l_angle);
      }
    }break;
    case WORK_FOOT7:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT8;
        t_move_step.u_last_step.e_work_step = WORK_FOOT7;
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_l_angle);
      }
    }break;
    case WORK_FOOT8:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT8;
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_3, foot_delay, foot_l_angle);
      }
    }break;
    case WORK_FOOT2:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_4, foot_delay, foot_l_angle);
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_4, foot_delay, foot_l_angle);
				if (cur_move == 86 || cur_move == 90)
					t_move_step.u_cur_step.e_work_step = WORK_OVER;
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_1, foot_delay, foot_r_angle);
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_1, foot_delay, foot_r_angle);
      }
    }break;
    case WORK_FOOT4:
    {
      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_OVER;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
				move_body_set(MOVE_FOOT_L, MOVE_SHOCK_2, foot_delay, foot_r_angle);
				move_body_set(MOVE_FOOT_R, MOVE_SHOCK_2, foot_delay, foot_r_angle);
      }
    }break;
//    case WORK_LEG4:
//    {
//      if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
//      {
//        t_move_step.u_cur_step.e_work_step = WORK_OVER;
//        t_move_step.u_last_step.e_work_step = WORK_LEG4;
//				move_body_set(MOVE_LEG_L, MOVE_SHOCK_4, leg_delay, leg_l_angle/Kleg);
//				move_body_set(MOVE_LEG_R, MOVE_SHOCK_4, leg_delay, leg_r_angle);
//			}
//    }break;
    
    case WORK_OVER:
    {
      t_move_step.u_cur_step.e_work_step = WORK_FOOT;
      pt_move->step_count ++;
    }break;
    default : break; 
  } 

}


void move_fall_forward(void *param)   //120  falldown
{
	T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
	T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
	T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
	T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;


	uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	float Kleg = 3;	// ktmp
	float Kfoot = 2;
	T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;

	
	if(pt_move->step_count >= pt_move->step_num)
	{
		if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
		{
			move_stop_en();
			l81_tim_task_den(ID_MOVE);  
			move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl/Kleg); //left +
			move_step_en(MOVE_LEG_L);
			move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangl); //left +
			move_step_en(MOVE_LEG_R);
			printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set,pt_move->step_num);
		}
	return;
	}
	
	switch(t_move_step.u_cur_step.e_work_step)
	{
		case WORK_NONE:
		{
     
		}break;
		
		case WORK_LEG:
		{
			if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
			{
				t_move_step.u_cur_step.e_work_step = WORK_FOOT;
				t_move_step.u_last_step.e_work_step = WORK_LEG;
				move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangl/Kleg); //
				move_step_en(MOVE_LEG_L);
				
				move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangl); // 
				move_step_en(MOVE_LEG_R);
			}
		}break;
    
		case WORK_FOOT:
		{
			if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
			{
				t_move_step.u_cur_step.e_work_step = WORK_OVER;
				t_move_step.u_last_step.e_work_step = WORK_FOOT;

				move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl/Kleg); //
				move_step_en(MOVE_FOOT_L);

				move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim, foot_gangl); //
				move_step_en(MOVE_FOOT_R);
			}
		}break;
		    
		case WORK_OVER:
		{
		  t_move_step.u_cur_step.e_work_step = WORK_LEG;
		  pt_move->step_count ++;
		}break;
		default : break; 
	}
	
}
void move_fall_backward(void *param)   //121  falldown
{
	T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;
  

	uint32_t b_leg_gangl = 15;
	uint32_t foot_gangl = t_move_step.foot_l_angle;
	uint32_t foot_gangr = t_move_step.foot_r_angle;
	uint32_t leg_gangl = t_move_step.leg_l_angle;
	uint32_t leg_gangr =  20;   //t_move_step.leg_r_angle;

  static uint32_t tim_count = 0;
	uint32_t tim_delay = 1000;
	
	float Kp = 0;
	float Kp_l = 0;
	float Kleg = 3;
	float Kfoot = 2;
  
  T_MOVE_STEP_TYPDEF *pt_move = &t_move_step;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
     if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
    {
      move_stop_en();
			l81_tim_task_den(ID_MOVE);
    
      printf("AT+MOVEW,%d,%d\r\n",pt_move->e_act_set, pt_move->step_num);

    }
    return;
  }


  switch(t_move_step.u_cur_step.e_work_step)
  {
    case WORK_NONE:
    {
  static uint32_t tim_count = 0;
     tim_count += 10;
      if(tim_count == tim_delay)
      {
        t_move_step.u_cur_step.e_work_step = WORK_LEG7;
        tim_count = 0;
      }
     
    }break;
    case WORK_LEG:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT;
        t_move_step.u_last_step.e_work_step = WORK_LEG;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim/2, b_leg_gangl/Kleg ); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim/2, b_leg_gangl); //left +
        move_step_en(MOVE_LEG_R);
      }
      
    }break;
    case WORK_FOOT:
    {

      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
			{

        t_move_step.u_cur_step.e_work_step = WORK_LEG2;
        t_move_step.u_last_step.e_work_step = WORK_FOOT;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim/2, foot_gangl); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim/2, foot_gangl); //left +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    
    case WORK_LEG2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {

        t_move_step.u_cur_step.e_work_step = WORK_LEG3;
        t_move_step.u_last_step.e_work_step = WORK_LEG2;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim/2, b_leg_gangl/Kleg  ); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_2, pt_move->leg_tim/2, b_leg_gangl); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT2;
        t_move_step.u_last_step.e_work_step = WORK_LEG3;
        
        
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_3, pt_move->leg_tim/2, b_leg_gangl + yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim/2, b_leg_gangl/Kleg  + yaw_d*Kp_l); //left -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT2:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				yaw_d = yaw_start - yaw_last;
        t_move_step.u_cur_step.e_work_step = WORK_FOOT3;
        t_move_step.u_last_step.e_work_step = WORK_FOOT2;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_2, pt_move->leg_tim/2, foot_gangl - yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim/2, foot_gangl - yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_FOOT3:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {  
        
        t_move_step.u_cur_step.e_work_step = WORK_LEG4;
        t_move_step.u_last_step.e_work_step = WORK_FOOT3;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim/2, foot_gangl + yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_3, pt_move->leg_tim/2, foot_gangl + yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;
    case WORK_LEG4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
			
        t_move_step.u_cur_step.e_work_step = WORK_LEG5;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
				
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_4, pt_move->leg_tim/2, b_leg_gangl + yaw_d*Kp_l); //right -
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim/2, b_leg_gangl/Kleg  + yaw_d*Kp_l); //right -
        move_step_en(MOVE_LEG_R);
      }
    }break;
    case WORK_LEG5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT4;
        t_move_step.u_last_step.e_work_step = WORK_LEG4;
        move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim/2, b_leg_gangl/Kleg  - yaw_d*Kp_l); //left +
        move_step_en(MOVE_LEG_L);
        move_step_set(MOVE_LEG_R, MOVE_SHOCK_1, pt_move->leg_tim/2, b_leg_gangl - yaw_d*Kp_l); //left +
        move_step_en(MOVE_LEG_R);
      }
    }break;
    
    case WORK_FOOT4:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
        t_move_step.u_cur_step.e_work_step = WORK_FOOT5;
        t_move_step.u_last_step.e_work_step = WORK_FOOT4;
        
        
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim/2, foot_gangl + yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_4, pt_move->leg_tim/2, foot_gangl + yaw_d*Kp); //ritht -
        move_step_en(MOVE_FOOT_R);
      }
    }break;

    case WORK_FOOT5:
    {
      if(pt_foot_l->run_flag && pt_foot_r->run_flag && pt_leg_l->run_flag && pt_leg_r->run_flag)
      {
				if(pt_move->step_count == pt_move->step_num -1)
				{
					t_move_step.u_cur_step.e_work_step = WORK_LEG6;
					t_move_step.u_last_step.e_work_step = WORK_FOOT5;
					break;
				}
        
        t_move_step.u_cur_step.e_work_step = WORK_LEG6;
        t_move_step.u_last_step.e_work_step = WORK_FOOT5;
        move_step_set(MOVE_FOOT_L, MOVE_SHOCK_1, pt_move->leg_tim/2, foot_gangl - yaw_d*Kp); //left +
        move_step_en(MOVE_FOOT_L);
        move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim/2, foot_gangl - yaw_d*Kp); //ritht +
        move_step_en(MOVE_FOOT_R);
      }
    }break;
	case WORK_LEG6:
			{
				if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
				{
					t_move_step.u_cur_step.e_work_step = WORK_FOOT6;
					t_move_step.u_last_step.e_work_step = WORK_LEG6;
					move_step_set(MOVE_LEG_L, MOVE_SHOCK_1, pt_move->leg_tim, leg_gangr); //
					move_step_en(MOVE_LEG_L);
					
					move_step_set(MOVE_LEG_R, MOVE_SHOCK_3, pt_move->leg_tim, leg_gangr ); // 
					move_step_en(MOVE_LEG_R);
				}
			}break;
			
		case WORK_FOOT6:
		{
			if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
			{
				t_move_step.u_cur_step.e_work_step = WORK_NONE;
				t_move_step.u_last_step.e_work_step = WORK_FOOT6;

				move_step_set(MOVE_FOOT_L, MOVE_SHOCK_3, pt_move->leg_tim , foot_gangl*Kfoot ); //
				move_step_en(MOVE_FOOT_L);

				move_step_set(MOVE_FOOT_R, MOVE_SHOCK_1, pt_move->leg_tim, foot_gangl*Kfoot); //
				move_step_en(MOVE_FOOT_R);
			}
		}break;
	case WORK_LEG7:
			{
				if(pt_leg_l->run_flag && pt_leg_r->run_flag && pt_foot_l->run_flag && pt_foot_r->run_flag)
				{
					t_move_step.u_cur_step.e_work_step = WORK_OVER;
					t_move_step.u_last_step.e_work_step = WORK_LEG7;
					move_step_set(MOVE_LEG_L, MOVE_SHOCK_2, pt_move->leg_tim, leg_gangr); //
					move_step_en(MOVE_LEG_L);
					
					move_step_set(MOVE_LEG_R, MOVE_SHOCK_4, pt_move->leg_tim, leg_gangr ); // 
					move_step_en(MOVE_LEG_R);
					move_step_set(MOVE_FOOT_L, MOVE_SHOCK_4, pt_move->leg_tim*2 , foot_gangl*Kfoot ); //
					move_step_en(MOVE_FOOT_L);

					move_step_set(MOVE_FOOT_R, MOVE_SHOCK_2, pt_move->leg_tim*2, foot_gangl*Kfoot); //
					move_step_en(MOVE_FOOT_R);
				}
			}break;
    
    case WORK_OVER:
    {

      t_move_step.u_cur_step.e_work_step = WORK_LEG;
      pt_move->step_count ++;

    }break;
    default : break; 
  } 
	
}
int left_or_right = 0;
uint32_t angle_leg = 0;
uint32_t angle_leg_min = 0;
uint32_t angle_foot = 0;
uint32_t angle_foot_min = 0;
void action_set(E_MOVE_ACTION_TYPDEF e_action, int32_t step, int32_t delay)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_l   = &t_move.tLegL;
  T_MOVE_BODY_DATA_TYPDEF *pt_leg_r   = &t_move.tLegR;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_l  = &t_move.tFootL;
  T_MOVE_BODY_DATA_TYPDEF *pt_foot_r  = &t_move.tFootR;

  move_stop_en();
  l81_tim_task_den(ID_MOVE);
	
  last_move = cur_move;
	cur_move = e_action;
  
  pt_leg_l->run_flag = 1;
  pt_leg_r->run_flag = 1;
  pt_foot_l->run_flag = 1;
  pt_foot_r->run_flag = 1;
  
  t_move_step.u_cur_step.e_work_step = WORK_LEG;
  t_move_step.u_cur_step.e_step_num = STEP_1;
  
  t_move_step.step_num = step;
  t_move_step.step_count = 0;
  t_move_step.foot_tim = delay;
  t_move_step.leg_tim  = delay;
	
			yaw_start = 0;
			yaw_start = yaw_dat;
			yaw_now = yaw_start;
  switch(e_action)
  {		
		case ACT_105: //105
		{
			t_move_step.leg_l_angle = 23;
			t_move_step.leg_r_angle = 5;
			t_move_step.foot_l_angle = 34;
			t_move_step.foot_r_angle = 34;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_goforward_up2_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_106: //106 115
		{
			t_move_step.leg_l_angle = 17;
			t_move_step.leg_r_angle = 0;
			t_move_step.foot_l_angle = 35;
			t_move_step.foot_r_angle = 35;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_goforward_up2_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_107: //
		{
			t_move_step.leg_l_angle = 20;
			t_move_step.leg_r_angle = 5;
			t_move_step.foot_l_angle = 34;
			t_move_step.foot_r_angle = 34;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_goforward_up2_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
{
//		case ACT_108: //tiaoshide zhuanshen dongzuo
//		{
//			Kleg = 3;
//			t_move_step.leg_l_angle = 30;
//			t_move_step.leg_r_angle = 30;
//			t_move_step.foot_l_angle = 30;
//			t_move_step.foot_r_angle = 30;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_2_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_109: //
//		{
//			Kleg = 3;
//			t_move_step.leg_l_angle = 30;
//			t_move_step.leg_r_angle = 30;
//			t_move_step.foot_l_angle = 30;
//			t_move_step.foot_r_angle = 30;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_2_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_110: //
//		{
//			Kleg = 3;
//			t_move_step.leg_l_angle = 30;
//			t_move_step.leg_r_angle = 30;
//			t_move_step.foot_l_angle = 30;
//			t_move_step.foot_r_angle = 30;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_2_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_111: //
//		{
//			Kleg = 5;
//			t_move_step.leg_l_angle = 15;
//			t_move_step.leg_r_angle = 15;
//			t_move_step.foot_l_angle = 20;
//			t_move_step.foot_r_angle = 20;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_111_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_112: //
//		{
//			Kleg = 5;
//			t_move_step.leg_l_angle = 10;
//			t_move_step.leg_r_angle = 10;
//			t_move_step.foot_l_angle = 20;
//			t_move_step.foot_r_angle = 20;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_111_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_113: //
//		{
//			Kleg = 5;
//			t_move_step.leg_l_angle = 15;
//			t_move_step.leg_r_angle = 15;
//			t_move_step.foot_l_angle = 30;
//			t_move_step.foot_r_angle = 30;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_111_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_114: //
//		{
//			Kleg = 5;
//			t_move_step.leg_l_angle = 10;
//			t_move_step.leg_r_angle = 10;
//			t_move_step.foot_l_angle = 30;
//			t_move_step.foot_r_angle = 30;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_111_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_115: //
//		{
//			Kleg = 5;
//			t_move_step.leg_l_angle = 17;
//			t_move_step.leg_r_angle = 0;
//			t_move_step.foot_l_angle = 35;
//			t_move_step.foot_r_angle = 35;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_goforward_up2_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_116: //
//		{
//			Kleg = 5;
//			t_move_step.leg_l_angle = 17;
//			t_move_step.leg_r_angle = 0;
//			t_move_step.foot_l_angle = 30;
//			t_move_step.foot_r_angle = 30;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_112_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_117: //
//		{
//			Kleg = 5;
//			t_move_step.leg_l_angle = 17;
//			t_move_step.leg_r_angle = 0;
//			t_move_step.foot_l_angle = 25;
//			t_move_step.foot_r_angle = 25;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_112_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
}
		case ACT_108: //
		{
			t_move_step.leg_l_angle = 14;
			t_move_step.leg_r_angle = 14;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_right_task);
      l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_109: //
		{
			t_move_step.leg_l_angle = 13;
			t_move_step.leg_r_angle = 13;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_right_task);
      l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_110: //  zhuan shen
		{
//			t_move_step.leg_l_angle = 12;
//			t_move_step.leg_r_angle = 12;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_right_task);
//      l81_tim_task_en(ID_MOVE);
			
			Kleg = 3;
			t_move_step.leg_l_angle = 30;
			t_move_step.leg_r_angle = 30;
			t_move_step.foot_l_angle = 30;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_2_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_111: //
		{
			t_move_step.leg_l_angle = 11;
			t_move_step.leg_r_angle = 11;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_right_task);
      l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_112: //
		{
			t_move_step.leg_l_angle = 10;
			t_move_step.leg_r_angle = 10;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_right_task);
      l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_113: //
		{
			t_move_step.leg_l_angle = 9;
			t_move_step.leg_r_angle = 9;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_right_task);
      l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_114: //
		{
			t_move_step.leg_l_angle = 8;
			t_move_step.leg_r_angle = 8;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_right_task);
      l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_115: //
		{
			t_move_step.leg_l_angle = 7;
			t_move_step.leg_r_angle = 7;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_right_task);
      l81_tim_task_en(ID_MOVE);
//			Kleg = 5;
//			t_move_step.leg_l_angle = 17;
//			t_move_step.leg_r_angle = 0;
//			t_move_step.foot_l_angle = 35;
//			t_move_step.foot_r_angle = 35;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_goforward_up2_task);
//			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_116: //
		{
			t_move_step.leg_l_angle = 6;
			t_move_step.leg_r_angle = 6;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_right_task);
      l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_117: //
		{
			t_move_step.leg_l_angle = 5;
			t_move_step.leg_r_angle = 5;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_right_task);
      l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_118: //
		{
			Kleg = 5;
			t_move_step.leg_l_angle = 17;
			t_move_step.leg_r_angle = 0;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_112_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_119: //
		{
			Kleg = 5;
			t_move_step.leg_l_angle = angle_leg;
			t_move_step.leg_r_angle = angle_leg_min;
			t_move_step.foot_l_angle = angle_foot;
			t_move_step.foot_r_angle = angle_foot_min;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_112_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		
    case ACT_NONE:
    {
      move_stop_en();
      l81_tim_task_den(ID_MOVE);
      l18_move_zero();
    }
    break;
    case ACT_CRAB_L:
    {
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_left_task);
      l81_tim_task_en(ID_MOVE);
//      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_crab_left_task);
//      l81_tim_task_en(ID_MOVE);
    }
    break;
    case ACT_CRAB_R:
    {
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_right_task);
      l81_tim_task_en(ID_MOVE);
//      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_crab_right_task);
//      l81_tim_task_en(ID_MOVE);
    }
    break;
    
    case ATC_SHAKE_LEG_L:
    {
			t_move_step.leg_l_angle = 18;//25;
			t_move_step.leg_r_angle = 18;//25;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_leg_left_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    case ATC_SHAKE_LEG_R:
    {
			t_move_step.leg_l_angle = 18;//25;
			t_move_step.leg_r_angle = 18;//25;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_leg_right_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    
    case ATC_SHAKE_FOOT_L:
    {
//			t_move.tFootL.run_flag = 1;
//			t_move.tFootR.run_flag = 0;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 0;
			
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_task);
      //l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_left_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    case ATC_SHAKE_FOOT_R:
    {
//			t_move.tFootL.run_flag = 0;
//			t_move.tFootR.run_flag = 1;
			t_move_step.foot_l_angle = 0;
			t_move_step.foot_r_angle = 20;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_task);
      //l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_right_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    
    case ATC_CROSS_LEGS_L:
    {
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_cross_legs_left_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    case ATC_CROSS_LEGS_R:
    {
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_cross_legs_right_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    
    
    case ATC_LEAN_L:
    {
			t_move_step.leg_l_angle = 35;
			t_move_step.leg_r_angle = 10;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_lean_left_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    case ATC_LEAN_R:
    {
			t_move_step.leg_l_angle = 10;
			t_move_step.leg_r_angle = 35;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_lean_right_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    
    case ATC_STOMP_L:
    {
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_stomp_left_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    case ATC_STOMP_R:
    {
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_stomp_right_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    
    case ATC_SHAKE_BODY_UD:
    {
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.leg_tim  = 30;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_body_up_down_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    case ATC_SHAKE_BODY_LR:
    {
			t_move_step.foot_l_angle = 15;
			t_move_step.foot_r_angle = 15;
			t_move_step.leg_tim  = 60;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_body_left_right_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
//    case ATC_SHAKE_HEAD_LR:  //19
//    {
//			t_move_step.foot_l_angle = 12;
//			t_move_step.foot_r_angle = 12;
//      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_head_left_right_task);
//      l81_tim_task_en(ID_MOVE);
//    }
//    break;
    
    
    case ATC_STAND_EASE:
    {
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_stand_ease_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    
    case ACT_TURN_L:
    {
			t_move_step.foot_l_angle = 45;
			t_move_step.foot_r_angle = 45;
			l81_move_body_write(MOVE_LEG_L, MOVE_R, 10);
			l81_move_body_write(MOVE_LEG_R, MOVE_R, 5);
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    case ACT_TURN_R:
    {
			t_move_step.foot_l_angle = 45;
			t_move_step.foot_r_angle = 45;
			l81_move_body_write(MOVE_LEG_L, MOVE_L, 5);
			l81_move_body_write(MOVE_LEG_R, MOVE_L, 10);
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_right_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    
    case ACT_CLAMP_LEG:
    {
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_clamp_leg_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    case ACT_SHAKE_FOOT_TEMPO: //24
    {
			t_move_step.foot_l_angle = 15;
			t_move_step.foot_r_angle = 15;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_right_tempo_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
		case ACT_TURN_MINI_L:
		{
			t_move_step.foot_l_angle = 15;
			t_move_step.foot_r_angle = 15;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left_task);
			//l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left_mini_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_TURN_MINI_R:
		{
			t_move_step.foot_l_angle = 15;
			t_move_step.foot_r_angle = 15;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_right_task);
			//l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_right_mini_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_SWAY:
    {
			yaw_start = yaw_dat;
			yaw_now = yaw_dat;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_sway_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
    case ACT_RANDOM28: //28
    {
			t_move_step.foot_l_angle = 8;
			t_move_step.foot_r_angle = 8;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_head_left_right_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
//		case ACT_RANDOM29: //29
//		{
//			t_move_step.leg_l_angle = 12;
//			t_move_step.leg_r_angle = 12;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_incline_left_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_RANDOM30: //30
//		{
//			t_move_step.leg_l_angle = 12;
//			t_move_step.leg_r_angle = 12;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_incline_right_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_RANDOM31: //31
//		{
//			t_move_step.leg_l_angle = 18;
//			t_move_step.leg_r_angle = 18;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_incline_left_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_RANDOM32: //32
//		{
//			t_move_step.leg_l_angle = 18;
//			t_move_step.leg_r_angle = 18;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_incline_right_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
    
//    case ACT_RANDOM33: //33
//    {
////			t_move_step.foot_tim = 80;
////			t_move_step.leg_tim  = 80;
//			t_move_step.foot_l_angle = 10;
//			t_move_step.foot_r_angle = 10;
//			
//      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_task);
//      l81_tim_task_en(ID_MOVE);
//    }
//		break;
    case ACT_RANDOM34: //34
    {
			t_move_step.foot_l_angle = 8;
			t_move_step.foot_r_angle = 8;
			
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_in_half_task);
      l81_tim_task_en(ID_MOVE);
    }
		break;
    case ACT_RANDOM35: //35    //0720 rjq+
    {
//			//t_move_step.leg_tim  = 200;
//			t_move_step.foot_l_angle = 6;
//			t_move_step.foot_r_angle = 6;
//			t_move_step.leg_l_angle = 10;
//			t_move_step.leg_r_angle = 10;
//			
//      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_task);  //move_work_forward_task    
//      l81_tim_task_en(ID_MOVE);
			
			t_move_step.foot_l_angle = 12;
			t_move_step.foot_r_angle = 12;
			t_move_step.leg_l_angle = 10;
			t_move_step.leg_r_angle = 10;
			
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_gyro3_task);  //move_work_forward_task    
      l81_tim_task_en(ID_MOVE);
    }
		break;
		case ACT_RANDOM36: //36 
		{
			t_move_step.foot_l_angle = 10;
			t_move_step.foot_r_angle = 10;
			t_move_step.leg_l_angle = 8;
			t_move_step.leg_r_angle = 8;
			//l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_leg_left_task);
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_backward_gyro_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
//		case ACT_RANDOM37: //37 
//		{
//			t_move_step.leg_l_angle = 16;
//			t_move_step.leg_r_angle = 16;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_leg_right_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_RANDOM38: //38 
//		{
//			t_move_step.leg_l_angle = 12;
//			t_move_step.leg_r_angle = 12;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_leg_left_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_RANDOM39: //39 
//		{
//			t_move_step.leg_l_angle = 16;
//			t_move_step.leg_r_angle = 16;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_leg_right_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_RANDOM40: //40 
//		{
//			t_move_step.leg_l_angle = 8;
//			t_move_step.leg_r_angle = 8;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ACT_RANDOM41: //41 
//		{
//			t_move_step.foot_l_angle = 8;
//			t_move_step.foot_r_angle = 8;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_right_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
		case ACT_RANDOM42: //42
		{
			t_move_step.foot_l_angle = 8;
			t_move_step.foot_r_angle = 8;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_right_tempo_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ATC_SHAKE_HEAD_SMALL_ANGLE: //28 43
    {
			t_move_step.foot_tim = 15;
			t_move_step.leg_tim  = 15;
			t_move_step.foot_l_angle = 8;
			t_move_step.foot_r_angle = 8;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_head_left_right_task);
      l81_tim_task_en(ID_MOVE);
    }
    break;
		case ACT_INCLINE_L: //29 44
		{
			t_move_step.leg_l_angle = 16;
			t_move_step.leg_r_angle = 16;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_incline_left_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_INCLINE_R: //30 45
		{
			t_move_step.leg_l_angle = 16;
			t_move_step.leg_r_angle = 16;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_incline_right_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_INCLINE_SMALL_ANGLE_L: //31 46
		{
			t_move_step.leg_l_angle = 10;
			t_move_step.leg_r_angle = 10;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_incline_left_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_INCLINE_SMALL_ANGLE_R: //32 47
		{
			t_move_step.leg_l_angle = 10;
			t_move_step.leg_r_angle = 10;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_incline_right_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    
//    case ATC_SHAKE_FOOT_QUICK: //33 48
//    {
//			t_move_step.foot_tim = 15;
//			t_move_step.leg_tim  = 15;
//			t_move_step.foot_l_angle = 10;
//			t_move_step.foot_r_angle = 10;
//			
//      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_task);
//      l81_tim_task_en(ID_MOVE);
//    }
//		break;
    case ATC_SHAKE_FOOT_IN_HALF: //34 49
    {
			t_move_step.leg_tim  = 30;
			t_move_step.foot_l_angle = 8;
			t_move_step.foot_r_angle = 8;
			
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_in_half_task);
      l81_tim_task_en(ID_MOVE);
    }
		break;
    case ACT_TWIST: //35 50
    {
			//t_move_step.leg_tim  = 200;
			t_move_step.foot_l_angle = 6;
			t_move_step.foot_r_angle = 6;
			t_move_step.leg_l_angle = 10;
			t_move_step.leg_r_angle = 10;
			
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_task);
      l81_tim_task_en(ID_MOVE);
    }
		break;
		case ATC_SHAKE_FOOT_SMALL_ANGLE_L: //36 51
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_leg_left_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ATC_SHAKE_FOOT_SMALL_ANGLE_R: //37 52
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_leg_right_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
//		case ATC_MOVE_LEFT_OUT_HALF: //38 53
//		{
//			t_move_step.leg_l_angle = 15;
//			t_move_step.leg_r_angle = 0;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_leg_left_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
//		case ATC_MOVE_RIGHT_OUT_HALF: //39 54
//		{
//			t_move_step.leg_l_angle = 0;
//			t_move_step.leg_r_angle = 15;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_leg_right_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
		case ATC_TURN_LEFT_MINI_HALF: //40 55
		{
//			t_move_step.leg_l_angle = 10;
//			t_move_step.leg_r_angle = 10;
			t_move_step.foot_l_angle = 12;
			t_move_step.foot_r_angle = 12;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ATC_TURN_RIGHT_MINI_HALF: //41 56
		{
			t_move_step.foot_l_angle = 12;
			t_move_step.foot_r_angle = 12;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_right_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
//		case ATC_SHAKE_RIGHT_MINI_HALF: //42 57
//		{
//			t_move_step.foot_l_angle = 12;
//			t_move_step.foot_r_angle = 12;
//			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_right_tempo_task);
//			l81_tim_task_en(ID_MOVE);
//		}
//		break;
		case ACT_INCLINE_BIG_ANGLE_R: //30 58
		{
			t_move_step.leg_tim  = 1300;
			t_move_step.leg_l_angle = 25;
			t_move_step.leg_r_angle = 35;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_incline_right_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_CLAMP_LEG_SHAKE_FOOT_IN_LR: //59
		{
			t_move_step.leg_l_angle = 18;
			t_move_step.leg_r_angle = 18;
			t_move_step.foot_l_angle = 9;
			t_move_step.foot_r_angle = 9;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_clamp_leg_shake_foot_in_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_CLAMP_LEG_SHAKE_FOOT_LR: //60
		{
			t_move_step.leg_l_angle = 18;
			t_move_step.leg_r_angle = 18;
			t_move_step.foot_l_angle = 10;
			t_move_step.foot_r_angle = 10;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_clamp_leg_shake_foot_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_OPEN_LEG_L: //61  
		{
			t_move_step.leg_l_angle = 10;
			t_move_step.leg_r_angle = 10;
			t_move_step.foot_l_angle = 0;
			t_move_step.foot_r_angle = 0;
			t_move_step.leg_tim  = 50;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_open_leg_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_OPEN_LEG_R: //62
		{
			
			t_move_step.foot_tim = 20;
			t_move_step.leg_tim  = 20;
			t_move_step.leg_l_angle = 10; //15
			t_move_step.leg_r_angle = 10; //15
			t_move_step.foot_l_angle = 25;
			t_move_step.foot_r_angle = 25;
			
      //l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_open_leg_task);
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_open_leg_shake_task);
      l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_63: //63
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 23;
			t_move_step.foot_r_angle = 23;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_gyro3_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_64: //64
		{
			yaw_start = 0;
			yaw_start = yaw_dat;
			yaw_now = yaw_start;
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 23;
			t_move_step.foot_r_angle = 23;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_backward_gyro_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_65: //65
		{
			t_move_step.leg_l_angle = 0;
			t_move_step.leg_r_angle = 0;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 0; //5
			t_move_step.leg_tim  = 20;
			t_move_step.foot_tim  = 20;
      //l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_open_leg_task);
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_66: //66
		{
			t_move_step.leg_l_angle = 0;
			t_move_step.leg_r_angle = 0;
			t_move_step.foot_l_angle = 0; //5
			t_move_step.foot_r_angle = 20;
			t_move_step.leg_tim  = 20;
			t_move_step.foot_tim  = 20;
      //l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_open_leg_task);
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_67: //67  
		{
			t_move_step.leg_tim  = 20;
			t_move_step.leg_l_angle = 0;
			t_move_step.leg_r_angle = 0;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
			
      //l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_open_leg_task);
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_foot_task);
      l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_68: //68
		{
			t_move_step.leg_tim  = 30;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_body_left_right_task);
      l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_69: //69
		{
			Kleg = 5;
			t_move_step.leg_l_angle = 25;
			t_move_step.leg_r_angle = 25;
			t_move_step.foot_l_angle = 38;
			t_move_step.foot_r_angle = 38;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_goforward_up1_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_70: //70
		{
			Kleg = 5;
			t_move_step.leg_l_angle = 23;
			t_move_step.leg_r_angle = 23;
			t_move_step.foot_l_angle = 34;
			t_move_step.foot_r_angle = 34;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_goforward_up1_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_71: //71
		{
			t_move_step.leg_l_angle = 25;
			t_move_step.leg_r_angle = 25;
			t_move_step.foot_l_angle = 30;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left_up_foor_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_72: //72
		{
			t_move_step.leg_l_angle = 20;
			t_move_step.leg_r_angle = 20;
			t_move_step.foot_l_angle = 30;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_right_up_foor_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_73: //73
		{
			t_move_step.leg_l_angle = 30;
			t_move_step.leg_r_angle = 30;
			t_move_step.foot_l_angle = 30;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left2_up_foor_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    
		case ACT_74: //74  turn 2.1
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 12;
			t_move_step.foot_r_angle = 12;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left3_up_foor_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_75: //75  turn 2.2
		{
			t_move_step.leg_l_angle = 20;
			t_move_step.leg_r_angle = 20;
			t_move_step.foot_l_angle = 12;
			t_move_step.foot_r_angle = 12;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left4_up_foor_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_76: //76
		{
			t_move_step.foot_tim = 0;
			t_move_step.leg_tim  = delay;
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 12;
			t_move_step.foot_r_angle = 12;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_gesture_task1);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_77: //77
		{			
			t_move_step.foot_tim = 2000;
			t_move_step.leg_tim  = delay;
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_gesture_task1);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_78: //78
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 15;
			t_move_step.foot_r_angle = 15;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_excited_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_79: //79
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 15;
			t_move_step.foot_r_angle = 15;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_Rock_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_80: //80 dance
		{
			t_move_step.leg_l_angle = 30;
			t_move_step.leg_r_angle = 30;
			t_move_step.foot_l_angle = 30;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_dance_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_81: //81 Step lf
		{
			t_move_step.step_num = 1;
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 35;
			t_move_step.foot_r_angle = 35;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_step_lf_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_82: //82 Step lb
		{
			t_move_step.step_num = 1;
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 30;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_step_lb_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_83: //83 Step rf
		{
			t_move_step.step_num = 1;
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 35;
			t_move_step.foot_r_angle = 35;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_step_rf_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_84: //84 Step rb
		{
			t_move_step.step_num = 1;
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 30;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_step_rb_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_85: //85 Step l
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 35;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_step_l_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_86: //86 Step r
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 35;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_step_r_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_87: //87 Step l
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 35;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_step_l_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_88: //88 Step r
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 35;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_step_r_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_89: //89 Step l shake foor
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 35;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_step_l_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_90: //90 Step r shake foor
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 35;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_step_r_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_91: //91
		{
			yaw_start = 0;
			yaw_start = yaw_dat;
			yaw_now = yaw_start;
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 28;
			t_move_step.foot_r_angle = 28;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_gyro_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_92: //92
		{
			yaw_start = 0;
			yaw_start = yaw_dat;
			yaw_now = yaw_start;
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 28;
			t_move_step.foot_r_angle = 28;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_backward_gyro_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_93: //93 
		{
			t_move_step.foot_l_angle = 15;
			t_move_step.foot_r_angle = 15;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_shake_head_left_right_task);
      l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_94: //94
		{
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_gyro_task_left);
      l81_tim_task_en(ID_MOVE);
		}
		break;
    case ACT_95: //95
		{
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
      l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_backward_gyro_task_left);
      l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_96: //96  turn right
		{
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 12;
			t_move_step.foot_r_angle = 12;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_right2_up_foor_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_97: //97
		{
			t_move_step.leg_l_angle = 12;
			t_move_step.leg_r_angle = 12;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
			if(left_or_right++ % 2 == 0)
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_backward_r_task);
			else
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_backward_l_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_98: //98
		{
			//get_algo_avg();
			t_move_step.leg_l_angle = 15;
			t_move_step.leg_r_angle = 15;
			t_move_step.foot_l_angle = 23;
			t_move_step.foot_r_angle = 23;
			if(left_or_right++ % 2 == 0)
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_gyro3_task);
			else
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_gyro4_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_99: //99
		{
			t_move_step.leg_l_angle = 12;
			t_move_step.leg_r_angle = 12;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_gyro3_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_100: //100
		{
			t_move_step.leg_l_angle = 12;
			t_move_step.leg_r_angle = 12;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_forward_gyro4_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_101: //101
		{
			t_move_step.leg_l_angle = 12;
			t_move_step.leg_r_angle = 12;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_backward_r_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_102: //102
		{
			t_move_step.leg_l_angle = 12;
			t_move_step.leg_r_angle = 12;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_work_backward_l_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_103: //103
		{
			t_move_step.leg_l_angle = 12;
			t_move_step.leg_r_angle = 12;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_right_2_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_104: //104
		{
			t_move_step.leg_l_angle = 12;
			t_move_step.leg_r_angle = 12;
			t_move_step.foot_l_angle = 20;
			t_move_step.foot_r_angle = 20;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_turn_left_2_task);
			l81_tim_task_en(ID_MOVE);
		}
		break;
		case ACT_120 :
		{
			t_move_step.leg_l_angle = 30;
			t_move_step.leg_r_angle = 30;
			t_move_step.foot_l_angle = 30;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_fall_forward);
			l81_tim_task_en(ID_MOVE);
			
			
		}break;
		case ACT_121 :
		{
			t_move_step.leg_l_angle = 30;
			t_move_step.leg_r_angle = 30;
			t_move_step.foot_l_angle = 30;
			t_move_step.foot_r_angle = 30;
			l81_tim_task_creat(ID_MOVE, TIM_TASK_CYCLE_ALL, 10, NULL, move_fall_backward);
			l81_tim_task_en(ID_MOVE);
			
			
		}break;
    default :break;
  }
  
}


//#endif
int is_use_gyro_move(int move_num)
{
	if(move_num == 0)
		return 0;
	int use_move[30] = {63,64,97,98};
	for(int i=0;i<30;i++)
		if(move_num == use_move[i])
			return 1;
	
	return 0;
}
int is_shuld_turn(float start ,float now)
{
//		printf("start : %f\r\n" , start);
//		printf("now : %f\r\n" , now);
	if(start == now)
		return 0;
	if( start > now)
	{
		return (start - now)/10;
	}
	else{
		return (start - now)/10;
	}
	return 0;
}

E_MOVE_ACTION_SET_TYPDEF move_cmd_last = SET_ZERO;
float yaw_first_go = 0; 
float yaw_delt_go = 0;
float yaw_nows_go = 0;
int move_num_conn = 0;  //lianxushu
int turn_num_conn = 0;  //lianxushu
int max_turn_num = 2;
int donot_move_num = 0;
int is_use_gyro = 1;
int is_print_gyro = 0;
E_MOVE_ACTION_SET_TYPDEF check_move_cmd(E_MOVE_ACTION_SET_TYPDEF move_cmd)
{
//	if(is_in_turn == 1)
//	{
//			printf("is_in_turn == 1\r\n" );
//		if(move_cmd == 0)
//		{
//			is_in_turn = 0;
//			return move_cmd;
//		}
//		
//		printf("donot_move_num = %d\r\n" ,donot_move_num);
//		if(donot_move_num++ <= max_turn_num)
//			return 200;
//		else{
//			donot_move_num = 0;
//			is_in_turn = 0;
//			return move_cmd;
//		}
//	}
	if(move_cmd != move_cmd_last)
	{
		move_cmd_last = move_cmd;
		yaw_first_go = yaw_dat;
		move_num_conn = 0;
		turn_num_conn = 0;
		is_in_turn = 0;
		return move_cmd;
	}
	if(is_use_gyro_move(move_cmd) == 0)
		return move_cmd;
	E_MOVE_ACTION_SET_TYPDEF return_cmd = move_cmd;
	if(move_cmd == move_cmd_last)
	{
		move_num_conn ++;
		if(is_print_gyro)
		printf("move_num_conn : %d\r\n" , move_num_conn);
		float yaw_first_now = is_shuld_turn(yaw_first_go,yaw_dat);
		if(yaw_first_now == 0)
		{
//			printf("return_move_cmd : %d\r\n" , move_cmd);
			return move_cmd;
		}
		if(move_num_conn >= 3)
		{
			if(is_print_gyro)
			printf("move_num_conn : %d\r\n" , move_num_conn);
//			yaw_first_go = yaw_dat  //meici xiuzhenghou chongzhi chushijiaoduzhi
			if(yaw_first_now > 0)
			{ // 96 turn right
				move_num_conn = 0;
				l18_move_zero();
//				printf("turn right: %d\r\n" , ACT_103);
				action_set(ACT_103,yaw_first_now,2*100);
				is_in_turn = 1;
				return 200;
			}
			else if(yaw_first_now < 0)
			{ // 74 turn lift
				move_num_conn = 0;
				l18_move_zero();
//				printf("turn lift: %d\r\n" , ACT_104);
				action_set(ACT_104,-yaw_first_now,2*100);
				is_in_turn = 1;
				return 200;
			}
		}
	}
		
	return return_cmd;
}
uint8_t l81_AT_MOVE_W_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);
	
//	if (param_num != 3U){  //motor must has three paramters, (motor number, value type(angle or pulse), value)
//		printf("AT+RES,ACK\r\n");
//		printf("AT+RES,Err,wrong param num\r\n");
//		printf("AT+RES,end\r\n");
//		return 0U;
//	}
	if (param_num < 3U || param_num >7U){  //motor must has three paramters, (motor number, value type(angle or pulse), value)
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	uint32_t move_cmd = String2Int(param[0]);
	uint32_t move_step = String2Int(param[1]);
	uint32_t speed = String2Int(param[2]);
	
	if(param_num >= 4U)
	angle_leg = String2Int(param[3]);
	if(param_num >= 5U)
	angle_leg_min = String2Int(param[4]);
	if(param_num >= 6U)
	angle_foot = String2Int(param[5]);
	if(param_num >= 7U)
	angle_foot_min = String2Int(param[6]);
		
	if (move_cmd < 0U || move_cmd > 200U) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong move comd %d, move cmd is[1,100]\r\n", move_cmd);
		printf("AT+RES,end\r\n");
		return 0U;
	}
//	if (move_step < 1U) {
//		printf("AT+RES,ACK\r\n");
//		printf("AT+RES,Err,wrong value_kind %d, value_kind is[0,1]\r\n", value_kind);
//		printf("AT+RES,end\r\n");
//		return 0U;
//	}	
	if (speed < 1  || speed > 10) 
  {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong value %d, value is[1,10]\r\n", speed);
		printf("AT+RES,end\r\n");
		return 0U;
	}	
	
  t_move_step.e_act_set = (E_MOVE_ACTION_SET_TYPDEF)move_cmd;
	
	if(move_cmd == 191)
	{
		is_use_gyro = 1-is_use_gyro;
		printf("set is_use_gyro : %d \r\n",is_use_gyro);
	}
	if(move_cmd == 192)
	{
		is_print_gyro = 1-is_print_gyro;
		printf("set is_print_gyro : %d \r\n",is_print_gyro);
	}
	
	E_MOVE_ACTION_SET_TYPDEF set_cmd = (E_MOVE_ACTION_SET_TYPDEF)move_cmd;
	if(is_use_gyro)
	{
		set_cmd = check_move_cmd((E_MOVE_ACTION_SET_TYPDEF)move_cmd);
		if(is_print_gyro)
		printf("set_cmd : %d\r\n" , set_cmd);
	}
//  switch(move_cmd)
  switch(set_cmd)
  {
    case 0: //
    {
      l18_move_zero();
      move_set(WORK_STOP,move_step,speed*100);
      printf("AT+MOVEW,%d,%d\r\n",t_move_step.e_act_set, t_move_step.step_num);
    }
    break;
    case 1: //forward
    {
      l18_move_zero();
      move_set(WORK_DIR_F,move_step,speed*100);
    }break;
    case 2: //backward
    {
      l18_move_zero();
      move_set(WORK_DIR_B,move_step,speed*100);
    }break;
    case 3: //left
    {
      l18_move_zero();
      move_set(WORK_DIR_L,move_step,speed*100);
    }break;
    case 4: //right
    {
      l18_move_zero();
      move_set(WORK_DIR_R,move_step,speed*100);
    }break;
    
    case 5: //ACT_CRAB_L param(step=3,delay=3)
    {
      l18_move_zero();
      action_set(ACT_CRAB_L, move_step, speed*100);
    }break;
    case 6: //ACT_CRAB_R
    {
      l18_move_zero();
      action_set(ACT_CRAB_R, move_step, speed*100);
//      l81_move_calibration_stor();
    }break;
    
    case 7: //ATC_SHAKE_LEG_L
    {
      l18_move_zero();
      action_set(ATC_SHAKE_LEG_L, move_step, speed*100);
    }break;
    case 8: //ATC_SHAKE_LEG_R
    {
      l18_move_zero();
      action_set(ATC_SHAKE_LEG_R, move_step, speed*100);
    }break;
  
    case 9: //ATC_SHAKE_FOOT_L
    {
      l18_move_zero();
      action_set(ATC_SHAKE_FOOT_L, move_step, speed*100);
    }break;
    case 10: //ATC_SHAKE_FOOT_R
    {
      l18_move_zero();
      action_set(ATC_SHAKE_FOOT_R, move_step, speed*100);
    }break;
    
    
    case 11: //ATC_CROSS_LEGS_L
    {
      l18_move_zero();
      action_set(ATC_CROSS_LEGS_L, move_step, speed*100);
    }break;
    case 12: //ATC_CROSS_LEGS_R
    {
      l18_move_zero();
      action_set(ATC_CROSS_LEGS_R, move_step, speed*100);
    }break;
    
    case 13: //ATC_LEAN_L
    {
      l18_move_zero();
      action_set(ATC_LEAN_L, move_step, speed*100);
    }break;
    case 14: //ATC_LEAN_R
    {
      l18_move_zero();
      action_set(ATC_LEAN_R, move_step, speed*100);
    }break;
    
    case 15: //ATC_STOMP_L
    {
      l18_move_zero();
      action_set(ATC_STOMP_L, move_step, speed*100);
    }break;
    case 16: //ATC_STOMP_R
    {
      l18_move_zero();
      action_set(ATC_STOMP_R, move_step, speed*100);
    }break;
    
    
    case 17: //ATC_SHAKE_BODY_UD
    {
      l18_move_zero();
      action_set(ATC_SHAKE_BODY_UD, move_step, speed*100);
    }break;
    case 18: //ATC_SHAKE_BODY_LR
    {
			if(move_step < 3) move_step = 3;
      l18_move_zero();
      action_set(ATC_SHAKE_BODY_LR, move_step, speed*100);
    }break;
    case 19: //ATC_SHAKE_HEAD_LR to 28  rjq-0508
    {
      l18_move_zero();
      action_set(ACT_RANDOM28, move_step, speed*100);
      //action_set(ATC_SHAKE_HEAD_LR, move_step, speed*100);
    }break;
    
    
    case 20: //ATC_STAND_EASE
    {
      l18_move_zero();
      action_set(ATC_STAND_EASE, move_step, speed*100);
    }break;
    
    case 21: //ACT_TURN_L
    {
      l18_move_zero();
      action_set(ACT_TURN_L, move_step, speed*100);
    }break;
    case 22: //ACT_TURN_R
    {
      l18_move_zero();
      action_set(ACT_TURN_R, move_step, speed*100);
    }break;
    
    case 23: //ACT_CLAMP_LEG
    {
      l18_move_zero();
      action_set(ACT_CLAMP_LEG, move_step, speed*100);
    }break;
    case 24: //ACT_SHAKE_FOOT_TEMPO
    {
      l18_move_zero();
      action_set(ACT_SHAKE_FOOT_TEMPO, move_step, speed*100);
    }break;
    case 25: //ACT_TURN_MINI_L
    {
      l18_move_zero();
      action_set(ACT_TURN_MINI_L, move_step, speed*100);
    }break;
    case 26: //ACT_TURN_MINI_R
    {
      l18_move_zero();
      action_set(ACT_TURN_MINI_R, move_step, speed*100);
    }break;
    case 27: //ACT_SWAY
    {
      l18_move_zero();
      action_set(ACT_SWAY, move_step, speed*100);
    }break;
    case 28: //ATC_SHAKE_HEAD_SMALL_ANGLE
    {
      l18_move_zero();
      action_set(ACT_RANDOM28, move_step, speed*100);
    }break;
    case 29: //ACT_INCLINE_L to 46  rjq-0508
    {
      l18_move_zero();
      action_set(ACT_INCLINE_SMALL_ANGLE_L, move_step, speed*100);
      //action_set(ACT_RANDOM29, move_step, speed*100);
    }break;
    case 30: //ACT_INCLINE_R to 47  rjq-0508
    {
      l18_move_zero();
      action_set(ACT_INCLINE_SMALL_ANGLE_R, move_step, speed*100);
      //action_set(ACT_RANDOM30, move_step, speed*100);
    }break;
    case 31: //ACT_INCLINE_SMALL_ANGLE_L to 44  rjq-0508
    {
      l18_move_zero();
      action_set(ACT_INCLINE_L, move_step, speed*100);
      //action_set(ACT_RANDOM31, move_step, speed*100);
    }break;
    case 32: //ACT_INCLINE_SMALL_ANGLE_R to 45  rjq-0508
    {
      l18_move_zero();
      action_set(ACT_INCLINE_R, move_step, speed*100);
      //action_set(ACT_RANDOM32, move_step, speed*100);
    }break;
    case 33: //ATC_SHAKE_FOOT_QUICK  to 34  rjq-0508
    {
			if(move_step < 3) move_step = 3;
      l18_move_zero();
      action_set(ACT_RANDOM34, move_step, speed*100);
      //action_set(ACT_RANDOM33, move_step, speed*100);
    }break;
    case 34: //ATC_SHAKE_FOOT_OUT_HALF
    {
			if(move_step < 3) move_step = 3;
      l18_move_zero();
      action_set(ACT_RANDOM34, move_step, speed*100);
    }break;
    case 35: //ACT_TWIST  to 50  rjq-0508
    {
      l18_move_zero();
      //action_set(ACT_TWIST, move_step, speed*100);
      action_set(ACT_RANDOM35, move_step, speed*100);     //0720  rjq+
    }break;
    case 36: //ATC_SHAKE_FOOT_SMALL_ANGLE_L  to 51  rjq-0508
    {
      l18_move_zero();
      //action_set(ATC_SHAKE_FOOT_SMALL_ANGLE_L, move_step, speed*100);
      action_set(ACT_RANDOM36, move_step, speed*100);  // reback to 36  rjq-0728
    }break;
    case 37: //ATC_SHAKE_FOOT_SMALL_ANGLE_R  to 52  rjq-0508
    {
      l18_move_zero();
      action_set(ATC_SHAKE_FOOT_SMALL_ANGLE_R, move_step, speed*100);
      //action_set(ACT_RANDOM37, move_step, speed*100);
    }break;
    case 38: //ATC_MOVE_LEFT_OUT_HALF  to 51  rjq-0508
    {
      l18_move_zero();
      action_set(ATC_SHAKE_FOOT_SMALL_ANGLE_L, move_step, speed*100);
      //action_set(ACT_RANDOM38, move_step, speed*100);
    }break;
    case 39: //ATC_MOVE_RIGHT_OUT_HALF  to 52  rjq-0508
    {
      l18_move_zero();
      action_set(ATC_SHAKE_FOOT_SMALL_ANGLE_R, move_step, speed*100);
      //action_set(ACT_RANDOM39, move_step, speed*100);
    }break;
    case 40: //ATC_TURN_LEFT_MINI_HALF  to 55  rjq-0508
    {
      l18_move_zero();
      action_set(ATC_TURN_LEFT_MINI_HALF, move_step, speed*100);
      //action_set(ACT_RANDOM40, move_step, speed*100);
    }break;
    case 41: //ATC_TURN_RIGHT_MINI_HALF  to 56  rjq-0508
    {
      l18_move_zero();
      action_set(ATC_TURN_RIGHT_MINI_HALF, move_step, speed*100);
      //action_set(ACT_RANDOM41, move_step, speed*100);
    }break;
    case 42: //ATC_SHAKE_RIGHT_MINI_HALF
    {
      l18_move_zero();
      action_set(ACT_RANDOM42, move_step, speed*100);
    }break;
    case 43: //ATC_SHAKE_HEAD_SMALL_ANGLE
    {
      l18_move_zero();
      action_set(ATC_SHAKE_HEAD_SMALL_ANGLE, move_step, speed*100);
    }break;
    case 44: //ACT_INCLINE_L
    {
      l18_move_zero();
      action_set(ACT_INCLINE_L, move_step, speed*100);
    }break;
    case 45: //ACT_INCLINE_R
    {
      l18_move_zero();
      action_set(ACT_INCLINE_R, move_step, speed*100);
    }break;
    case 46: //ACT_INCLINE_SMALL_ANGLE_L
    {
      l18_move_zero();
      action_set(ACT_INCLINE_SMALL_ANGLE_L, move_step, speed*100);
    }break;
    case 47: //ACT_INCLINE_SMALL_ANGLE_R
    {
      l18_move_zero();
      action_set(ACT_INCLINE_SMALL_ANGLE_R, move_step, speed*100);
    }break;
    case 48: //ATC_SHAKE_FOOT_QUICK  to 49  rjq-0508
    {
      l18_move_zero();
      action_set(ATC_SHAKE_FOOT_IN_HALF, move_step, speed*100);
      //action_set(ATC_SHAKE_FOOT_QUICK, move_step, speed*100);
    }break;
    case 49: //ATC_SHAKE_FOOT_OUT_HALF
    {
      l18_move_zero();
      action_set(ATC_SHAKE_FOOT_IN_HALF, move_step, speed*100);
    }break;
    case 50: //ACT_TWIST
    {
      l18_move_zero();
      action_set(ACT_TWIST, move_step, speed*100);
    }break;
    case 51: //ATC_SHAKE_FOOT_SMALL_ANGLE_L
    {
      l18_move_zero();
      action_set(ATC_SHAKE_FOOT_SMALL_ANGLE_L, move_step, speed*100);
    }break;
    case 52: //ATC_SHAKE_FOOT_SMALL_ANGLE_R
    {
      l18_move_zero();
      action_set(ATC_SHAKE_FOOT_SMALL_ANGLE_R, move_step, speed*100);
    }break;
    case 53: //ATC_MOVE_LEFT_OUT_HALF  to 51  rjq-0508
    {
      l18_move_zero();
      action_set(ATC_SHAKE_FOOT_SMALL_ANGLE_L, move_step, speed*100);
      //action_set(ATC_MOVE_LEFT_OUT_HALF, move_step, speed*100);
    }break;
    case 54: //ATC_MOVE_RIGHT_OUT_HALF  to 52  rjq-0508
    {
      l18_move_zero();
      action_set(ATC_SHAKE_FOOT_SMALL_ANGLE_R, move_step, speed*100);
      //action_set(ATC_MOVE_RIGHT_OUT_HALF, move_step, speed*100);
    }break;
    case 55: //ATC_TURN_LEFT_MINI_HALF
    {
      l18_move_zero();
      action_set(ATC_TURN_LEFT_MINI_HALF, move_step, speed*100);
    }break;
    case 56: //ATC_TURN_RIGHT_MINI_HALF
    {
      l18_move_zero();
      action_set(ATC_TURN_RIGHT_MINI_HALF, move_step, speed*100);
    }break;
    case 57: //ATC_SHAKE_RIGHT_MINI_HALF  to 42  rjq-0508
    {
      l18_move_zero();
      action_set(ACT_RANDOM42, move_step, speed*100);
      //action_set(ATC_SHAKE_RIGHT_MINI_HALF, move_step, speed*100);
    }break;
    case 58: //ACT_INCLINE_BIG_ANGLE_R
    {
      l18_move_zero();
      action_set(ACT_INCLINE_BIG_ANGLE_R, move_step, speed*100);
    }break;
    case 59: //ACT_CLAMP_LEG_SHAKE_FOOT_IN_LR
    {
      l18_move_zero();
      action_set(ACT_CLAMP_LEG_SHAKE_FOOT_IN_LR, move_step, speed*100);
    }break;
    case 60: //ACT_CLAMP_LEG_SHAKE_FOOT_LR
    {
      l18_move_zero();
      action_set(ACT_CLAMP_LEG_SHAKE_FOOT_LR, move_step, speed*100);
    }break;
    case 61: //ACT_61
    {
      l18_move_zero();
      action_set(ACT_OPEN_LEG_L, move_step, speed*100);
    }break;
    case 62: //ACT_62
    {
      l18_move_zero();
      action_set(ACT_OPEN_LEG_R, move_step, speed*100);
    }break;
    case 63: //ACT_63
    {
      l18_move_zero();
      action_set(ACT_63, move_step, speed*100);
    }break;
    case 64: //ACT_64
    {
      l18_move_zero();
      action_set(ACT_64, move_step, speed*100);
    }break;
    case 65: //ACT_65
    {
			if(move_step < 3) move_step = 3;
      l18_move_zero();
      action_set(ACT_65, move_step, speed*100);
    }break;
    case 66: //ACT_66
    {
			if(move_step < 3) move_step = 3;
      l18_move_zero();
      action_set(ACT_66, move_step, speed*100);
    }break;
    case 67: //ACT_67
    {
      l18_move_zero();
      action_set(ACT_67, move_step, speed*100);
    }break;
    case 68: //ACT_68
    {
      l18_move_zero();
      action_set(ACT_68, move_step, speed*100);
    }break;
    case 69: //ACT_69
    {
      l18_move_zero();
      action_set(ACT_69, move_step, speed*100);
    }break;
    case 70: //ACT_70
    {
      l18_move_zero();
      action_set(ACT_70, move_step, speed*100);
    }break;
    case 71: //ACT_71
    {
      l18_move_zero();
      action_set(ACT_71, move_step, speed*100);
    }break;
    case 72: //ACT_72
    {
      l18_move_zero();
      action_set(ACT_72, move_step, speed*100);
    }break;
    case 73: //ACT_73
    {
      l18_move_zero();
      action_set(ACT_73, move_step, speed*100);
    }break;
    case 74: //ACT_74
    {
      l18_move_zero();
      action_set(ACT_74, move_step, speed*100);
    }break;
    case 75: //ACT_75
    {
      l18_move_zero();
      action_set(ACT_75, move_step, speed*100);
    }break;
    case 76: //ACT_76
    {
      l18_move_zero();
      action_set(ACT_76, move_step, speed*100);
    }break;
    case 77: //ACT_77
    {
      l18_move_zero();
      action_set(ACT_77, move_step, speed*100);
    }break;
    case 78: //ACT_78
    {
      l18_move_zero();
      action_set(ACT_78, move_step, speed*100);
    }break;
    case 79: //ACT_79
    {
      l18_move_zero();
      action_set(ACT_79, move_step, speed*100);
    }break;
    case 80: //ACT_80 dance
    {
      l18_move_zero();
      action_set(ACT_80, move_step, speed*100);
    }break;
    case 81: //ACT_81
    {
			if(last_move != 82)
				l18_move_zero();
      action_set(ACT_81, 1, speed*100);
    }break;
    case 82: //ACT_82
    {
			if(last_move != 81)
				l18_move_zero();
      action_set(ACT_82, 1, speed*100);
    }break;
    case 83: //ACT_83
    {
			if(last_move != 84)
				l18_move_zero();
      action_set(ACT_83, 1, speed*100);
    }break;
    case 84: //ACT_84
    {
			if(last_move != 83)
				l18_move_zero();
      action_set(ACT_84, 1, speed*100);
    }break;
    case 85: //ACT_85
    {
			l18_move_zero();
      action_set(ACT_85, move_step, speed*100);
    }break;
    case 86: //ACT_86
    {
			l18_move_zero();
      action_set(ACT_86, move_step, speed*100);
    }break;
    case 87: //ACT_87
    {
			l18_move_zero();
      action_set(ACT_87, move_step, speed*100);
    }break;
    case 88: //ACT_88
    {
			l18_move_zero();
      action_set(ACT_88, move_step, speed*100);
    }break;
    case 89: //ACT_89
    {
			l18_move_zero();
      action_set(ACT_89, move_step, speed*100);
    }break;
    case 90: //ACT_90
    {
			l18_move_zero();
      action_set(ACT_90, move_step, speed*100);
    }break;
    case 91: //ACT_91
    {
			l18_move_zero();
      action_set(ACT_91, move_step, speed*100);
    }break;
    case 92: //ACT_92
    {
			l18_move_zero();
      action_set(ACT_92, move_step, speed*100);
    }break;
    case 93: //ACT_92
    {
			l18_move_zero();
      action_set(ACT_93, move_step, speed*100);
    }break;
    case 94: //ACT_92
    {
			l18_move_zero();
      action_set(ACT_94, move_step, speed*100);
    }break;
    case 95: //ACT_95
    {
			l18_move_zero();
      action_set(ACT_95, move_step, speed*100);
    }break;
    case 96: //ACT_96
    {
			l18_move_zero();
      action_set(ACT_96, move_step, speed*100);
    }break;
    case 97: //ACT_97
    {
			l18_move_zero();
      action_set(ACT_97, move_step, speed*100);
    }break;
    case 98: //ACT_98
    {
			l18_move_zero();
      action_set(ACT_98, move_step, speed*100);
    }break;
    case 99: //ACT_99
    {
			l18_move_zero();
      action_set(ACT_99, move_step, speed*100);
    }break;
    case 100: //ACT_100
    {
			l18_move_zero();
      action_set(ACT_100, move_step, speed*100);
    }break;
    case 101: //ACT_101
    {
			l18_move_zero();
      action_set(ACT_101, move_step, speed*100);
    }break;
    case 102: //ACT_102
    {
			l18_move_zero();
      action_set(ACT_102, move_step, speed*100);
    }break;
    case 103: //ACT_103
    {
			l18_move_zero();
      action_set(ACT_103, move_step, speed*100);
    }break;
    case 104: //ACT_104
    {
			l18_move_zero();
      action_set(ACT_104, move_step, speed*100);
    }break;
    case 105: //ACT_105
    {
			l18_move_zero();
      action_set(ACT_105, move_step, speed*100);
    }break;
    case 106: //ACT_106
    {
			l18_move_zero();
      action_set(ACT_106, move_step, speed*100);
    }break;
    case 107: //ACT_107
    {
			l18_move_zero();
      action_set(ACT_107, move_step, speed*100);
    }break;
    case 108: //ACT_108
    {
			l18_move_zero();
      action_set(ACT_108, move_step, speed*100);
    }break;
    case 109: //ACT_109
    {
			l18_move_zero();
      action_set(ACT_109, move_step, speed*100);
    }break;
    case 110: //ACT_110
    {
			l18_move_zero();
      action_set(ACT_110, move_step, speed*100);
    }break;
    case 111: //ACT_111
    {
			l18_move_zero();
      action_set(ACT_111, move_step, speed*100);
    }break;
    case 112: //ACT_112
    {
			l18_move_zero();
      action_set(ACT_112, move_step, speed*100);
    }break;
    case 113: //ACT_113
    {
			l18_move_zero();
      action_set(ACT_113, move_step, speed*100);
    }break;
    case 114: //ACT_114
    {
			l18_move_zero();
      action_set(ACT_114, move_step, speed*100);
    }break;
    case 115: //ACT_115
    {
			l18_move_zero();
      action_set(ACT_105, move_step, speed*100);
    }break;
    case 116: //ACT_116
    {
			l18_move_zero();
      action_set(ACT_116, move_step, speed*100);
    }break;
    case 117: //ACT_117
    {
			l18_move_zero();
      action_set(ACT_117, move_step, speed*100);
    }break;
    case 118: //ACT_118
    {
			l18_move_zero();
      action_set(ACT_118, move_step, speed*100);
    }break;
    case 119: //ACT_119
    {
			l18_move_zero();
      action_set(ACT_119, move_step, speed*100);
    }break;
		
		case 120: //ACT_120
    {
			l18_move_zero();
      action_set(ACT_120, move_step, speed*100);
    }break;
		case 121: //ACT_121
    {
			l18_move_zero();
      action_set(ACT_121, move_step, speed*100);
    }break;
		
    default:break;
  }
  
  printf("AT+RES,ACK\r\n");
  printf("AT+RES,end\r\n");
  

  return 1U;
}


uint8_t is_moving()
{
	
//	printf("t_move.run_flag: %d,%d,%d,%d",t_move.tLegL.run_flag , t_move.tLegR.run_flag , t_move.tFootL.run_flag , t_move.tFootR.run_flag);
	if(t_move.tLegL.run_flag && t_move.tLegR.run_flag && t_move.tFootL.run_flag && t_move.tFootR.run_flag)
		return 0;
	return 1;
}
/*-body stop-************/





/*-ear start-************/
void move_ear_zero(void)
{
  l81_move_body_write(MOVE_FOOT_L, MOVE_L, 0);
  l81_move_body_write(MOVE_EAR_R, MOVE_L, 0);
}

void move_ear_step_set(E_MOVE_BODY_TYPDEF e_body, E_MOVE_SHOCK_TYPDEF e_shock, uint32_t delay, uint32_t angle)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_l   = &t_move.tEarL;
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_r   = &t_move.tEarR;

  
  switch(e_body)
  {
    case MOVE_EAR_L  : 
    {
      pt_ear_l->e_shock = e_shock;
      pt_ear_l->delay_value  = delay;
      pt_ear_l->target_angle = angle;
      pt_ear_l->cycle_num  = pt_ear_l->delay_value/MOVE_TIME_SAMPE;
      pt_ear_l->cycle_count = 0;
      pt_ear_l->tim_count  = 0;
      
    }break;
    case MOVE_EAR_R  : 
    {
      pt_ear_r->e_shock = e_shock;
      pt_ear_r->delay_value  = delay;
      pt_ear_r->target_angle = angle;
      pt_ear_r->cycle_num  = pt_ear_l->delay_value/MOVE_TIME_SAMPE;
      pt_ear_r->cycle_count = 0;
      pt_ear_r->tim_count  = 0;
    }break;
    
    default :break;
  }
}




void move_ear_step_en(E_MOVE_BODY_TYPDEF e_body)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_l   = &t_move.tEarL;
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_r   = &t_move.tEarR;
  switch(e_body)
  {
    case MOVE_EAR_L  : 
    {
      pt_ear_l->stop = MOVE_OFF;
      pt_ear_l->run_flag = MOVE_OFF;
    }break;
    case MOVE_EAR_R  : 
    {
      pt_ear_r->stop = MOVE_OFF;
      pt_ear_r->run_flag = MOVE_OFF;
    }break;

    default :break;
  }
  
}

void move_ear_left_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_l   = &t_move.tEarL;
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_r   = &t_move.tEarR;

  uint32_t angle = 90;
  
  T_MOVE_EAR_TYPDEF *pt_move = &t_move_ear_step;
  angle = pt_move->ear_l_angle;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    move_refesh_ear_den();
    l81_tim_task_den(ID_EAR);
    move_ear_zero();
    
    printf("AT+EARW,%d,%d\r\n",pt_move->e_ear_set, pt_move->step_num);
    MOVE_EAR_L_PWM_OFF();
    MOVE_EAR_R_PWM_OFF();
    return;
  }


  switch(pt_move->e_cur_step)
  {
    case EAR_STEP_0:
    {
    }
    break;
    case EAR_STEP_1:
    {
      if(pt_ear_l->run_flag)
      {
        move_ear_step_set(MOVE_EAR_L, MOVE_SHOCK_1, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_L);
      }
      if(pt_ear_r->run_flag)
      {
        pt_move->e_cur_step = EAR_STEP_2;
        pt_move->e_last_step = EAR_STEP_1;
        
        move_ear_step_set(MOVE_EAR_R, MOVE_SHOCK_1, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_R);
      }
    }
    break;
    case EAR_STEP_2:
    {
      if(pt_ear_l->run_flag)
      {
        move_ear_step_set(MOVE_EAR_L, MOVE_SHOCK_2, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_L);
      }
      if(pt_ear_r->run_flag)
      {
        pt_move->e_cur_step = EAR_OVER;
        pt_move->e_last_step = EAR_STEP_2;
        
        move_ear_step_set(MOVE_EAR_R, MOVE_SHOCK_2, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_R);
      }
    }
    break;
    case EAR_STEP_3:
    {
    }
    break;
    case EAR_STEP_4:
    {
    }
    break;
    case EAR_OVER:
    {
      if((pt_ear_l->run_flag)&&(pt_ear_r->run_flag))
      {
        pt_move->e_cur_step = EAR_STEP_1;
        pt_move->step_count ++;
      }
      
    }
    break;
    
    default :break;
  }
  
}
void move_ear_right_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_l   = &t_move.tEarL;
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_r   = &t_move.tEarR;

  uint32_t angle = 90;
  
  T_MOVE_EAR_TYPDEF *pt_move = &t_move_ear_step;
  angle = pt_move->ear_l_angle;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    move_refesh_ear_den();
    l81_tim_task_den(ID_EAR);
    move_ear_zero();
    
    printf("AT+EARW,%d,%d\r\n",pt_move->e_ear_set, pt_move->step_num);
    MOVE_EAR_L_PWM_OFF();
    MOVE_EAR_R_PWM_OFF();
    return;
  }


  switch(pt_move->e_cur_step)
  {
    case EAR_STEP_0:
    {
    }
    break;
    case EAR_STEP_1:
    {
      if(pt_ear_l->run_flag)
      {
        move_ear_step_set(MOVE_EAR_L, MOVE_SHOCK_3, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_L);
      }
      if(pt_ear_r->run_flag)
      {
        pt_move->e_cur_step = EAR_STEP_2;
        pt_move->e_last_step = EAR_STEP_1;
        
        move_ear_step_set(MOVE_EAR_R, MOVE_SHOCK_3, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_R);
      }
    }
    break;
    case EAR_STEP_2:
    {
      if(pt_ear_l->run_flag)
      {
        move_ear_step_set(MOVE_EAR_L, MOVE_SHOCK_4, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_L);
      }
      if(pt_ear_r->run_flag)
      {
        pt_move->e_cur_step = EAR_OVER;
        pt_move->e_last_step = EAR_STEP_2;
        
        move_ear_step_set(MOVE_EAR_R, MOVE_SHOCK_4, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_R);
      }
    }
    break;
    case EAR_STEP_3:
    {
    }
    break;
    case EAR_STEP_4:
    {
    }
    break;
    case EAR_OVER:
    {
      if((pt_ear_l->run_flag)&&(pt_ear_r->run_flag))
      {
        pt_move->e_cur_step = EAR_STEP_1;
        pt_move->step_count ++;
      }
      
    }
    break;
    
    default :break;
  }
  
}
void move_ear_LR_task(void *param)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_l   = &t_move.tEarL;
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_r   = &t_move.tEarR;

  uint32_t angle = 90;
  
  T_MOVE_EAR_TYPDEF *pt_move = &t_move_ear_step;
  angle = pt_move->ear_l_angle;
  
  if(pt_move->step_count >= pt_move->step_num)
  {
    move_refesh_ear_den();
    l81_tim_task_den(ID_EAR);
    move_ear_zero();
    
    printf("AT+EARW,%d,%d\r\n",pt_move->e_ear_set, pt_move->step_num);
    MOVE_EAR_L_PWM_OFF();
    MOVE_EAR_R_PWM_OFF();
    return;
  }


  switch(pt_move->e_cur_step)
  {
    case EAR_STEP_0:
    {
    }
    break;
    case EAR_STEP_1:
    {
      if(pt_ear_l->run_flag)
      {
        move_ear_step_set(MOVE_EAR_L, MOVE_SHOCK_1, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_L);
      }
      if(pt_ear_r->run_flag)
      {
        pt_move->e_cur_step = EAR_STEP_2;
        pt_move->e_last_step = EAR_STEP_1;
        
        move_ear_step_set(MOVE_EAR_R, MOVE_SHOCK_1, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_R);
      }
    }
    break;
    case EAR_STEP_2:
    {
      if(pt_ear_l->run_flag)
      {
        move_ear_step_set(MOVE_EAR_L, MOVE_SHOCK_2, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_L);
      }
      if(pt_ear_r->run_flag)
      {
        pt_move->e_cur_step = EAR_STEP_3;
        pt_move->e_last_step = EAR_STEP_2;
        
        move_ear_step_set(MOVE_EAR_R, MOVE_SHOCK_2, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_R);
      }
    }
    break;
    case EAR_STEP_3:
    {
      if(pt_ear_l->run_flag)
      {
        move_ear_step_set(MOVE_EAR_L, MOVE_SHOCK_3, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_L);
      }
      if(pt_ear_r->run_flag)
      {
        pt_move->e_cur_step = EAR_STEP_4;
        pt_move->e_last_step = EAR_STEP_3;
        
        move_ear_step_set(MOVE_EAR_R, MOVE_SHOCK_3, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_R);
      }
    }
    break;
    case EAR_STEP_4:
    {
      if(pt_ear_l->run_flag)
      {
        move_ear_step_set(MOVE_EAR_L, MOVE_SHOCK_4, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_L);
      }
      if(pt_ear_r->run_flag)
      {
        pt_move->e_cur_step = EAR_OVER;
        pt_move->e_last_step = EAR_STEP_2;
        
        move_ear_step_set(MOVE_EAR_R, MOVE_SHOCK_4, pt_move->ear_tim, angle);
        move_ear_step_en(MOVE_EAR_R);
      }
    }
    break;
    case EAR_OVER:
    {
      if((pt_ear_l->run_flag)&&(pt_ear_r->run_flag))
      {
        pt_move->e_cur_step = EAR_STEP_1;
        pt_move->step_count ++;
      }
    }
    break;
    
    default :break;
  }
  
}

void move_ear_set(E_MOVE_EAR_SCENE_TYPDEF e_ear_secne, int32_t step, int32_t delay, uint32_t angle)
{
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_l   = &t_move.tEarL;
  T_MOVE_BODY_DATA_TYPDEF *pt_ear_r   = &t_move.tEarR;
  
  move_refesh_ear_den();
  l81_tim_task_den(ID_EAR);
  
//  t_move_step.u_cur_step.e_work_step = WORK_LEG;
  
  pt_ear_l->run_flag = 1;
  pt_ear_r->run_flag = 1;

  
  

  t_move_ear_step.step_num = step;
  t_move_ear_step.step_count = 0;
  t_move_ear_step.ear_tim = delay;
  
  t_move_ear_step.ear_l_angle = angle; //for turn angle

  switch(e_ear_secne)
  {
    case EAR_SCENE_ZERO:
    {
      t_move_ear_step.e_cur_step = EAR_STEP_1;
      move_refesh_ear_den();
      l81_tim_task_den(ID_EAR);
      move_ear_zero();
    }
    break;
    case EAR_SCENE_LEFT:
    {
      t_move_ear_step.e_cur_step = EAR_STEP_1;
      
      l81_tim_task_creat(ID_EAR, TIM_TASK_CYCLE_ALL, 10, NULL, move_ear_left_task);
      l81_tim_task_en(ID_EAR);
    }
    break;
    case EAR_SCENE_RIGHT:
    {
      t_move_ear_step.e_cur_step = EAR_STEP_1;
      
      l81_tim_task_creat(ID_EAR, TIM_TASK_CYCLE_ALL, 10, NULL, move_ear_right_task);
      l81_tim_task_en(ID_EAR);
    }
    break;
    case EAR_SCENE_LR:
    {
      t_move_ear_step.e_cur_step = EAR_STEP_1;
      
      l81_tim_task_creat(ID_EAR, TIM_TASK_CYCLE_ALL, 10, NULL, move_ear_LR_task);
      l81_tim_task_en(ID_EAR);
    }
    break;

    break;
    default :break;
  }
  
  
}


uint8_t l81_AT_EAR_W_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  uint32_t ear_cmd = 0;
	uint32_t ear_step = 1;
	uint32_t speed = 100;
  uint32_t angle = 90;  //default angle = 90
  
  ATcmd_split_params(params, param, &param_num);
	
	if ((param_num < 3U) || (param_num > 4U))    
  {
    //motor must has three paramters, (motor number, value type(angle or pulse), value)
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
  } 

	
	ear_cmd = String2Int(param[0]);
	ear_step = String2Int(param[1]);
	speed = String2Int(param[2]);
		
	if (ear_cmd < 0U || ear_cmd > 3U) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong EAR CMD %d, ear_cmd  is[0,3]\r\n", ear_cmd);
		printf("AT+RES,end\r\n");
		return 0U;
	}
//	if (move_step < 1U) {
//		printf("AT+RES,ACK\r\n");
//		printf("AT+RES,Err,wrong value_kind %d, value_kind is[0,1]\r\n", value_kind);
//		printf("AT+RES,end\r\n");
//		return 0U;
//	}	
	if (speed < 1 ) 
  {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong value %d, value is[500,2500]\r\n", speed);
		printf("AT+RES,end\r\n");
		return 0U;
	}

  if(param_num == 3U)   // param:cmd,step,speed (default angle = 90)
  {
    angle = 90;
  }
  else if(param_num == 4U)  // param:cmd,step,speed,angle
  {
    angle = String2Int(param[3]); 
    if(angle>90)
    {
      angle = 90;
    }
    printf("angle=%d\r\n",angle);
  }
	
  //2g motor speed 0.06s/60
  if(speed < angle*2)
  {
    speed = angle*2;
  }
	#if 1
  
  t_move_ear_step.e_ear_set = (E_MOVE_EAR_SCENE_TYPDEF)ear_cmd;
  
  //ear ctr pwr on
  MOVE_EAR_L_PWM_ON();
  MOVE_EAR_R_PWM_ON();
  
  switch(ear_cmd)
  {
    case 0:
    {
      motor_set_pulse(5, 1500);
      motor_set_pulse(6, 1500);
      
      printf("AT+EARW,%d,%d\r\n",t_move_ear_step.e_ear_set, ear_step);
      MOVE_EAR_L_PWM_OFF();
      MOVE_EAR_R_PWM_OFF();
    }
    case 1: //left
    {
      move_ear_set(EAR_SCENE_LEFT, ear_step, speed, angle);
    }break;
    case 2: //right
    {
      move_ear_set(EAR_SCENE_RIGHT, ear_step, speed, angle);
    }break;
    case 3: //left-right param(3,3,60)
    {
      move_ear_set(EAR_SCENE_LR, ear_step, speed, angle);
    }break;
   
    default:break;
     
  }
  #endif
  printf("AT+RES,ACK\r\n");
  printf("AT+RES,end\r\n");
  

  return 1U;
}

/*-ear stop-************/

/*-cliff control start-************/

//pMoveState : for backup cur act
//return: moving flag 0=not moving 1=moving
uint8_t move_cliff_control (uint8_t *pMoveState)
{
  uint8_t moving = MOVE_OFF;
  T_MOVE_STEP_TYPDEF *pt_act = &t_move_step;
  
  #if 0
  if(pt_act->e_act_set != SET_ZERO) 
  {
    if(pMoveState != NULL)
    {
      *pMoveState = (uint8_t)pt_act->e_act_set;  //Records the current motion flag
    }
    
    move_stop_en();               //move stop
    l81_tim_task_den(ID_MOVE);
    
    l18_move_zero();              //stand
    pt_act->e_act_set = SET_ZERO; //set stand flag
    
    moving = MOVE_ON;   
  }
  
  #else
  
  switch(pt_act->e_act_set)
  {
    case SET_WORK_F:     //param(step=n,delay=3)
    case SET_WORK_B:     //param(step=n,delay=3)
    case SET_WORK_L:     //param(step=n,delay=3)
    case SET_WORK_R:     //param(step=n,delay=3)
    case SET_CRAB_L:      //param(step,delay)
    case SET_CRAB_R:      //param(step,delay)
    case SET_TURN_L:      //21 param(step=n,)
    case SET_TURN_R:      //22 param(step=n
    case 17:      //17 clock  ATC_SHAKE_BODY_UD
    case 25:      //25 turn left  ACT_TURN_MINI_L
    case 26:      //26 turn right  ACT_TURN_MINI_R
    case 55:      //55 turn left  ATC_TURN_LEFT_MINI_HALF
    case 56:      //56 turn right  ATC_TURN_RIGHT_MINI_HALF
    case 63:      //63
    case 64:      //64
    case 69:      //69
    case 70:      //70
    case 71:      //71
    case 72:      //72
    case 73:      //73
    case 74:      //74
    case 75:      //75
    case 81:      //81
    case 82:      //82
    case 83:      //83
    case 84:      //84
    case 85:      //85 
    case 86:      //86
    case 87:      //87
    case 88:      //88
    case 89:      //89
    case 90:      //90
    case 91:      //91
    case 92:      //92
    case 94:      //94
    case 95:      //95
    {
      if(pMoveState != NULL)
      {
        *pMoveState = (uint8_t)pt_act->e_act_set;  //Records the current motion flag
      }
      
      move_stop_en();               //move stop
      l81_tim_task_den(ID_MOVE);
      
      l18_move_zero();              //stand
      pt_act->e_act_set = SET_ZERO; //set stand flag
      
      moving = MOVE_ON; 
      
    }break;
    
    default:break;
  }
  
  #endif
  
  return moving;   //return moving flag
}

/*-cliff control end-************/







