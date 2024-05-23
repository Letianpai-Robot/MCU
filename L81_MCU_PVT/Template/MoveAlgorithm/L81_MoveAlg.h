/*!
    \file    L81_MoveAlg.h
    \brief   the header for L81_MoveAlg.c
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

#ifndef L81_MOVE_ALG_H
#define L81_MOVE_ALG_H

#include "gd32l23x.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "algorithm.h"

/*define port*/

#define MOVE_EAR_L_PWM_ON()     timer_channel_output_state_config(TIMER2, TIMER_CH_2, TIMER_CCX_ENABLE)   //gpio_bit_set(GPIOC, GPIO_PIN_2)
#define MOVE_EAR_L_PWM_OFF()    timer_channel_output_state_config(TIMER2, TIMER_CH_2, TIMER_CCX_DISABLE)  //gpio_bit_reset(GPIOC, GPIO_PIN_2)

#define MOVE_EAR_R_PWM_ON()     timer_channel_output_state_config(TIMER2, TIMER_CH_3, TIMER_CCX_ENABLE)   //gpio_bit_set(GPIOC, GPIO_PIN_3)
#define MOVE_EAR_R_PWM_OFF()    timer_channel_output_state_config(TIMER2, TIMER_CH_3, TIMER_CCX_DISABLE)  //gpio_bit_reset(GPIOC, GPIO_PIN_3)



/**/


#define MOVE_ERR    1
#define MOVE_OK     0

#define MOVE_ON     1
#define MOVE_OFF    0

#define MOVE_MIN_VALUE  500
#define MOVE_MAX_VALUE  2500
#define MOVE_MIN_ANGLE  0
#define MOVE_MAX_ANGLE  180

#define MOVE_ANGLE3_VALUE 33 


#define MOVE_TIME_SAMPE  30     //time sampe 30ms

//motor dir
typedef enum{
  MOVE_L     = 1U,
	MOVE_R     = 2U,
}E_MOVE_DIRECTION_TYPDEF;

//body motor map
typedef enum{
  MOVE_FOOT_L     = 1U,
	MOVE_FOOT_R     = 2U,
	MOVE_LEG_L      = 4U,
	MOVE_LEG_R      = 3U,
  
	MOVE_EAR_L      = 6U,
	MOVE_EAR_R      = 5U,
}E_MOVE_BODY_TYPDEF;

//body unit action
typedef enum
{
  MOVE_SHOCK_1 = 0, //0   -> 90
  MOVE_SHOCK_2,     //90  -> 0
  MOVE_SHOCK_3,     //0   -> -90
  MOVE_SHOCK_4,     //-90 -> 0
}E_MOVE_SHOCK_TYPDEF;

//walk step flag
typedef enum
{
  WORK_NONE = 0,
  WORK_LEG,
  WORK_FOOT,
  WORK_LEG2,
  WORK_LEG3,
  WORK_FOOT2,
  WORK_FOOT3,
  WORK_LEG4,
  WORK_LEG5,
  WORK_FOOT4,
  WORK_FOOT5,
  WORK_LEG6,
  WORK_LEG7,
  WORK_FOOT6,
  WORK_FOOT7,
  WORK_FOOT8,
  
  WORK_OVER,
  
}E_MOVE_WORK_FLAG_TYPDEF;


typedef enum
{
  STEP_NONE = 0,
  STEP_1,
  STEP_2,
  STEP_3,
  STEP_4,
  STEP_5,
  STEP_6,
  STEP_7,
  STEP_8,
  STEP_9,
  STEP_10,
  STEP_11,
  STEP_12,
  
  STEP_OVER,
  
}E_MOVE_STEP_FLAG_TYPDEF;

//body data
typedef struct
{
  uint8_t stop;
  uint8_t run_flag;
  E_MOVE_BODY_TYPDEF  e_body;
  E_MOVE_SHOCK_TYPDEF e_shock; 
  
  int32_t zero_value;
  
  int32_t delay_value;
  int32_t tim_count;
  int32_t cycle_num;  // delay_value/MOVE_TIME_SAMPE
  int32_t cycle_count;  //
  int32_t step_num;
  int32_t step_count;
  
  int32_t last_set_angle;
  int32_t last_set_value;
  int32_t last_read_value;
  
  int32_t cur_set_angle;
  int32_t cur_set_value;
  int32_t cur_read_value;
  
  int32_t target_angle;    //
  
}T_MOVE_BODY_DATA_TYPDEF;

//body object
typedef struct
{
  
  T_MOVE_BODY_DATA_TYPDEF tFootL;
  T_MOVE_BODY_DATA_TYPDEF tFootR;
  T_MOVE_BODY_DATA_TYPDEF tLegL;
  T_MOVE_BODY_DATA_TYPDEF tLegR;
  
  T_MOVE_BODY_DATA_TYPDEF tEarL;
  T_MOVE_BODY_DATA_TYPDEF tEarR;
  
}T_MOVE_TYPDEF;


typedef enum
{
  SET_ZERO = 0U,
  SET_WORK_F,     //param(step=n,delay=3)
  SET_WORK_B,     //param(step=n,delay=3)
  SET_WORK_L,     //param(step=n,delay=3)
  SET_WORK_R,     //param(step=n,delay=3)
  SET_CRAB_L,      //param(step,delay)
  SET_CRAB_R,      //param(step,delay)
  SET_SHAKE_LEG_L, //param(step=1,delay=2)
  SET_SHAKE_LEG_R, //param(step=1,delay=2)
  SET_SHAKE_FOOT_L, //param(step=2,delay=3)
  SET_SHAKE_FOOT_R, //param(step=2,delay=3)
  
  SET_CROSS_LEGS_L,  //param(step,delay)
  SET_CROSS_LEGS_R,  //param(step,delay)
  SET_LEAN_L,       //param(step=1,delay=6)
  SET_LEAN_R,       //param(step=1,delay=6)
  SET_STOMP_L,      //param(step=1,delay=3)
  SET_STOMP_R,      //param(step=1,delay=3)
  SET_SHAKE_BODY_UD, //param(step=1,)
  SET_SHAKE_BODY_LR, //param(step=1,)
  SET_SHAKE_HEAD_LR, //param(step=2,)
  SET_STAND_EASE,    //param(step,delay)
  
  SET_TURN_L,      //param(step=n,)
  SET_TURN_R,      //param(step=n,)
  SET_SHAKE_FOOT,         //param(step=3,delay=1)
  SET_SHAKE_FOOT_TEMPO,   //param(step=3,delay=1)
  SET_TURN_MINI_L,      //param(step=n,)
  SET_TURN_MINI_R,      //param(step=n,)
  SET_SWAY,      //param(step=n,)
	
	SET_RANDOM28,
	SET_RANDOM29,
	SET_RANDOM30,
	SET_RANDOM31,
	SET_RANDOM32,
	SET_RANDOM33,
	SET_RANDOM34,
	SET_RANDOM35,
	SET_RANDOM36,
	SET_RANDOM37,
	SET_RANDOM38,
	SET_RANDOM39,
	SET_RANDOM40,
	SET_RANDOM41,
	SET_RANDOM42,
	
  SET_SHAKE_HEAD_SMALL_ANGLE, //param(step=2,)  43
  SET_INCLINE_L,      //param(step=n,)
  SET_INCLINE_R,      //param(step=n,)
  SET_INCLINE_SMALL_ANGLE_L,      //param(step=n,)
  SET_INCLINE_SMALL_ANGLE_R,      //param(step=n,)
	SET_SHAKE_FOOT_QUICK,
	SET_SHAKE_FOOT_IN_HALF,
	SET_TWIST,
	SET_SHAKE_FOOT_SMALL_ANGLE_L, //param(step=1,)
	SET_SHAKE_FOOT_SMALL_ANGLE_R, //param(step=1,)
	SET_MOVE_LEFT_OUT_HALF,
	SET_MOVE_RIGHT_OUT_HALF,
	SET_TURN_LEFT_MINI_HALF,
	SET_TURN_RIGHT_MINI_HALF,  //56
	SET_SHAKE_RIGHT_MINI_HALF,
	SET_INCLINE_BIG_ANGLE_R,
	SET_CLAMP_LEG_SHAKE_FOOT_IN_LR,
	SET_CLAMP_LEG_SHAKE_FOOT_LR,
	SET_OPEN_LEG_L,
	SET_OPEN_LEG_R,
	SET_63,
	SET_64,
	SET_65,
	SET_66,
	SET_67,
	SET_68,
	SET_69,
	SET_70,
	SET_71,
	SET_72,
	SET_73,
	SET_74,
	SET_75,
	SET_76,
	SET_77,
	SET_78,
	SET_79,
	SET_80, //0627-dance
	SET_81,
	SET_82,
	SET_83,
	SET_84,
	SET_85,
	SET_86,
	SET_87,
	SET_88,
	SET_89,
	SET_90,
	SET_91,
	SET_92,
	SET_93,
	SET_94,
	SET_95,
	SET_96,
	SET_97,
	SET_98,
	SET_99,
	SET_100,
	SET_101,
	SET_102,
	SET_103,
	SET_104,
	SET_105,
	SET_106,
	SET_107,
	SET_108,
	SET_109,
	SET_110,
	SET_111,
	SET_112,
	SET_113,
	SET_114,
	SET_115,
	SET_116,
	SET_117,
	SET_118,
	SET_119,
	SET_120,
	SET_121,
	SET_122,
	SET_123,
	SET_124,
	SET_125,
	SET_126,
	SET_127,
	SET_128,
	SET_129,
	SET_130,
	SET_131,
	SET_132,
	SET_133,
	SET_134,
	SET_135,
	SET_136,
	SET_137,
	SET_138,
	SET_139,

}E_MOVE_ACTION_SET_TYPDEF;

/*-body start-************/
typedef union
{
  E_MOVE_WORK_FLAG_TYPDEF e_work_step;
  E_MOVE_STEP_FLAG_TYPDEF e_step_num;
}U_MOVE_STEP_TYPDEF;  



//walk 
typedef enum
{
  WORK_STOP = 0,
  WORK_DIR_F = 1,
  WORK_DIR_B,
  WORK_DIR_L,
  WORK_DIR_R,
  
}E_MOVE_WORK_DIR_TYPDEF;

//move unit  angle:(little=15 middle=30 large=60)
typedef enum
{
  UNIT_NONE = 0U,
  UNIT_LEG_L0,  //any angle
  UNIT_LEG_L1,  //angle_little
  UNIT_LEG_L2,  //angle_middle
  UNIT_LEG_L3,  //angle_large
  
  UNIT_LEG_R0,  //any angle
  UNIT_LEG_R1,  //angle_little
  UNIT_LEG_R2,  //angle_middle
  UNIT_LEG_R3,  //angle_large
  
  UNIT_FOOT_L0,  //any angle
  UNIT_FOOT_L1,  //angle_little
  UNIT_FOOT_L2,  //angle_middle
  UNIT_FOOT_L3,  //angle_large
  
  UNIT_FOOT_R0,  //any angle
  UNIT_FOOT_R1,  //angle_little
  UNIT_FOOT_R2,  //angle_middle
  UNIT_FOOT_R3,  //angle_large
  
}E_MOVE_UNIT_TYPDEF;

//action flag
typedef enum
{
  ACT_NONE = 0U,
  ACT_CRAB_L = 5U,      //param(step,delay)
  ACT_CRAB_R,
  ATC_SHAKE_LEG_L, //param(step=1,delay=2)
  ATC_SHAKE_LEG_R, //param(step=1,delay=2)

  ATC_SHAKE_FOOT_L, //param(step=2,delay=3)
  ATC_SHAKE_FOOT_R, //param(step=2,delay=3)
  
  ATC_CROSS_LEGS_L, //
  ATC_CROSS_LEGS_R, //
  
  ATC_LEAN_L,   //param(step=1,delay=6)
  ATC_LEAN_R,   //param(step=1,delay=6)
  
  ATC_STOMP_L,  //param(step=1,delay=3)
  ATC_STOMP_R,  //param(step=1,delay=3)
  
  ATC_SHAKE_BODY_UD, //param(step=1,)
  ATC_SHAKE_BODY_LR, //param(step=1,)
  ATC_SHAKE_HEAD_LR, //param(step=2,)
  
  ATC_STAND_EASE, 
  
  ACT_TURN_L,      //param(step=n,)
  ACT_TURN_R,      //param(step=n,)
  
  ACT_CLAMP_LEG,   //param(step=3,delay=1)  //ACT_SHAKE_FOOT,   //param(step=3,delay=1)  Clamp leg
  ACT_SHAKE_FOOT_TEMPO,   //param(step=3,delay=1)
	
  ACT_TURN_MINI_L,      //param(step=n,)
  ACT_TURN_MINI_R,      //param(step=n,)
  
  ACT_SWAY,      //param(step=n,)
	
	ACT_RANDOM28,
	ACT_RANDOM29,
	ACT_RANDOM30,
	ACT_RANDOM31,
	ACT_RANDOM32,
	ACT_RANDOM33,
	ACT_RANDOM34,
	ACT_RANDOM35,
	ACT_RANDOM36,
	ACT_RANDOM37,
	ACT_RANDOM38,
	ACT_RANDOM39,
	ACT_RANDOM40,
	ACT_RANDOM41,
	ACT_RANDOM42,
	
  ATC_SHAKE_HEAD_SMALL_ANGLE, //param(step=2,)  43
  ACT_INCLINE_L,      //param(step=n,)
  ACT_INCLINE_R,      //param(step=n,)
  ACT_INCLINE_SMALL_ANGLE_L,      //param(step=n,)
  ACT_INCLINE_SMALL_ANGLE_R,      //param(step=n,)
	ATC_SHAKE_FOOT_QUICK,
	ATC_SHAKE_FOOT_IN_HALF,
	ACT_TWIST,
	ATC_SHAKE_FOOT_SMALL_ANGLE_L, //param(step=1,)
	ATC_SHAKE_FOOT_SMALL_ANGLE_R, //param(step=1,)
	ATC_MOVE_LEFT_OUT_HALF,
	ATC_MOVE_RIGHT_OUT_HALF,
	ATC_TURN_LEFT_MINI_HALF,
	ATC_TURN_RIGHT_MINI_HALF,  //56
	ATC_SHAKE_RIGHT_MINI_HALF,
	ACT_INCLINE_BIG_ANGLE_R,
	ACT_CLAMP_LEG_SHAKE_FOOT_IN_LR,
	ACT_CLAMP_LEG_SHAKE_FOOT_LR,
	ACT_OPEN_LEG_L,
	ACT_OPEN_LEG_R,
	ACT_63,
	ACT_64,
	ACT_65,
	ACT_66,
	ACT_67,
	ACT_68,
	ACT_69,
	ACT_70,
	ACT_71,
	ACT_72,
	ACT_73,
	ACT_74,
	ACT_75,
	ACT_76,
	ACT_77,
	ACT_78,
	ACT_79,
	ACT_80, //0627-dance 
	ACT_81,
	ACT_82,
	ACT_83,
	ACT_84,
	ACT_85,
	ACT_86,
	ACT_87,
	ACT_88,
	ACT_89,
	ACT_90,
	ACT_91,
	ACT_92,
	ACT_93,
	ACT_94,
	ACT_95,
	ACT_96,
	ACT_97, //0816  walk f
	ACT_98,
	ACT_99,
	ACT_100,
	ACT_101,
	ACT_102,
	ACT_103,
	ACT_104,
	ACT_105,
	ACT_106,
	ACT_107,
	ACT_108,
	ACT_109,
	ACT_110,
	ACT_111,
	ACT_112,
	ACT_113,
	ACT_114,
	ACT_115,
	ACT_116,
	ACT_117,
	ACT_118,
	ACT_119,
	ACT_120,
	ACT_121,
	ACT_122,
	ACT_123,
	ACT_124,
	ACT_125,
	ACT_126,
	ACT_127,
	ACT_128,
	ACT_129,
	ACT_130,
	ACT_131,
	ACT_132,
	ACT_133,
	ACT_134,
	ACT_135,
	ACT_136,
	ACT_137,
	ACT_138,
	ACT_139,
	

//  ATC_WORK_ALGORITHM25, //
//  ATC_WORK_ALGORITHM35, //
  
  
}E_MOVE_ACTION_TYPDEF;



typedef struct
{
  uint8_t  step_en;
  uint8_t  step_type; //1-forward 2-backward 3-left 4-right
  
  int8_t  leg_l_angle;   //
  int8_t  leg_r_angle;
  int8_t  foot_l_angle;  //
  int8_t  foot_r_angle;
  
  uint32_t leg_tim;
  uint32_t foot_tim;
  uint32_t leg_foot_tim;
  
  
  
//  E_MOVE_WORK_FLAG_TYPDEF e_step;
  U_MOVE_STEP_TYPDEF u_last_step;
  U_MOVE_STEP_TYPDEF u_cur_step;
  
  E_MOVE_ACTION_SET_TYPDEF e_act_set;

  uint32_t step_num;
  uint32_t step_count;
}T_MOVE_STEP_TYPDEF;

/*-body end-************/


/*-ear start-************/

typedef enum
{
  EAR_STEP_0 = 0,  //none or zero
  EAR_STEP_1 = 1,  //0->90
  EAR_STEP_2,      //90->0
  EAR_STEP_3,      //0->-90
  EAR_STEP_4,      //-90->0
  EAR_OVER,      //
  
}E_MOVE_EAR_STEP_TYPDEF;


typedef enum
{
  EAR_SCENE_STOP = 0,
  EAR_SCENE_ZERO,
  EAR_SCENE_LEFT,
  EAR_SCENE_RIGHT,
  EAR_SCENE_LR,     //param(step=3,delay=60)
  
}E_MOVE_EAR_SCENE_TYPDEF;


typedef struct
{
  uint8_t  step_en;
  uint8_t  step_type; //1-forward 2-backward 3-left 4-right
  
  int8_t  ear_l_angle;   //
  int8_t  ear_r_angle;

  uint32_t ear_tim;

  E_MOVE_EAR_STEP_TYPDEF e_last_step;
  E_MOVE_EAR_STEP_TYPDEF e_cur_step;
  
  E_MOVE_EAR_SCENE_TYPDEF e_ear_set;

  uint32_t step_num;
  uint32_t step_count;
}T_MOVE_EAR_TYPDEF;

/*-ear end-************/


extern T_MOVE_TYPDEF t_move;
extern T_MOVE_STEP_TYPDEF t_move_step;
extern T_MOVE_EAR_TYPDEF  t_move_ear_step;

uint8_t l81_move_body_zero(E_MOVE_BODY_TYPDEF e_body);
uint8_t l81_move_zero_read();



void l81_move_init();

void l81_move_zero_set(uint8_t i, uint32_t value);
uint8_t l81_move_body_write(E_MOVE_BODY_TYPDEF e_body, E_MOVE_DIRECTION_TYPDEF e_dir, uint32_t angle);

//void l81_move_walk(void);
uint8_t move_cliff_control (uint8_t *pMoveState);

uint8_t l81_AT_MOVE_W_func(char params[]);
uint8_t l81_AT_EAR_W_func(char params[]);

uint8_t is_moving();

#endif  //L81_MOTORS_H