/*!
    \file    L81_TimTask.c
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

#include "L81_inf.h"
#include "L81_TimTask.h"



/************************************motor operation brief*******************************************/
/* 

*/
/****************************************************************************************************/

uint8_t tim_task_flag = TIM_TASK_UNUSE;
T_TIM_TASK_TYPDEF tTimTask[TIM_TASK_BUF_LEN] = {0};

/***********************************************************************/
/*!
    \brief      tim it init, cycle 10ms
    \param[in]  none
    \param[out] none
    \retval     none
*/
void l81_tim_it_init(void)
{
  
  /* TIMER6 configuration: input capture mode -------------------
    the external signal is connected to TIMER6 CH0 pin (PA0)
    the rising edge is used as active edge
    the TIMER6 CH0CV is used to compute the frequency value
    ------------------------------------------------------------ */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;

    /* enable the peripherals clock */
    rcu_periph_clock_enable(RCU_TIMER6);

    /* deinit a TIMER */
    timer_deinit(TIMER6);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER6 configuration */
    timer_initpara.prescaler        = 63999; //
    timer_initpara.alignedmode      = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period           = 9; //10ms
    timer_initpara.clockdivision    = TIMER_CKDIV_DIV1;
    timer_init(TIMER6, &timer_initpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER6);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER6, TIMER_INT_FLAG_UP);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER6, TIMER_INT_UP);

    nvic_irq_enable(TIMER6_IRQn, 2);
    /* enable a TIMER */
    timer_enable(TIMER6);
  
    
}


#if 0 //move to **it.c
/*!
    \brief      tim it call
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER6_IRQHandler()
{
  if(SET == timer_interrupt_flag_get(TIMER6, TIMER_INT_FLAG_UP)) 
  {
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER6, TIMER_INT_FLAG_UP);
    
    tim_task_flag = TIM_TASK_USE; //
  }
}
#endif

void tim_call(void)
{
  tim_task_flag = TIM_TASK_USE; //
}
/***********************************************************************/

void l81_tim_task_init(void)
{
  //tim init
  l81_tim_it_init();
  
  for(uint16_t i=0; i<TIM_TASK_BUF_LEN; i++)
  {
    tTimTask[i].tim_use = TIM_TASK_UNUSE;
    tTimTask[i].tim_type = 0;
    tTimTask[i].tim_delay = 0;
    tTimTask[i].tim_count = 0;
    tTimTask[i].param = NULL;
    tTimTask[i].FunTimTask = NULL;
    
  }
}

/*!
    \brief      tim task creat,
    \param[in]  tim_id : 0~TIM_TASK_BUF_LEN
    \param[in]  tim_type :TIM_TASK_CYCLE_ONCE,TIM_TASK_CYCLE_ALL
    \param[in]  tim_delay  ,ms delay
    \param[in]  param  
    \param[in]  fun
    \retval     err: TIM_TASK_OK, TIM_TASK_ERR
*/
uint8_t  l81_tim_task_creat(uint8_t tim_id, uint8_t tim_type, uint32_t tim_delay, void *param, funTimTask fun)
{
  //
  if(tim_id >= TIM_TASK_BUF_LEN)
  {
    return TIM_TASK_ERR; //err
  }

  if((tim_type != TIM_TASK_CYCLE_ONCE) && (tim_type != TIM_TASK_CYCLE_ALL))
  {
    return TIM_TASK_ERR; //err
  }
    
  if(fun == NULL)
  {
    return TIM_TASK_ERR; //err
  }
  
  
  
  
  tTimTask[tim_id].tim_type   = tim_type;
  tTimTask[tim_id].tim_delay  = tim_delay;
  tTimTask[tim_id].param      = param;
  tTimTask[tim_id].FunTimTask = fun;
  
  return TIM_TASK_OK;
}


/*!
    \brief      tim task creat
    \param[in]  tim_id : 0~TIM_TASK_BUF_LEN
    \retval     err: TIM_TASK_OK, TIM_TASK_ERR
*/
uint8_t  l81_tim_task_en(uint8_t tim_id)
{
  if(tim_id >= TIM_TASK_BUF_LEN)
  {
    return TIM_TASK_ERR; //err
  }
  tTimTask[tim_id].tim_use = TIM_TASK_USE;
  tTimTask[tim_id].tim_count = 0;
  
  return TIM_TASK_OK;
}

/*!
    \brief      tim task creat
    \param[in]  tim_id : 0~TIM_TASK_BUF_LEN
    \retval     err: TIM_TASK_OK, TIM_TASK_ERR
*/
uint8_t  l81_tim_task_den(uint8_t tim_id)
{
  if(tim_id >= TIM_TASK_BUF_LEN)
  {
    return TIM_TASK_ERR; //err
  }
  
  tTimTask[tim_id].tim_use = TIM_TASK_UNUSE;
  
  return TIM_TASK_OK;
}

/*!
    \brief      tim task creat
    \param[in]  tim_id : 0~TIM_TASK_BUF_LEN
    \param[in]  param  
    \retval     err: TIM_TASK_OK, TIM_TASK_ERR
*/
uint8_t  l81_tim_task_reset_param(uint8_t tim_id, void *param)
{
  if(param == NULL)
  {
    return TIM_TASK_ERR; //err
  }
  
  tTimTask[tim_id].param = param;
  
  return TIM_TASK_OK;
}

/*!
    \brief      tim task reset delay
    \param[in]  tim_id : 0~TIM_TASK_BUF_LEN
    \param[in]  tim_delay  ,ms delay
    \retval     err: TIM_TASK_OK, TIM_TASK_ERR
*/
uint8_t  l81_tim_task_reset_delay(uint8_t tim_id, uint32_t tim_delay)
{
  if(tim_id >= TIM_TASK_BUF_LEN)
  {
    return TIM_TASK_ERR; //err
  }
  
  tTimTask[tim_id].tim_delay = tim_delay;
  
  return TIM_TASK_OK;
}

/*!
    \brief      tim task server,need put in main while()
    \param[in]  none
    \param[out] none
    \retval     none
*/
void l81_tim_task_serve(void)
{
  if(tim_task_flag == TIM_TASK_USE)
  {
    tim_task_flag = TIM_TASK_UNUSE;
    
    for(uint16_t i=0; i<TIM_TASK_BUF_LEN; i++)
    {
      if(tTimTask[i].tim_use == TIM_TASK_UNUSE)
      {
        continue;
      }
      
      tTimTask[i].tim_count += TIM_TASK_PERIOD;
      if(tTimTask[i].tim_count >= tTimTask[i].tim_delay)
      {
        if(tTimTask[i].FunTimTask != NULL)
        {
          tTimTask[i].FunTimTask(tTimTask[i].param);
        }
        
        tTimTask[i].tim_count = 0;
        if(tTimTask[i].tim_type == TIM_TASK_CYCLE_ONCE)
        {
          tTimTask[i].tim_use = TIM_TASK_UNUSE;  
        }
      }
      
    }
  }
}

















