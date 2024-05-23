/*!
    \file    L81_cliff.c
    \brief   the basical interface for fall sensor (ITR1502SR40A)
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
#include "main.h"
#include <string.h>
#include "L81_inf.h"
#include "L81_cliff.h"
#include "L81_AT.h"
#include "L81_FMC.h"

volatile CLIFF_OnOff_ENUM cliff_OnOff = cliff_Off;
volatile CLIFF_STATUS_ENUM cliff_status = cliff_idle;
volatile uint8_t cliff_lock = 0u;
volatile static CLIFF_DANGER_ENUM cliff_danger_status = cliff_safe;

void l81_cliff_lock(void)
{
		cliff_lock = 1u;
}

void l81_cliff_unlock(void)
{
		cliff_lock = 0u;
}

uint8_t l81_cliff_getlock(void)
{
		return cliff_lock;
}
/**
    \brief      configure the GPIO ports
								PB1 is used for CLIFF EN1 and PB2 is used for CLIFF EN2
    \param[in]  none
    \param[out] none
    \retval     none
  */
void l81_cliff_en_gpio_config(void)
{
		/* enable the GPIO clock */
  rcu_periph_clock_enable(RCU_GPIOB);
	
	/* configure LED GPIO pin */
  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_1 | GPIO_PIN_2);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1 | GPIO_PIN_2);
	
	/*low disable cliff and high enable cliff. set disable in default. gpio_bit_set is low and gpio_bit_reset is high*/
	gpio_bit_set(GPIOB, GPIO_PIN_1 | GPIO_PIN_2);
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void l81_cliff_timer_nvic_config(void)
{
    nvic_irq_enable(TIMER1_IRQn, 0);
}

/*!
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void l81_cliff_timer_config(void)
{
    /* TIMER1 configuration: input capture mode -------------------
    the external signal is connected to TIMER1 CH0 pin (PA0)
    the rising edge is used as active edge
    the TIMER1 CH0CV is used to compute the frequency value
    ------------------------------------------------------------ */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;

    /* enable the peripherals clock */
    rcu_periph_clock_enable(RCU_TIMER1);

    /* deinit a TIMER */
    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler        = 63;
    timer_initpara.alignedmode      = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period           = 10000;
    timer_initpara.clockdivision    = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER1, TIMER_INT_UP);

    /* enable a TIMER */
    timer_enable(TIMER1);
}

uint16_t l81_cliff_get_rightB_adc(void)
{
		return l81_adc_channel_get_sample(ADC_CHANNEL_14);
}

uint16_t l81_cliff_get_rightF_adc(void)
{
		return l81_adc_channel_get_sample(ADC_CHANNEL_15);
}

uint16_t l81_cliff_get_leftF_adc(void)
{
		return l81_adc_channel_get_sample(ADC_CHANNEL_7);
}

uint16_t l81_cliff_get_leftB_adc(void)
{
		return l81_adc_channel_get_sample(ADC_CHANNEL_6);
}

/*must be get data during cliff running*/
void l81_cliff_get_adc(uint16_t *leftF, uint16_t *leftB, uint16_t *rightF, uint16_t *rightB)
{	
	if (cliff_status == cliff_run){
		
		l81_cliff_lock();
		*leftF = l81_cliff_get_leftF_adc(); 
		*leftB = l81_cliff_get_leftB_adc();
		*rightF = l81_cliff_get_rightF_adc();
		*rightB = l81_cliff_get_rightB_adc(); 
		l81_cliff_unlock();
	}
}

void l81_cliff_danger_status(void)
{
	uint16_t leftF = 0u, leftB = 0u, rightF = 0u, rightB = 0u;
	
	if (cliff_status == cliff_run){
		l81_cliff_get_adc(&leftF, &leftB, &rightF, &rightB);
	
		if (leftF >= 3000U || leftB >= 3000u || rightF >= 3000u || rightB >= 3000u)
			cliff_danger_status = cliff_danger;
		else
			cliff_danger_status = cliff_safe;
	}
}
uint8_t is_printf_cliff = 0;
uint8_t l81_AT_CLIFF_R_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

	uint16_t leftF = 0u, leftB = 0u, rightF = 0u, rightB = 0u;
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num == 1U){  //1124 rjq++
		if (String2Int(param[0]) == 1){
			is_printf_cliff = 1;
		}else if (String2Int(param[0]) == 0){
			is_printf_cliff = 0;
		}
	}
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}

	if (cliff_status == cliff_run){
		l81_cliff_get_adc(&leftF, &leftB, &rightF, &rightB);
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,%d,%d,%d,%d\r\n", leftF, leftB, rightF, rightB);
		printf("AT+RES,end\r\n");
	}else{
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,cliff is in idle status, please try read\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
    return 1U;		
}
uint8_t l81_AT_CLIFF_W_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

	uint16_t leftF = 0u, leftB = 0u, rightF = 0u, rightB = 0u;
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}

	if (cliff_status == cliff_run){
		l81_cliff_get_adc(&leftF, &leftB, &rightF, &rightB);
		
		if(leftF < 3000 || leftB < 3000  || rightF < 3000 || rightB < 3000)
		{
			printf("AT+RES,ACK\r\n");
		  printf("AT+RES,Err,cliff data too small:%d,%d,%d,%d\r\n", leftF, leftB, rightF, rightB);
			printf("AT+RES,end\r\n");
		  return 0U;
		}
		
		
		l81_fmc_program(Cliff_ADDR + 0 * sizeof(uint32_t), 0);
		l81_fmc_program(Cliff_ADDR + 1 * sizeof(uint32_t), 0);
		l81_fmc_program(Cliff_ADDR + 2 * sizeof(uint32_t), 0);
		l81_fmc_program(Cliff_ADDR + 3 * sizeof(uint32_t), 0);
		
		uint32_t addr = Cliff_ADDR;
		uint32_t data = leftF;
		l81_fmc_program(addr, data);
	  if (!l81_fmc_program_check(addr, data)){
			printf("AT+RES,ACK\r\n");
		  printf("AT+RES,Err,write data 0x%x in addr 0x%x failed\r\n", data, addr);
			printf("AT+RES,end\r\n");
		  return 0U;
	  } 		
		addr = Cliff_ADDR + sizeof(uint32_t);
		data  = leftB;
		l81_fmc_program(addr, data);
	  if (!l81_fmc_program_check(addr, data)){
			printf("AT+RES,ACK\r\n");
		  printf("AT+RES,Err,write data 0x%x in addr 0x%x failed\r\n", data, addr);
			printf("AT+RES,end\r\n");
		  return 0U;
	  } 		
		addr = Cliff_ADDR + 2 * sizeof(uint32_t);
		data  = rightF;
		l81_fmc_program(addr, data);
	  if (!l81_fmc_program_check(addr, data)){
			printf("AT+RES,ACK\r\n");
		  printf("AT+RES,Err,write data 0x%x in addr 0x%x failed\r\n", data, addr);
			printf("AT+RES,end\r\n");
		  return 0U;
	  } 	
		addr = Cliff_ADDR + 3 * sizeof(uint32_t);
		data  = rightB;
		l81_fmc_program(addr, data);
	  if (!l81_fmc_program_check(addr, data)){
			printf("AT+RES,ACK\r\n");
		  printf("AT+RES,Err,write data 0x%x in addr 0x%x failed\r\n", data, addr);
			printf("AT+RES,end\r\n");
		  return 0U;
	  } 	
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,%d,%d,%d,%d\r\n", leftF, leftB, rightF, rightB);
		printf("AT+RES,end\r\n");
		
	}else{
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,cliff is in idle status, please try read\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
    return 1U;		
}

uint8_t l81_AT_CLIFF_Danger_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	if (cliff_danger_status == cliff_danger){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,danger\r\n");
		printf("AT+RES,end\r\n");
	}
	if (cliff_danger_status == cliff_safe){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,safe\r\n");
		printf("AT+RES,end\r\n");
	}
	
    return 1U;		
}

void L81_cliff_init(void)
{
	l81_cliff_en_gpio_config();
	l81_cliff_timer_nvic_config();
	l81_cliff_timer_config();
}
