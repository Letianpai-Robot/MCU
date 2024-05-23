/*!
    \file    L81_IR.c
    \brief   the basical interface for IR
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
#include <string.h>
#include "L81_inf.h"
#include "L81_AT.h"
#include "L81_IR.h"

typedef enum{
	LEFT  = 0x4c,  //'L'
	RIGHT = 0x52,  //'R'
	NONE  = 0x0,
}single_status_e;

/*
比较函数封装
time1 -- 要检测的数据
time2 -- 标准数据
range1 -- 下限
range2 -- 上限
*/
volatile uint32_t guide=0;
volatile uint32_t ir_buff[105]; //存放捕获的计数器的值
//uint8_t ir_temp[105];
volatile uint8_t ir_data[5];
//uint16_t ir_count=0; //保存边沿的个数
volatile uint32_t cache = 0; 
//uint32_t ir_temp = 0; 
volatile uint8_t  ir_interrupt_flag;//0_up,1_dwn;
//uint8_t  sign=0;
volatile single_status_e  direction=NONE;
volatile static uint16_t times = 0;   
volatile static uint8_t rep_times = 0;   //用于第一次接收重复码时，将上一次重复码次数清除掉  
volatile static uint64_t counter_15ms = 0;

                            //捕获次数，用于标记是否是第一次捕获，0是第一次，非0不是
														
volatile uint8_t ir_data_begin_index = 0u;
volatile uint8_t ir_data_buffer[6u] = {0};
volatile uint8_t IR_status = 0u;
/**
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
  */

void l81_IR_gpio_config(void)
{	
    // LPTIMER GPIO RCU //
    rcu_periph_clock_enable(RCU_GPIOD);

	//pin 6 right, pin 9 left
	gpio_mode_set(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_9);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_9);

}
 

/*!
    \brief      initialize the EXTI configuration of the key
    \param[in]  none
    \param[out] none
    \retval     none
	
	LPTIMER_IN1___PD6
	LPTIMER_IN0___PD9
*/
void key_exti_init(void)
{
    /* enable the SYSCFG clock */
    rcu_periph_clock_enable(RCU_SYSCFG);

    /* connect key EXTI line to key GPIO pin */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOD, EXTI_SOURCE_PIN6);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOD, EXTI_SOURCE_PIN9);
    exti_init(EXTI_6, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    
	  exti_init(EXTI_9, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
	  exti_interrupt_flag_clear(EXTI_6);
    exti_interrupt_flag_clear(EXTI_9);
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void l81_IR_key_nvic_config(void)
{
   nvic_irq_disable(EXTI5_9_IRQn);
}

/*
引导码-13.5ms: 9ms_1;4.5ms_0;
逻辑1-2.25ms，数据 1(2.25ms:)：560us低电平 + 1690us高电平
逻辑0-1.12ms。数据 0(1.12ms) ：560us低电平 + 560us高电平
 前导码 + 地址码 + 数据码 + 结束码  
*/
void IR_processed()
{
//	if(cache > 50 )
//  	printf("AT+RES,cache=%d,times=%d\r\n",cache,times);
	if(cache < 100 )
		return;
	ir_buff[times++] = cache;
	if(times < 40)
		return;
	ir_buff[104]++;
	times = 0;
	
//	
//	if(ir_buff[104] < 5)   
//		return;
		// 10轮查一次  一分半会i2c bus is busy in read!
		// 20轮查一次  十分钟会i2c bus is busy in read!
	ir_buff[104] = 0;
//			for(int k = 0;k < 90; k++)		   
//				printf("%d  ",ir_buff[k]);  
//			printf("\r\n");
	for(int i=0;i<90;i++)
	{
		if(i + 18 >= 90) return;
		if(ir_buff[i] < 8000 || ir_buff[i] > 9900) continue;  //{printf("ir_buff[i] < 8000 || ir_buff[i] > 9900\r\n");continue;}
		if(ir_buff[i+1]<4000 || ir_buff[i+1]>5000)  continue;  //{printf("ir_buff[i+1]<4000 || ir_buff[i+1]\r\n");continue;}
		//if(ir_buff[i+17]<4000 || ir_buff[i+17]>5000 )  continue;  //{printf("ir_buff[i+17]<4000 || ir_buff[i+17]>5000\r\n");continue;}
		if(ir_buff[i + 18] < 8000 || ir_buff[i + 18] > 9900)  continue;  //{printf("ir_buff[i + 18] < 8000 || ir_buff[i + 18] > 9900\r\n");continue;}
		int temp = 0;
		for(int j = i + 2;j< i+ 16;j++)
		{
			if(ir_buff[j] < 200 || ir_buff[j] > 2000)
				{
//					printf("%d %d not 0 or 1 \r\n",j,ir_buff[j]);
//					for(int k = i+3;k < i+18; k++)	
//					{			
//						printf("%d  ",ir_buff[k]);
//					}  
//					printf("\r\n");
					temp = 1;
					break;
				}
		}
		if(temp == 0)
		{
			int val = 0;
			for(int k = i+3;k < i+18; k=k+2)	
			{			
				if(ir_buff[k] > 1000)
					val++;
				if(ir_buff[k] > 10000)
					val--;
				val<<=1;
			}
			printf("AT+IR,%d\r\n",val/2);  
			
//			for(int k = i+3;k < i+18; k++)	
//			{			
//				printf("%d  ",ir_buff[k]);
//			}  
//				printf("\r\n");
//			
//			printf("\r\n");
//			if(ir_buff[i+17] > 10000)
//				printf("%d not val \r\n",ir_buff[i+17]);
			//return;
			
			
		}
	}
	
}

 
void  ir_stop(void)
{
	nvic_irq_disable(TIMER8_IRQn);
	nvic_irq_disable(EXTI5_9_IRQn);
	timer_disable(TIMER8);
	printf("AT+RES,IR stop\r\n");
}

void  ir_start(void)
{
	nvic_irq_enable(TIMER8_IRQn, 1u);
	nvic_irq_enable(EXTI5_9_IRQn, 1u);
	timer_enable(TIMER8);
	printf("AT+RES,IR start\r\n");
	times =0;
}
void get_ir_data(void)
{
	uint8_t i = 0u;
	uint8_t j = 0u;
	uint8_t bit_count = 8u;

	memset((void*)ir_data_buffer, '\0', 6u);
	if (IR_status){	
		if (ir_buff[0] == 9000u){
			for(i = 2u; i < 18u; i += 2){
				if (ir_buff[i+1u] == 1690u)
					ir_data_buffer[j] |= 1u << (bit_count - 1u);
				bit_count--;
				if (bit_count == 0){
//					bit_count = 8u;
//					j++;
//					if (j == 3)
						break;
				}
			}
		}
//		j = 3u;
//		if (ir_buff[50] == 9000u){
//			for(i = 52u; i < 100u; i += 2){
//				if (ir_buff[i+1u] == 1690u)
//					ir_data_buffer[j] |= 1u << (bit_count - 1u);
//				bit_count--;
//				if (bit_count == 0){
//					bit_count = 8u;
//					j++;
//					if (j == 6u)
//						break;
//				}
//			}		
//		}

		if (ir_buff[0] == 9000u ){
			if(ir_data_buffer[0]>0&&ir_data_buffer[0]<9)
							printf("AT+RES,IR_DATA:%d \r\n",ir_data_buffer[0]);
//			IR_status=0;
//			if (ir_data_buffer[3] == 0x81)
//				printf("AT+RES,IR_DATA:%c %x %x %x\r\n",direction, ir_data_buffer[3], ir_data_buffer[4],ir_data_buffer[5]);
		}
		
		memset((void*)ir_buff, '\0', 105);
	}
}
//void get_ir_data(void)
//{
//	uint8_t i = 0u;
//	uint8_t j = 0u;
//	uint8_t bit_count = 8u;

//	memset((void*)ir_data_buffer, '\0', 6u);
//	if (IR_status){	
//		if (ir_buff[0] == 9000u){
//			for(i = 2u; i < 50u; i += 2){
//				if (ir_buff[i+1u] == 1690u)
//					ir_data_buffer[j] |= 1u << (bit_count - 1u);
//				bit_count--;
//				if (bit_count == 0){
//					bit_count = 8u;
//					j++;
//					if (j == 3)
//						break;
//				}
//			}
//		}
//		j = 3u;
//		if (ir_buff[50] == 9000u){
//			for(i = 52u; i < 100u; i += 2){
//				if (ir_buff[i+1u] == 1690u)
//					ir_data_buffer[j] |= 1u << (bit_count - 1u);
//				bit_count--;
//				if (bit_count == 0){
//					bit_count = 8u;
//					j++;
//					if (j == 6u)
//						break;
//				}
//			}		
//		}

//		if (ir_buff[0] == 9000u || ir_buff[50] == 9000u){
//			if (ir_data_buffer[0] == 0x81)
//				printf("AT+RES,IR_DATA:%c %x %x %x\r\n",direction, ir_data_buffer[0], ir_data_buffer[1],ir_data_buffer[2]);
//			if (ir_data_buffer[3] == 0x81)
//				printf("AT+RES,IR_DATA:%c %x %x %x\r\n",direction, ir_data_buffer[3], ir_data_buffer[4],ir_data_buffer[5]);
//		}
//		
//		memset((void*)ir_buff, '\0', 105);
//	}
//}

/*!
    \brief      this function handles external lines 5 to 9 interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EXTI5_9_IRQHandler(void)
{
	timer_disable( TIMER8 );
	
	if(exti_interrupt_flag_get(EXTI_6)==SET){
		exti_interrupt_flag_clear(EXTI_6);
		direction = RIGHT;
	}

	if(exti_interrupt_flag_get(EXTI_9)==SET){
		exti_interrupt_flag_clear(EXTI_9);
		direction = LEFT;
	}	
//	if(times > 100)
//		times = 0;
//	if(guide>4)
//		printf("guide = %d   \r\n",guide); 
	cache = guide ;     //核算NEC码的时长
	timer_flag_clear(TIMER8, 1000);
	timer_enable(TIMER8);
//	if(guide>2)
	   cache = 4*guide;
	guide = 0;
	//if(cache > 350 )
//	if(cache > 50 )
		IR_processed();
	cache = 0;
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void l81_IR_timer8_nvic_config(void)
{
    nvic_irq_disable(TIMER8_IRQn);
}
/*!
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
//void l81_delay_timer_config(void)
void l81_IR_timer8_config(void)
{
    /* TIMER8 configuration: input capture mode -------------------
    the external signal is connected to TIMER1 CH0 pin (PA0)
    the rising edge is used as active edge
    the TIMER1 CH0CV is used to compute the frequency value
    ------------------------------------------------------------ */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;

    /* enable the peripherals clock */
    rcu_periph_clock_enable(RCU_TIMER8);

    /* deinit a TIMER */
    timer_deinit(TIMER8);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER8 configuration */
    timer_initpara.prescaler        = 63;//68
    timer_initpara.alignedmode      = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
	//9999_26.4ms,5999_16ms,
    timer_initpara.period           = 3;   //68//51_56us/5_12.98701
    timer_initpara.clockdivision    = TIMER_CKDIV_DIV1;
    timer_init(TIMER8, &timer_initpara);

    // auto-reload preload enable 
    //timer_auto_reload_shadow_enable(TIMER8);
    // clear channel 0 interrupt bit 
    timer_interrupt_flag_clear(TIMER8, TIMER_INT_FLAG_UP);
    // channel 0 interrupt enable /
    timer_interrupt_enable(TIMER8, TIMER_INT_UP);
}


void TIMER8_IRQHandler(void)
{

	if(guide++>99000){
			guide = 0; 
	}
	timer_interrupt_flag_clear(TIMER8, TIMER_INT_FLAG_UP);

}

void l81_IR_init(void)
{
	//l81_IR_gpio_configtest();
	l81_IR_gpio_config();
	key_exti_init();
	l81_IR_key_nvic_config();

	l81_IR_timer8_config();
	l81_IR_timer8_nvic_config();
	
	IR_status = 0u;

}


uint8_t l81_IR_start_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint16_t pulse = 0;
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command hava no params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	if (IR_status == 1u){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,IR start already\r\n");
		printf("AT+RES,end\r\n");
		return 0;
	}
	
	printf("AT+RES,ACK\r\n");
	ir_start();
	IR_status = 1u;
	direction = NONE;
	printf("AT+RES,end\r\n");	
	
  return 1U;		
}

uint8_t l81_IR_stop_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint16_t pulse = 0;
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command hava no params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	if (IR_status == 0u){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,IR stop already\r\n");
		printf("AT+RES,end\r\n");
		return 0;
	}
	
	printf("AT+RES,ACK\r\n");
	ir_stop();
	IR_status = 0u;
	direction = NONE;
	printf("AT+RES,end\r\n");	
	
  return 1U;		
}
