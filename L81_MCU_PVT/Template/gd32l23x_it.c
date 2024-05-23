/*!
    \file    gd32l23x_it.c
    \brief   interrupt service routines

    \version 2021-08-04, V1.0.0, firmware for GD32L23x
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

#include "gd32l23x_it.h"
#include "main.h"
#include "systick.h"
#include "L81_inf.h"
#include "L81_cliff.h"
#include "aw9310x.h"
#include "L81_TimTask.h"

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_decrement();
}

#include <string.h>
void USART1_IRQHandler(void)
{
	if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)){ 

		uint8_t ch = (uint8_t)usart_data_receive(USART1);
		
		if(rx_count < TRANSFER_NUM){
			com_rxbuffer[rx_count++] = ch;
		}
		
		usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);
		
		if( SET==usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE_ORERR)|| \
		 SET==usart_interrupt_flag_get(USART1, USART_INT_FLAG_PERR) || \
		 SET==usart_interrupt_flag_get(USART1, USART_INT_FLAG_ERR_ORERR) || \
	   SET==usart_interrupt_flag_get(USART1, USART_INT_FLAG_ERR_NERR) || \
		 SET==usart_interrupt_flag_get(USART1, USART_INT_FLAG_ERR_FERR)){
			 
			usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE_ORERR);
			 usart_interrupt_flag_clear(USART1, USART_INT_FLAG_PERR);
			 usart_interrupt_flag_clear(USART1, USART_INT_FLAG_ERR_ORERR);
			 usart_interrupt_flag_clear(USART1, USART_INT_FLAG_ERR_NERR);
			 usart_interrupt_flag_clear(USART1, USART_INT_FLAG_ERR_FERR);
			 
			usart_receive_config(USART1, USART_RECEIVE_DISABLE);
			usart_interrupt_disable(USART1, USART_INT_RBNE);
			usart_interrupt_disable(USART1, USART_INT_IDLE);
			usart_interrupt_disable(USART1, USART_INT_ERR);
			usart_interrupt_disable(USART1, USART_INT_PERR);
			 	
		  usart_receive_config(USART1, USART_RECEIVE_ENABLE);
			usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);
		  usart_interrupt_enable(USART1, USART_INT_RBNE);
		  usart_interrupt_enable(USART1, USART_INT_IDLE);
	    usart_interrupt_enable(USART1, USART_INT_ERR);
	    usart_interrupt_enable(USART1, USART_INT_PERR);
			 
			
		  rx_len = rx_count;
		  rx_count = 0;
			rx_state = IDLE;
		}
		rx_state = BUSY;
	}

		if(SET==usart_interrupt_flag_get(USART1, USART_INT_FLAG_IDLE)){
			
			
		 if(SET==usart_interrupt_flag_get(USART1, USART_INT_FLAG_PERR) || \
		 SET==usart_interrupt_flag_get(USART1, USART_INT_FLAG_ERR_ORERR) || \
	   SET==usart_interrupt_flag_get(USART1, USART_INT_FLAG_ERR_NERR) || \
		 SET==usart_interrupt_flag_get(USART1, USART_INT_FLAG_ERR_FERR)){
			 			 
		 usart_interrupt_flag_clear(USART1, USART_INT_FLAG_IDLE);
			 
		 usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE_ORERR);
		 usart_interrupt_flag_clear(USART1, USART_INT_FLAG_PERR);
		 usart_interrupt_flag_clear(USART1, USART_INT_FLAG_ERR_ORERR);
		 usart_interrupt_flag_clear(USART1, USART_INT_FLAG_ERR_NERR);
			usart_interrupt_flag_clear(USART1, USART_INT_FLAG_ERR_FERR);
		 
  		usart_receive_config(USART1, USART_RECEIVE_DISABLE);			
			usart_interrupt_disable(USART1, USART_INT_RBNE);
			usart_interrupt_disable(USART1, USART_INT_IDLE);
			usart_interrupt_disable(USART1, USART_INT_ERR);
			usart_interrupt_disable(USART1, USART_INT_PERR);
			 
			 
			rx_len = rx_count;
			rx_count = 0;
		  
		  usart_receive_config(USART1, USART_RECEIVE_ENABLE);
		  usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);
		  usart_interrupt_enable(USART1, USART_INT_RBNE);
		  usart_interrupt_enable(USART1, USART_INT_IDLE);
	    usart_interrupt_enable(USART1, USART_INT_ERR);
	    usart_interrupt_enable(USART1, USART_INT_PERR);		 

		 rx_state = IDLE;
	 }
		 
	 usart_interrupt_flag_clear(USART1, USART_INT_FLAG_IDLE);
			rx_len = rx_count;
			rx_count = 0;	 
	 rx_state = IDLE;
   rx_state = READY;
 }
}

/*!
    \brief      this function handles DMA_Channel0_IRQHandler interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DMA_Channel0_IRQHandler(void)
{

}

/*!
    \brief      this function handles DMA_Channel1_IRQHandler interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/

void DMA_Channel1_IRQHandler(void)
{

}

/*!
    \brief      this function handles TIMER1 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER1_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP)) {
      /* clear channel 0 interrupt bit */
      timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
		
			if (!l81_cliff_getlock()) {
			/*set cliff EN status*/
			gpio_bit_write(GPIOB, GPIO_PIN_1 | GPIO_PIN_2, (uint8_t)cliff_OnOff);
					
			/*revert cliff EN status*/
			if (cliff_OnOff == cliff_On) {
				cliff_status = cliff_run;
				cliff_OnOff = cliff_Off;
			}	else {
				cliff_status = cliff_idle;
				cliff_OnOff = cliff_On;
			}
		}
		}
}

/*!
    \brief      this function handles EXTI0 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*//*
void EXTI10_15_IRQHandler(void)
{
    if(RESET != exti_interrupt_flag_get(EXTI_12)) {
        exti_interrupt_flag_clear(EXTI_12);
			aw9310x_irq_cb();
//        gd_eval_led_toggle(LED1);
    }
}
*/
#if 0
/*!
    \brief      this function handles TIMER5 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER2_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP)) {
      /* clear channel 0 interrupt bit */
      timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
			aw9310x_press_timer_cb();
		}
}

/*!
    \brief      this function handles TIMER6 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
#endif
//todo is single double,three,long click
void TIMER6_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER6, TIMER_INT_FLAG_UP)) {
      /* clear channel 0 interrupt bit */
      timer_interrupt_flag_clear(TIMER6, TIMER_INT_FLAG_UP);
			//aw9310x_click_timer_cb();    //0714  rjq --
			//clickType++;
      
      tim_call();//for tim task
		}
}
