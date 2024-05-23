/*!
    \file    main.c
    \brief   led spark with systick, USART print and key example

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

#include "gd32l23x.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "L81_Motors.h"
#include "L81_inf.h"
#include "L81_AT.h"
#include <string.h>
#include "i2c_inf.h"
#include "qmi8658.h"
#include "L81_cliff.h"
#include "L81_FMC.h"
#include "sths34pf80.h"
#include "L81_factory.h"
#include "L81_IR.h"
#include "aw9310x.h"
#include "L81_TimTask.h"
#include "L81_led_task.h"
#include "L81_sensor_task.h"
#include "L81_MoveAlg.h"
#include "L81_cliff_task.h"
#include "aw9310x.h"
#include "VI530x_User_Handle.h"
#include "L81_humiture_task.h"
#include "qmi8658iic.h"
//#include "gxhtc3iic.h"

#define   USE_WDG_EN        1 //1=EN 0=DEN

void l81_led_gpio_config(void);


void yizima(void)
{
	motor_set_pulse(MOTOR_NUM3,600);
	motor_set_pulse(MOTOR_NUM4,2400);
	delay_1ms(1000);
	motor_set_pulse(MOTOR_NUM3,1500);
	motor_set_pulse(MOTOR_NUM4,1500);	
	delay_1ms(1000);
}

void xie_left(void)
{
	motor_set_pulse(MOTOR_NUM3,1500);
	motor_set_pulse(MOTOR_NUM4,1500);	
	delay_1ms(1000);
	motor_set_pulse(MOTOR_NUM3,1600);
	motor_set_pulse(MOTOR_NUM4,2100);	
	delay_1ms(1000);
}
void xie_right(void)
{
	motor_set_pulse(MOTOR_NUM3,1500);
	motor_set_pulse(MOTOR_NUM4,1500);	
	delay_1ms(1000);
	motor_set_pulse(MOTOR_NUM3,1100);
	motor_set_pulse(MOTOR_NUM4,1400);	
	delay_1ms(1000);
}

void xiebu(void)
{
	uint16_t i;
	    i = 0;
    while (i < 10){
        xie_right();
        i += 1;
		}
    i = 0;
    while (i < 10){
        xie_left();
        i += 1;
		}
}
/*!
    \brief      calibration_motor function, to set motor at 0 degree
    \param[in]  none
    \param[out] none
    \retval     none
*/
void mid_locate_for_factory_motor(void)
{
	motor_set_pulse(MOTOR_NUM1,1500);
	motor_set_pulse(MOTOR_NUM2,1500);
	motor_set_pulse(MOTOR_NUM3,1500);
	motor_set_pulse(MOTOR_NUM4,1500);
	motor_set_pulse(MOTOR_NUM5,1500);
	motor_set_pulse(MOTOR_NUM6,1500);
}

void set_LED()
{
		LED_On_set(LEDS_LEFT, LED_GRB_RED);
		LED_On_set(LEDS_RIGHT, LED_GRB_RED);
		RESET_GRB(GPIO_PIN_0);  // 0621-rjq add
	  RESET_GRB(GPIO_PIN_1);
		RESET_GRB(GPIO_PIN_0);
	  RESET_GRB(GPIO_PIN_1);	
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
  int16_t i = 0, j = 1800, a3 = 1700, a6 = 1400;
	uint16_t k = 0, led = 0;
	static uint16_t adc = 0;
	static uint32_t count = 0;
	
	uint16_t id = 0;
	
  FlagStatus breathe_flag = SET;
	
	//1. set vector table firstly according to boot option, app partition is from 0x0800D000u address
	nvic_vector_table_set(NVIC_VECTTAB_FLASH, (0x0800D000u & 0x000FFFFFu));

	//2. enable irq because the irq was disable in bootloader before jump
//	__enable_irq();
	
	//3. the following is APP
	
	/* configure systick */
  systick_config();
  
#if USE_WDG_EN
{//wxf-add wdg (10s for init) --0424
  fwdgt_config(10*250, FWDGT_PSC_DIV128);
  fwdgt_enable();
}
#endif

	//4us delay timer
	l81_4usdelay_timer_config();
	
	ATcmd_init();
	
  /* configure the GPIO ports */
  l81_led_gpio_config();

	l81_adc_init();
  l81_motor_init();
	
	l81_usart_init();
	
	sw_i2c_gpio_config(); //i2c0 gpio config
	i2c_config(I2C1, RCU_I2C1);
	i2c_config(I2C2, RCU_I2C2);
	
	L81_cliff_init();
	
	/* enable IRC32K */
	// rcu_osci_on(RCU_IRC32K);
	/* wait till IRC32K is ready */
 // while(SUCCESS != rcu_osci_stab_wait(RCU_IRC32K)) {  }
	 
	QMI_iic_init();
	if(!qmi8658_init()){
			printf("qmi 8658 init failed\n");
	}else{
		delay_1ms(300);
		printf("qmi 8658 init OK\n");
	}		
	
	printf("MCU booting successfully\r\n");
	printf("Firmware ReleaseDate: %d%d\r\n", DATE_INT,TIME_INT);
 
	
	l81_IR_init();
	VI530x_init();
  
  l81_tim_task_init(); //time task init 
  led_task_init();
  l81_move_init();    //
  sensor_task_init();
  cliff_task_init();
  
  algo_task_init(); //wxf-add 0427
	
	humiture_task_init(); //rjq-add 0505

//	set_LED();  // rjq-add 0621
	
	int	ret = (int)aw9310x_init(&aw9310x_demo_func);
	if(ret < 0) {
		printf("some func has not been initalized.");
	}

	
	printf("%s\n", l81_version);

#if USE_WDG_EN
{//wxf-add wdg (3s for run) --0424
  fwdgt_counter_reload();
  fwdgt_config(4*250, FWDGT_PSC_DIV128);//ns=n*250
}
#endif

  while(1) {
		//feed wdg
#if USE_WDG_EN
    fwdgt_counter_reload();
#endif
    
		//only used in factory
//		mid_locate_for_factory_motor();
		
		//monitor the system health
//		motor_write_pulse(test_pulse);
		
		/*check cliff danger status*/
		l81_cliff_danger_status();
//		get_ir_data();

		//check rx buffer status
		if(rx_state == READY) {
          rx_state = IDLE;
		//begin execute AT server
			ATcmd_server();
		//reset com_rxbuffer for next recieving
			 memset((void *)com_rxbuffer, '\0', TRANSFER_NUM);
			
		}
				#if 0 //move to led task
        /* delay a time in milliseconds */
        delay_1ms(10);
        if(SET == breathe_flag) {
            i = i + 100;
        } else {
            i = i - 100;
        }
        if((2500 < i)) {
            breathe_flag = RESET;
						gpio_bit_toggle(GPIOB, GPIO_PIN_6);
						gpio_bit_toggle(GPIOD, GPIO_PIN_4);
				}
        if(0 >= i) {
            breathe_flag = SET;
        }
        #endif
        
        l81_tim_task_serve();//time task
//#if USE_WDG_EN
//    fwdgt_counter_reload();
//#endif
				//KeyNumoutToSOC();
//				KeyNumoutToSOC((void *)USE_WDG_EN);
				
    }
}

/*!
    \brief      l81_led_gpio_config function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void l81_led_gpio_config(void)
{
	/* enable the GPIO clock */
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOD);
	
	/* configure LED GPIO pin */
  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
  gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
  gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
	
	/* reset LED GPIO pin */
  gpio_bit_reset(GPIOB, GPIO_PIN_6);
  gpio_bit_reset(GPIOD, GPIO_PIN_4);
	
		/* enable the Motor control  GPIO clock */
	rcu_periph_clock_enable(RCU_GPIOD);
	/* configure Motor control GPIO pin */
	gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
	gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
	gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_1);
	gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
	
	gpio_bit_reset(GPIOD, GPIO_PIN_0);
	gpio_bit_reset(GPIOD, GPIO_PIN_1);
	
	//test pin for motor or other modules
	rcu_periph_clock_enable(RCU_GPIOC);
	gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_10);
  gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{

  usart_data_transmit(USART1, (uint8_t) ch);
 	while(RESET == usart_flag_get(USART1, USART_FLAG_TBE)){}
  return ch;
}
