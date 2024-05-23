/*!
    \file    L81_factory.c
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
#include "main.h"
#include <string.h>
#include "L81_inf.h"
#include "L81_AT.h"
#include "i2c_inf.h"
#include "L81_factory.h"
#include "aw9310x.h"
#include "stk3311iic.h"
#include "sths34pf80_iic.h"
#include "L81_Motors.h"

uint8_t l81_AT_Light_id_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint8_t id = 0;
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
//	i2c_buffer_read(I2C2, 0x48 << 1, 0x3E, &id, 1u);
  STK_ReadReg_1(0x3E, &id);
	if(id == 0x12){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,0x%x\r\n", id);   //device id is 0x12
		printf("AT+RES,end\r\n");
	}else{
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,didn't get device id\r\n");  
		printf("AT+RES,end\r\n");		
	}
    return 1U;		
}

uint8_t l81_AT_Tmos_id_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint8_t id = 0;
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");   
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
//	i2c_buffer_read(I2C2, 0x5a << 1, 0x0F, &id, 1u);
  STHS_ReadReg_1(0x0F, &id);  //read reg
	if(id == 0xd3){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,0x%x\r\n", id);   //device is d3
		printf("AT+RES,end\r\n");
	}else{
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,didn't get device id\r\n");
		printf("AT+RES,end\r\n");		
	}
    return 1U;		
}

uint8_t l81_AT_Tof_id_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint8_t id = 0;
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	sw_i2c_send2read_8bit(0x6c, 0x06, &id, 1);
	if(id == 0xd8){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,0x%x\r\n", id);   //id should be 0xD8
		printf("AT+RES,end\r\n");
	}else{
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,didn't get device id\r\n"); 
		printf("AT+RES,end\r\n");		
	}
    return 1U;		
}

uint8_t l81_AT_Touch_id_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint32_t id = 0;
  uint8_t tmp[4];	//read 4 byte one time
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	sw_i2c_send2read_16bit(0x12, 0xff10, tmp, 4u);
	for (uint8_t i = 0; i < 4; i++)
	{
		id = (id << 8) | tmp[i];
		
	}
	
	if(id == 0xa9610b00){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,0x%x\r\n", id);   //device id is 0xa9610b00
		printf("AT+RES,end\r\n");
	}else{
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,didn't get device id\r\n");
		printf("AT+RES,end\r\n");		
	}
    return 1U;		
}

/*------------------------------------only be used in factory for temperature sensor----------------------------------*/


void sw_i2c1_gpio_config(void);
void sw_i2c1_send2read_8bit(uint8_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len);
void sw_i2c1_send2read_16bit(uint8_t slave_addr, uint16_t reg, uint8_t *buf, uint8_t len);

uint8_t l81_AT_AG_test_id_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint8_t id = 0;
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	sw_i2c1_gpio_config();
	sw_i2c1_send2read_8bit(0x6b, 0x0, &id, 1);
	if(id == 0x05){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,0x%x\r\n", id);   //id should be 0x05
		printf("AT+RES,end\r\n");
	}else{
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,didn't get device id\r\n"); 
		printf("AT+RES,end\r\n");		
	}

    return 1U;		
}

uint8_t l81_AT_TH_id_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint32_t id = 0;
  uint8_t tmp[3];	//read 3 byte one time
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	sw_i2c_send2read_16bit(0x70, 0xEFC8, tmp, 3u);
	for (uint8_t i = 0; i < 3; i++)
	{
		id = (id << 8) | tmp[i];
		
	}
	
	if(id != 0u){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,0x%x\r\n", id);  
		printf("AT+RES,end\r\n");
	}else{
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,didn't get device id\r\n");
		printf("AT+RES,end\r\n");		
	}
    return 1U;	
}


void sw_i2c1_gpio_config(void)
{
	rcu_periph_clock_enable(RCU_GPIOB);
	gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_10 | GPIO_PIN_11);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_11);
	
	gpio_bit_set(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
}

#define sw_i2c1_clk_1()  (GPIO_BOP(GPIOB) = GPIO_PIN_10)
#define sw_i2c1_clk_0()  (GPIO_BC(GPIOB)  = GPIO_PIN_10)
#define sw_i2c1_sda_1()  (GPIO_BOP(GPIOB) = GPIO_PIN_11)
#define sw_i2c1_sda_0()  (GPIO_BC(GPIOB)  = GPIO_PIN_11)

#define sw_i2c1_clk_read() (GPIO_ISTAT(GPIOB) & (GPIO_PIN_10))
#define sw_i2c1_sda_read() (GPIO_ISTAT(GPIOB) & (GPIO_PIN_11))

void sw_i2c1_set_sda_output(void)
{
	gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_11);
}

void sw_i2c1_set_sda_input(void)
{
	gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_11);
}

#define sw_i2c1_sda_input()    sw_i2c1_set_sda_input()
#define sw_i2c1_sda_output()   sw_i2c1_set_sda_output()

void sw_i2c1_delay(void)
{
	uint8_t count = 4;  //110khz
	for(;count > 0;count--){}
}

void sw_i2c1_start(void)
{
	sw_i2c1_sda_1();
	sw_i2c1_clk_1();
	sw_i2c1_delay();
	sw_i2c1_sda_0();
	sw_i2c1_delay();
	sw_i2c1_clk_0();
	sw_i2c1_delay();
}

void sw_i2c1_stop(void)
{
	sw_i2c1_sda_0();
	sw_i2c1_clk_1();
	sw_i2c1_delay();
	sw_i2c1_sda_1();	
	sw_i2c1_delay();
}

void sw_i2c1_ack(void)
{
	sw_i2c1_sda_0();
	sw_i2c1_delay();
	sw_i2c1_clk_1();
	sw_i2c1_delay();
	sw_i2c1_clk_0();
	sw_i2c1_delay();
	sw_i2c1_clk_1();
}

void sw_i2c1_nack(void)
{
	sw_i2c1_sda_1();
	sw_i2c1_delay();
	sw_i2c1_clk_1();
	sw_i2c1_delay();
	sw_i2c1_clk_0();
	sw_i2c1_delay();
}

uint8_t sw_i2c1_wait_ack(void)
{
	uint8_t sda_status;
	uint8_t wait_time = 0u;
	uint8_t ack_nack = 1;
	
	sw_i2c1_sda_input();
	
	while(sw_i2c1_sda_read())
	{
		wait_time++;
		
		if (wait_time >= 200){
			ack_nack = 0;
			break;
		}
	}
	
	sw_i2c1_delay();
	sw_i2c1_clk_1();
	sw_i2c1_delay();
	
	sw_i2c1_clk_0();
	sw_i2c1_delay();
	sw_i2c1_sda_output();
	sw_i2c1_delay();
	
	return ack_nack;
}

void sw_i2c1_writeByte(uint8_t data)
{
	uint8_t i;
	
	for(i = 0u; i < 8u; i++){
		sw_i2c1_clk_0();
		sw_i2c1_delay();
		if(data & 0x80u)
			sw_i2c1_sda_1();
		else
			sw_i2c1_sda_0();
		
		sw_i2c1_delay();
		sw_i2c1_clk_1();
		sw_i2c1_delay();
		data <<= 1;
	}
	
	sw_i2c1_clk_0();
	sw_i2c1_delay();
}

uint8_t sw_i2c1_readByte(void)
{
	uint8_t i = 0u, data = 0u;
	
	sw_i2c1_sda_input();
	for(i = 0u; i < 8; i++){
		data <<= 1;
		sw_i2c1_delay();
		
		sw_i2c1_clk_1();
		sw_i2c1_delay();
		
		if (sw_i2c1_sda_read())
			data |= 0x01;
		
		sw_i2c1_clk_0();
	}
	
	sw_i2c1_sda_output();
	return data;
}


void sw_i2c1_write_nBytes(uint8_t slave_addr, uint8_t *data, uint8_t len)
{
	uint8_t j;
	
	//7bit address should left shifft 1 bit
	slave_addr <<= 1;
	
	sw_i2c1_start();
	
	sw_i2c1_writeByte(slave_addr);
	
	if (!sw_i2c1_wait_ack())
		goto err;
	
	for (j = 0u; j < len; j++){
		sw_i2c1_writeByte(*(data + j));
		if (!sw_i2c1_wait_ack())
			goto err;		
	}
	
	err:
	sw_i2c1_stop();
}

void sw_i2c1_read_nBytes(uint8_t slave_addr, uint8_t *buf, uint8_t len)
{
	uint8_t j;
	
	slave_addr <<= 1;
	
	sw_i2c1_start();
	sw_i2c1_writeByte(slave_addr | 0x01);
	
	if(!sw_i2c1_wait_ack())
		goto err;
	
	for(j = 0; j < len; j++){
		buf[j] = sw_i2c1_readByte();
		sw_i2c1_ack();
	}
	
	err:
	sw_i2c1_stop();
}

void sw_i2c1_send2read_8bit(uint8_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t j;
	
	slave_addr <<= 1;
	
	sw_i2c1_start();
	
	sw_i2c1_writeByte(slave_addr);
	if (!sw_i2c1_wait_ack())
		goto err;
	
	sw_i2c1_writeByte(reg);
	if (!sw_i2c1_wait_ack())
		goto err;
	
	sw_i2c1_start();
	
	sw_i2c1_writeByte(slave_addr | 0x01);
	if (!sw_i2c1_wait_ack())
		goto err;	
	
	for (j = 0; j < len; j++){
		buf[j] = sw_i2c1_readByte();
		sw_i2c1_ack();
	}
	
	err:
	sw_i2c1_stop();
}

void sw_i2c1_send2read_16bit(uint8_t slave_addr, uint16_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t j;
	
	slave_addr <<= 1;
	
	sw_i2c1_start();
	
	sw_i2c1_writeByte(slave_addr);
	if (!sw_i2c1_wait_ack())
		goto err;
	
	sw_i2c1_writeByte((reg >> 8u) & 0xFFu);
	if (!sw_i2c1_wait_ack())
		goto err;

	sw_i2c1_writeByte(reg & 0xFFu);
	if (!sw_i2c1_wait_ack())
		goto err;	
	
	sw_i2c1_start();
	
	sw_i2c1_writeByte(slave_addr | 0x01);
	if (!sw_i2c1_wait_ack())
		goto err;	
	
	for (j = 0; j < len; j++){
		buf[j] = sw_i2c1_readByte();
		sw_i2c1_ack();
	}
	
	err:
	sw_i2c1_stop();
}

/*-------------------------------motor calibration-----------------------------------*/

volatile static uint64_t rissing_trigger = 0u;
volatile static uint64_t falling_trigger = 0u;
volatile uint64_t angle_pulse = 0u;  //return value  is (angle_pulse * 4)us

void motor_irq_init(void)
{
    /* enable the SYSCFG clock */
    rcu_periph_clock_enable(RCU_SYSCFG);

    /* connect key EXTI line to key GPIO pin */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN10);

    /* configure key EXTI line */
    exti_init(EXTI_10, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_interrupt_flag_clear(EXTI_10);
	
	   /* enable and set key EXTI interrupt to the specified priority */
    nvic_irq_enable(EXTI10_15_IRQn, 1U);
}


void EXTI10_15_IRQHandler(void)
{
    if(RESET != exti_interrupt_flag_get(EXTI_10)) {
        exti_interrupt_flag_clear(EXTI_10);

			if (GPIO_ISTAT(GPIOC) & (GPIO_PIN_10)){
				rissing_trigger = get_timer();
			}	else if (!(GPIO_ISTAT(GPIOC) & (GPIO_PIN_10))){
				falling_trigger = get_timer();
				angle_pulse = falling_trigger - rissing_trigger;
			}
    }else if(RESET != exti_interrupt_flag_get(EXTI_12)) {
        exti_interrupt_flag_clear(EXTI_12);
			aw9310x_irq_cb();			
      	exti_interrupt_flag_clear(EXTI_12);
			//printf("EXTI10_15_IRQHandler\r\n");  //0714 rjq+

			//nvic_irq_disable(EXTI10_15_IRQn);
//        gd_eval_led_toggle(LED1);
    }
}

void motor_send_readcmd(void)
{
	  angle_pulse = 0;  //reset angle pulse
	
	  GPIO_BC(GPIOC) = GPIO_PIN_10;
		delay_4us(24);  //delay 100us  make sure low 
		GPIO_BOP(GPIOC) = GPIO_PIN_10;
		delay_4us(24);  //delay 100us to send 100us high voltage signal
		GPIO_BC(GPIOC) = GPIO_PIN_10;
	
		gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_10);
	
	  motor_irq_init();
}

/*after get angle value should call this function*/
void motor_end_readcmd(void)
{
	nvic_irq_disable(EXTI10_15_IRQn);
	gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_10);
}

void motor_unlock_force(void)
{
	for(uint8_t i = 0; i < 3; i++){  //unlock single need send 3 times
		
		GPIO_BC(GPIOC) = GPIO_PIN_10;
	  delay_4us(24);  //delay 100us  make sure low 
		
		GPIO_BC(GPIOC) = GPIO_PIN_10;
		delay_4us(24);  //delay 100us  make sure low 
		GPIO_BOP(GPIOC) = GPIO_PIN_10;
		delay_4us(687);  //delay 2750us to send 2750us high voltage signal
		GPIO_BC(GPIOC) = GPIO_PIN_10;
		
	}
}

void motor_store_mid_offset(void)
{
	uint8_t i = 0;
	
	GPIO_BC(GPIOC) = GPIO_PIN_10;
	delay_4us(24);  //delay 100us  make sure low 
	
	for(i = 0; i < 3; i++){
		GPIO_BOP(GPIOC) = GPIO_PIN_10;
		delay_4us(865);  //delay 3400us to send 3400us high voltage signal
		GPIO_BC(GPIOC) = GPIO_PIN_10;
		delay_4us(5000 - 865); //20ms - 3.4ms to make sure 20ms period
	}

	for(i = 0; i < 3; i++){
		GPIO_BOP(GPIOC) = GPIO_PIN_10;
		delay_4us(1050);  //delay 4200us to send 4200us high voltage signal
		GPIO_BC(GPIOC) = GPIO_PIN_10;
		delay_4us(5000 - 1050); //20ms - 4.2ms to make sure 20ms period
	}
	

}

void motor_store_500_1500(void)
{
	uint8_t i = 0;
	
	GPIO_BC(GPIOC) = GPIO_PIN_10;
	delay_4us(24);  //delay 100us  make sure low 

	for(i = 0; i < 4; i++){
		GPIO_BOP(GPIOC) = GPIO_PIN_10;
		delay_4us(1150);  //delay 4600us to send 4600us high voltage signal
		GPIO_BC(GPIOC) = GPIO_PIN_10;
		delay_4us(5000 - 1150); //20ms - 4.6ms to make sure 20ms period
			
	}

	
	for(i = 0; i < 4; i++){
		GPIO_BOP(GPIOC) = GPIO_PIN_10;
		delay_4us(1050);  //delay 4200us to send 4200us high voltage signal
		GPIO_BC(GPIOC) = GPIO_PIN_10;
		delay_4us(5000 - 1050); //20ms - 4.2ms to make sure 20ms period
	}	

}

void motor_store_1500_2500(void)
{
	uint8_t i = 0;
	
	GPIO_BC(GPIOC) = GPIO_PIN_10;
	delay_4us(24);  //delay 100us  make sure low 
	
	for(i = 0; i < 4; i++){
		GPIO_BOP(GPIOC) = GPIO_PIN_10;
		delay_4us(1250);  //delay 5000us to send 5000us high voltage signal
		GPIO_BC(GPIOC) = GPIO_PIN_10;
		delay_4us(5000 - 1250); //20ms - 5ms to make sure 20ms period
	}

	for(i = 0; i < 4; i++){
		GPIO_BOP(GPIOC) = GPIO_PIN_10;
		delay_4us(1050);  //delay 4200us to send 4200us high voltage signal
		GPIO_BC(GPIOC) = GPIO_PIN_10;
		delay_4us(5000 - 1050); //20ms - 4.2ms to make sure 20ms period
	}	
}

void motor_reset(void)
{
	uint8_t i = 0;
	
	GPIO_BC(GPIOC) = GPIO_PIN_10;
	delay_4us(24);  //delay 100us  make sure low 
	
	for(i = 0; i < 4; i++){
		GPIO_BOP(GPIOC) = GPIO_PIN_10;
		delay_4us(950);  //delay 3800us to send 3800us high voltage signal
		GPIO_BC(GPIOC) = GPIO_PIN_10;
		delay_4us(5000 - 950); //20ms - 3.8ms to make sure 20ms period
	}

	for(i = 0; i < 4; i++){
		GPIO_BOP(GPIOC) = GPIO_PIN_10;
		delay_4us(1050);  //delay 4200us to send 4200us high voltage signal
		GPIO_BC(GPIOC) = GPIO_PIN_10;
		delay_4us(5000 - 1050); //20ms - 4.2ms to make sure 20ms period
	}	
}

void motor_write_pulse(uint16_t pulse)
{
	uint8_t i = 0;
	uint16_t delay_time = pulse / 4;
	
	GPIO_BC(GPIOC) = GPIO_PIN_10;
	delay_4us(24);  //delay 100us  make sure low 
	
	for(i = 0; i < 15; i++){
		GPIO_BOP(GPIOC) = GPIO_PIN_10;
		delay_4us(delay_time);  //high level delay time
		GPIO_BC(GPIOC) = GPIO_PIN_10;
		delay_4us(5000 - delay_time); //20ms - 3.8ms to make sure 20ms period
	}
}

uint16_t test_pulse = 0u;
uint8_t l81_AT_set_pulse_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint16_t pulse = 0;
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 1U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command hava 1 params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	pulse = String2Int(param[0]);
	
	motor_write_pulse(pulse);

	printf("AT+RES,ACK\r\n");
	printf("AT+RES,end\r\n");
	
    return 1U;		
}

uint8_t l81_AT_motor_reset_func(char params[])
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
	
	motor_reset();
	
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,end\r\n");
	
  return 1U;		
}

uint8_t l81_AT_lowstore_func(char params[])
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
	
	motor_store_500_1500();

	printf("AT+RES,ACK\r\n");
	printf("AT+RES,end\r\n");
	
  return 1U;		
}

uint8_t l81_AT_highstore_func(char params[])
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
	
	motor_store_1500_2500();

	printf("AT+RES,ACK\r\n");
	printf("AT+RES,end\r\n");
	
  return 1U;		
}

uint8_t l81_AT_midstore_func(char params[])
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
	
	motor_store_mid_offset();

	printf("AT+RES,ACK\r\n");
	printf("AT+RES,end\r\n");
#if 0	
	motor_send_readcmd();
	while(!angle_pulse){}
	if(angle_pulse){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,mid pulse %d\n", (uint32_t)angle_pulse);
		printf("AT+RES,end\r\n");
	}	else{
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,regulater mid angle error\r\n");
		printf("AT+RES,end\r\n");		
	}
	motor_end_readcmd();
#endif
  return 1U;		
}

uint8_t l81_AT_unlock_func(char params[])
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

	motor_unlock_force();
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,end\r\n");
	
  return 1U;		
}

uint8_t l81_AT_read_func(char params[])
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
	
	motor_send_readcmd();
	while(!angle_pulse){}
	if(angle_pulse){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,pulse %d\n", GET_ANGLE);
		printf("AT+RES,end\r\n");
	}	else{
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,didn't get angle pulse\r\n");
		printf("AT+RES,end\r\n");		
	}
	motor_end_readcmd();
	
  return 1U;		
}


/**for body motor calibration-start ****/
//add for body motor calibration 0325 
void four_motor_gpio_reinit(void)
{
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOC);

  gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_14 | GPIO_PIN_15);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14 | GPIO_PIN_15);
  
//  gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
//  gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
  gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_7);
  gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
  
}

void single_motor_unlock(uint32_t gpio_periph, uint32_t pin)
{
  for(uint8_t i = 0; i < 3; i++){  //unlock single need send 3 times

    GPIO_BC(gpio_periph) = pin;
    delay_4us(24);  //delay 100us  make sure low 

    GPIO_BC(gpio_periph) = pin;
    delay_4us(24);  //delay 100us  make sure low 
    GPIO_BOP(gpio_periph) = pin;
    delay_4us(687);  //delay 2750us to send 2750us high voltage signal
    GPIO_BC(gpio_periph) = pin;
  }
}

void single_motor_store_mid_offset(uint32_t gpio_periph, uint32_t pin)
{
  uint8_t i = 0;

  GPIO_BC(gpio_periph) = pin;
  delay_4us(24);  //delay 100us  make sure low 

  for(i = 0; i < 3; i++){
    GPIO_BOP(gpio_periph) = pin;
    delay_4us(865);  //delay 3400us to send 3400us high voltage signal
    GPIO_BC(gpio_periph) = pin;
    delay_4us(5000 - 865); //20ms - 3.4ms to make sure 20ms period
  }

  for(i = 0; i < 3; i++){
    GPIO_BOP(gpio_periph) = pin;
    delay_4us(1050);  //delay 4200us to send 4200us high voltage signal
    GPIO_BC(gpio_periph) = pin;
    delay_4us(5000 - 1050); //20ms - 4.2ms to make sure 20ms period
  }        
}

void single_motor_store_500_1500(uint32_t gpio_periph, uint32_t pin)
{
  uint8_t i = 0;

  GPIO_BC(gpio_periph) = pin;
  delay_4us(24);  //delay 100us  make sure low 

  for(i = 0; i < 4; i++){
    GPIO_BOP(gpio_periph) = pin;
    delay_4us(1150);  //delay 4600us to send 4600us high voltage signal
    GPIO_BC(gpio_periph) = pin;
    delay_4us(5000 - 1150); //20ms - 4.6ms to make sure 20ms period    
  }

  for(i = 0; i < 4; i++){
    GPIO_BOP(gpio_periph) = pin;
    delay_4us(1050);  //delay 4200us to send 4200us high voltage signal
    GPIO_BC(gpio_periph) = pin;
    delay_4us(5000 - 1050); //20ms - 4.2ms to make sure 20ms period
  }                
}

void single_motor_store_1500_2500(uint32_t gpio_periph, uint32_t pin)
{
  uint8_t i = 0;

  GPIO_BC(gpio_periph) = pin;
  delay_4us(24);  //delay 100us  make sure low 

  for(i = 0; i < 4; i++){
    GPIO_BOP(gpio_periph) = pin;
    delay_4us(1250);  //delay 5000us to send 5000us high voltage signal
    GPIO_BC(gpio_periph) = pin;
    delay_4us(5000 - 1250); //20ms - 5ms to make sure 20ms period
  }

  for(i = 0; i < 4; i++){
    GPIO_BOP(gpio_periph) = pin;
    delay_4us(1050);  //delay 4200us to send 4200us high voltage signal
    GPIO_BC(gpio_periph) = pin;
    delay_4us(5000 - 1050); //20ms - 4.2ms to make sure 20ms period
  }                
}

void four_motor_unlock_force(void)
{
  single_motor_unlock(GPIOC, GPIO_PIN_6);
  single_motor_unlock(GPIOC, GPIO_PIN_7);
  single_motor_unlock(GPIOB, GPIO_PIN_14);
  single_motor_unlock(GPIOB, GPIO_PIN_15);
}

void four_motor_store_mid_offset(void)
{
  single_motor_store_mid_offset(GPIOC, GPIO_PIN_6);
  single_motor_store_mid_offset(GPIOC, GPIO_PIN_7);
  single_motor_store_mid_offset(GPIOB, GPIO_PIN_14);
  single_motor_store_mid_offset(GPIOB, GPIO_PIN_15);
}



void four_motor_store_500_1500(void)
{
  single_motor_store_500_1500(GPIOC, GPIO_PIN_6);
  single_motor_store_500_1500(GPIOC, GPIO_PIN_7);
  single_motor_store_500_1500(GPIOB, GPIO_PIN_14);
  single_motor_store_500_1500(GPIOB, GPIO_PIN_15);
}

void four_motor_store_1500_2500(void)
{
  single_motor_store_1500_2500(GPIOC, GPIO_PIN_6);
  single_motor_store_1500_2500(GPIOC, GPIO_PIN_7);
  single_motor_store_1500_2500(GPIOB, GPIO_PIN_14);
  single_motor_store_1500_2500(GPIOB, GPIO_PIN_15);
}

void motor1_write_pulse(uint16_t pulse)
{
  uint8_t i = 0;
  uint16_t delay_time = pulse / 4;

  GPIO_BOP(GPIOB) = GPIO_PIN_14;
  delay_4us(24);  //delay 100us  make sure low 

  for(i = 0; i < 15; i++){
    GPIO_BC(GPIOB) = GPIO_PIN_14;
    delay_4us(delay_time);  //high level delay time
    GPIO_BOP(GPIOB) = GPIO_PIN_14;
    delay_4us(5000 - delay_time); //20ms - 3.8ms to make sure 20ms period
  }
}
void motor2_write_pulse(uint16_t pulse)
{
  uint8_t i = 0;
  uint16_t delay_time = pulse / 4;

  GPIO_BOP(GPIOB) = GPIO_PIN_15;
  delay_4us(24);  //delay 100us  make sure low 

  for(i = 0; i < 15; i++){
    GPIO_BC(GPIOB) = GPIO_PIN_15;
    delay_4us(delay_time);  //high level delay time
    GPIO_BOP(GPIOB) = GPIO_PIN_15;
    delay_4us(5000 - delay_time); //20ms - 3.8ms to make sure 20ms period
  }
}
void motor3_write_pulse(uint16_t pulse)
{
  uint8_t i = 0;
  uint16_t delay_time = pulse / 4;

  GPIO_BOP(GPIOC) = GPIO_PIN_5;
  delay_4us(24);  //delay 100us  make sure low 

  for(i = 0; i < 15; i++){
    GPIO_BC(GPIOC) = GPIO_PIN_5;
    delay_4us(delay_time);  //high level delay time
    GPIO_BOP(GPIOC) = GPIO_PIN_5;
    delay_4us(5000 - delay_time); //20ms - 3.8ms to make sure 20ms period
  }
}
void motor4_write_pulse(uint16_t pulse)
{
  uint8_t i = 0;
  uint16_t delay_time = pulse / 4;

  GPIO_BOP(GPIOC) = GPIO_PIN_7;
  delay_4us(24);  //delay 100us  make sure low 

  for(i = 0; i < 15; i++){
    GPIO_BC(GPIOC) = GPIO_PIN_7;
    delay_4us(delay_time);  //high level delay time
    GPIO_BOP(GPIOC) = GPIO_PIN_7;
    delay_4us(5000 - delay_time); //20ms - 3.8ms to make sure 20ms period
  }
}

uint8_t l81_AT_pre_calibration_func(char params[])
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

  four_motor_gpio_reinit();

  four_motor_unlock_force();
  printf("AT+RES,ACK\r\n");
  printf("AT+RES,end\r\n");

  return 1U;                
}

uint8_t l81_AT_save_90calibration_func(char params[])
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

  four_motor_store_mid_offset();
  
  printf("AT+RES,ACK\r\n");
  printf("AT+RES,end\r\n");
  l81_motor_poweroff();
  delay_1ms(10);
  l81_motor_init();
  

  return 1U;                
}

uint8_t l81_AT_save_0calibration_func(char params[])
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

  four_motor_store_500_1500();
  l81_motor_init();
  printf("AT+RES,ACK\r\n");
  printf("AT+RES,end\r\n");

  return 1U;                
}

uint8_t l81_AT_save_180calibration_func(char params[])
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

  four_motor_store_1500_2500();
  l81_motor_init();
  printf("AT+RES,ACK\r\n");
  printf("AT+RES,end\r\n");
        
  return 1U;                
}

/**for body motor calibration-end ****/