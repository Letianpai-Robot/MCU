/*!
    \file    L81_inf.c
    \brief   the basical interface for drivers and applications,eg. ADC, uart, I2C and so on.
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
#include "L81_Motors.h"
#include "L81_inf.h"
#include "L81_AT.h"

/*******************************LDO*************************************/
/* Boost_EN:   PC12                                                      */
/* MCU_LDO_EN: PD1                                                      */
/************************************************************************/
void l81_ldo_gpio_config(void)
{
	/* enable the GPIO clock */
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOD);
	
	/*configure MCU_LDO_EN*/
	gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_1);
  gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
	
	/*configure Boost_EN*/
	gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_12);
  gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
}

void l81_dc_off(void)
{
	/* enable the GPIO clock */
	rcu_periph_clock_enable(RCU_GPIOB);
	
	/*configure MCU DC EN pin*/
	gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
	
	gpio_bit_reset(GPIOB, GPIO_PIN_7);
}

void l81_ldo_on(void)
{
	gpio_bit_set(GPIOD, GPIO_PIN_1);
}

void l81_ldo_off(void)
{
	gpio_bit_reset(GPIOD, GPIO_PIN_1);
}

/*******************************ADC*************************************/
/* ADC0/1/2/3/4/5 is used for motor1/2/3/4/5/6                          */
/* ADC6/7, ADC14/15 is used for falling sensor                         */
/* ADC8 is used for VBAT ADC check                                    */
/* ADC12/13 is used for VBAT_NTC1/2                                      */
/************************************************************************/
void l81_adc_gpio_config(void)
{
	/* enable the GPIO clock */
	rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	
	//configure ADC_IN0/1/2/3/4/5/6/7
  gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | 
	              GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |GPIO_PIN_6 | GPIO_PIN_7);
	
	//configure ADC_IN8
	gpio_mode_set(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,  GPIO_PIN_0);
	
	//configure ADC_IN14/15
	gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,  GPIO_PIN_4 | GPIO_PIN_5);	
}

/*!
    \brief      configure the ADC								
    \param[in]  none
    \param[out] none
    \retval     none
*/
void l81_adc_config(void)
{
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC);;
    /* config ADC clock */
    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);
	
    /* ADC data alignment config */
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC_REGULAR_CHANNEL, 1U);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);
    /* ADC external trigger config */
    adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable();
    delay_1ms(1U);
    /* ADC calibration and reset calibration */
    adc_calibration_enable();
}

/*!
    \brief      get ADC channel sample
    \param[in]  none
    \param[out] the original ADC sample value.if you want to get the voltage you should sample*3.3/4096
    \retval     none
*/
uint16_t l81_adc_channel_get_sample(uint8_t channel)
{
		switch (channel){
		case ADC_CHANNEL_0:
			/* ADC regular channel config */
      adc_regular_channel_config(0U, ADC_CHANNEL_0, ADC_SAMPLETIME_7POINT5);
		  break;
		case ADC_CHANNEL_1:
			/* ADC regular channel config */
      adc_regular_channel_config(0U, ADC_CHANNEL_1, ADC_SAMPLETIME_7POINT5);
		  break;
		case ADC_CHANNEL_2:
			/* ADC regular channel config */
      adc_regular_channel_config(0U, ADC_CHANNEL_2, ADC_SAMPLETIME_7POINT5);
		  break;
		case ADC_CHANNEL_3:
			/* ADC regular channel config */
      adc_regular_channel_config(0U, ADC_CHANNEL_3, ADC_SAMPLETIME_7POINT5);
		  break;
		case ADC_CHANNEL_4:
			/* ADC regular channel config */
      adc_regular_channel_config(0U, ADC_CHANNEL_4, ADC_SAMPLETIME_7POINT5);
		  break;
		case ADC_CHANNEL_5:
			/* ADC regular channel config */
      adc_regular_channel_config(0U, ADC_CHANNEL_5, ADC_SAMPLETIME_7POINT5);
		  break;
		case ADC_CHANNEL_6:
			/* ADC regular channel config */
      adc_regular_channel_config(0U, ADC_CHANNEL_6, ADC_SAMPLETIME_7POINT5);
		  break;	
		case ADC_CHANNEL_7:
			/* ADC regular channel config */
      adc_regular_channel_config(0U, ADC_CHANNEL_7, ADC_SAMPLETIME_7POINT5);
		  break;	
		case ADC_CHANNEL_8:
			/* ADC regular channel config */
      adc_regular_channel_config(0U, ADC_CHANNEL_8, ADC_SAMPLETIME_7POINT5);
		  break;	
		case ADC_CHANNEL_14:
			/* ADC regular channel config */
      adc_regular_channel_config(0U, ADC_CHANNEL_14, ADC_SAMPLETIME_7POINT5);
		  break;	
		case ADC_CHANNEL_15:
			/* ADC regular channel config */
      adc_regular_channel_config(0U, ADC_CHANNEL_15, ADC_SAMPLETIME_7POINT5);
		  break;			
		default:
			return 0u;
	}
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);

    /* wait the end of conversion flag */
    while(!adc_flag_get(ADC_FLAG_EOC));
    /* clear the end of conversion flag */
    adc_flag_clear(ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_regular_data_read());	
}

void l81_adc_init(void)
{
	l81_adc_gpio_config();
	l81_adc_config();
}

uint8_t l81_AT_BAT_R_func(char params[])
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
	
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,%d\r\n", (3 * l81_adc_channel_get_sample(ADC_CHANNEL_8)));
		printf("AT+RES,end\r\n");

    return 1U;		
}

/*******************************UART*************************************/
/* USART1 is used for MCU transferring.                                 */
/* LPUART is used for touch sensor.                                 */
/************************************************************************/

#define USART1_RDATA_ADDRESS      (&USART_RDATA(USART1))
#define USART1_TDATA_ADDRESS      (&USART_TDATA(USART1))
#define ARRAYNUM(arr_nanme)      (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))

volatile uint8_t com_txbuffer[TRANSFER_NUM] = {'\0'}; //tx buffer
volatile uint8_t com_rxbuffer[TRANSFER_NUM] = {'\0'}; //rx buffer

BUFFER_STATE_TYPE txbuffer_state = EMPTY; //tx buffer state
BUFFER_STATE_TYPE rxbuffer_state = EMPTY; //rx buffer state

USART_STATE_TYPE tx_state = IDLE;         //tx bus state
USART_STATE_TYPE rx_state = IDLE;         //rx bus state

volatile uint8_t rx_count = 0U;                     //recieve count
volatile uint8_t rx_len = 0U;                       //recieve length

/*!
    \brief      configure DMA interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
//    nvic_irq_enable(DMA_Channel0_IRQn, 1);
//    nvic_irq_enable(DMA_Channel1_IRQn, 0);
	
	  nvic_irq_enable(USART1_IRQn, 0);
}

/*!
    \brief      initialize the USART configuration of the com
    \param[in]  none
    \param[out] none
    \retval     none
*/
void l81_com_usart_config(void)
{
    /* enable COM GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
	  rcu_periph_clock_enable(RCU_GPIOD);
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART1);

    /* connect port to USART TX */
    gpio_af_set(GPIOD, GPIO_AF_7, GPIO_PIN_5);
    /* connect port to USART RX */
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_15);

    /* configure USART TX as alternate function push-pull */
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_5);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_5);

    /* configure USART RX as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_15);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_15);

    /* USART configure */
    usart_deinit(USART1);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_baudrate_set(USART1, 115200U);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
		
		//enable rx RBNE interrupt
		usart_interrupt_enable(USART1, USART_INT_RBNE);

		//enable rx IDLE interrupt
		usart_interrupt_enable(USART1, USART_INT_IDLE);
		
	  usart_interrupt_enable(USART1, USART_INT_ERR);
	  usart_interrupt_enable(USART1, USART_INT_PERR);
		
    usart_enable(USART1);
}
#if 0
void l81_usart1_dma_config()
{
	  dma_parameter_struct dma_init_struct;
    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA);

    /* initialize DMA channel 0 */
    dma_deinit(DMA_CH0);
    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.request      = DMA_REQUEST_USART1_TX;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr  = (uint32_t)com_txbuffer;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number       = ARRAYNUM(com_txbuffer);
    dma_init_struct.periph_addr  = (uint32_t)USART1_TDATA_ADDRESS;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_CH0, &dma_init_struct);

    /* configure DMA mode */
    dma_circulation_disable(DMA_CH0);
    dma_memory_to_memory_disable(DMA_CH0);
    /* disable the DMAMUX_MUXCH0 synchronization mode */
    dmamux_synchronization_disable(DMAMUX_MUXCH0);
    /* USART DMA enable for transmission and reception */
    usart_dma_transmit_config(USART1, USART_DENT_ENABLE);
    /* enable DMA channel 0 transfer complete interrupt */
    dma_interrupt_enable(DMA_CH0, DMA_INT_FTF);
    /* enable DMA channel 0 */
    dma_channel_enable(DMA_CH0);

    /* initialize DMA channel 1 */
    dma_deinit(DMA_CH1);
    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.request      = DMA_REQUEST_USART1_RX;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr  = (uint32_t)com_rxbuffer;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number       = ARRAYNUM(com_rxbuffer);
    dma_init_struct.periph_addr  = (uint32_t)USART1_RDATA_ADDRESS;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_CH1, &dma_init_struct);

    /* configure DMA mode */
    dma_circulation_disable(DMA_CH1);
    dma_memory_to_memory_disable(DMA_CH1);
    /* disable the DMAMUX_MUXCH1 synchronization mode */
    dmamux_synchronization_disable(DMAMUX_MUXCH1);
    /* USART DMA enable for reception */
    usart_dma_receive_config(USART1, USART_DENR_ENABLE);
    /* enable DMA channel 1 transfer complete interrupt */
    dma_interrupt_enable(DMA_CH1, DMA_INT_FTF);
    /* enable DMA channel 1 */
    dma_channel_enable(DMA_CH1);
}

void l81_usart_send_dma_config(void)
{
	dma_parameter_struct dma_init_struct;
	uint32_t dma_periph;
	
  /* initialize DMA channel 0 */
  dma_deinit(DMA_CH0);
  dma_struct_para_init(&dma_init_struct);
  dma_init_struct.request      = DMA_REQUEST_USART1_TX;
  dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_addr  = (uint32_t)com_txbuffer;
  dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
  dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
  dma_init_struct.number       = ARRAYNUM(com_txbuffer);
  dma_init_struct.periph_addr  = (uint32_t)USART1_TDATA_ADDRESS;
  dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
  dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
  dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
  dma_init(DMA_CH0, &dma_init_struct);
	
	dma_circulation_disable(DMA_CH0);
	dma_interrupt_flag_clear(DMA_CH0, DMA_INT_FLAG_FTF|DMA_INT_FLAG_ERR);
	dma_interrupt_enable(DMA_CH0, DMA_INT_FTF|DMA_INT_ERR); 
	
	usart_dma_transmit_config(USART1, USART_DENT_ENABLE);
	usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
	usart_interrupt_flag_clear(USART1, USART_INT_FLAG_TC);
	usart_interrupt_disable(USART1, USART_INT_TC);
//	dma_channel_enable(DMA_CH0);
}

void l81_usart_receive_dma_config(void)
{
	dma_parameter_struct dma_init_struct;
	uint32_t dma_periph;
	
  /* initialize DMA channel 1 */
  dma_deinit(DMA_CH1);
  dma_struct_para_init(&dma_init_struct);
  dma_init_struct.request      = DMA_REQUEST_USART1_RX;
  dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_addr  = (uint32_t)com_rxbuffer;
  dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
  dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
  dma_init_struct.number       = ARRAYNUM(com_rxbuffer);
  dma_init_struct.periph_addr  = (uint32_t)USART1_RDATA_ADDRESS;
  dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
  dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
  dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
  dma_init(DMA_CH1, &dma_init_struct);
	
	dma_circulation_disable(DMA_CH1);
	dma_interrupt_flag_clear(DMA_CH1, DMA_INT_FLAG_FTF|DMA_INT_FLAG_ERR);
	dma_interrupt_enable(DMA_CH1, DMA_INT_FTF|DMA_INT_ERR); 
	dma_channel_enable(DMA_CH1);
	
	usart_dma_receive_config(USART1, USART_DENR_ENABLE); 
	usart_receive_config(USART1, USART_RECEIVE_ENABLE);
	usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);
	usart_interrupt_enable(USART1, USART_INT_RBNE);
	usart_interrupt_enable(USART1, USART_INT_IDLE);
	usart_interrupt_enable(USART1, USART_INT_ERR);
	usart_interrupt_enable(USART1, USART_INT_PERR);
	
}

void l81_open_usart_send(void)
{
	if( tx_state!=BUSY )
	{
			l81_usart_send_dma_config();
			tx_state = BUSY;
	}
}
#endif
void l81_usart_init(void)
{
	nvic_config();
	l81_com_usart_config();
//	l81_usart_receive_dma_config();
}

/*******************************I2C*************************************/


/*******************************LED***************************************/



void send_0(uint32_t pin)
{
	GPIO_BOP(GPIOD) = pin;
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();//__NOP();//__NOP();__NOP();
	GPIO_BC(GPIOD) = pin;
	__NOP();__NOP();__NOP();//__NOP();__NOP();
}

void send_1(uint32_t pin)
{
	GPIO_BOP(GPIOD) = pin;
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();
	GPIO_BC(GPIOD) = pin;
	__NOP();__NOP();__NOP();__NOP();__NOP();
}

void send_reset(uint32_t pin)
{
	GPIO_BC(GPIOD) = pin;
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}
void send_24bit(uint32_t pin, uint32_t LedData)
{
	uint16_t i;
	for (i = 0; i < 24u; i++)
	{
		if (LedData & 0x00800000u)
			send_1(pin);
		else
			send_0(pin);
		
		LedData <<= 1u;
	}
}
void send_GRB(uint32_t pin, uint32_t GRB)
{
  __disable_irq(); //wxf add 0321
	send_reset(pin);
	send_reset(pin);
	send_reset(pin);
	send_reset(pin);
	send_24bit(pin, GRB);
	send_24bit(pin, GRB);
	send_24bit(pin, GRB);
	send_24bit(pin, GRB);
	send_reset(pin);
	send_reset(pin);
	send_reset(pin);
	send_reset(pin);
  __enable_irq(); //wxf add 0321
}

void RESET_GRB(uint32_t pin)
{
	send_GRB(pin, 0x00000000);//GRB
	send_GRB(pin, 0x00000000);//GRB
	send_GRB(pin, 0x00000000);//GRB
	send_GRB(pin, 0x00000000);//GRB
}

void LED_Red_ON(void)
{
	send_GRB(GPIO_PIN_0,0x0000FF00);//R
	send_GRB(GPIO_PIN_0,0x0000FF00);//R
	send_GRB(GPIO_PIN_0,0x0000FF00);//R
	send_GRB(GPIO_PIN_0,0x0000FF00);//R

	send_GRB(GPIO_PIN_1,0x0000FF00);//R
	send_GRB(GPIO_PIN_1,0x0000FF00);//R
	send_GRB(GPIO_PIN_1,0x0000FF00);//R
	send_GRB(GPIO_PIN_1,0x0000FF00);//R
}

void LED_Green_ON(void)
{
	send_GRB(GPIO_PIN_0,0x00FF0000);//G
	send_GRB(GPIO_PIN_0,0x00FF0000);//G
	send_GRB(GPIO_PIN_0,0x00FF0000);//G
	send_GRB(GPIO_PIN_0,0x00FF0000);//G

	send_GRB(GPIO_PIN_1,0x00FF0000);//G
	send_GRB(GPIO_PIN_1,0x00FF0000);//G
	send_GRB(GPIO_PIN_1,0x00FF0000);//G
	send_GRB(GPIO_PIN_1,0x00FF0000);//G
}

void LED_Blue_ON(void)
{
	send_GRB(GPIO_PIN_0,0x000000FF);//B
	send_GRB(GPIO_PIN_0,0x000000FF);//B
	send_GRB(GPIO_PIN_0,0x000000FF);//B
	send_GRB(GPIO_PIN_0,0x000000FF);//B

	send_GRB(GPIO_PIN_1,0x000000FF);//B
	send_GRB(GPIO_PIN_1,0x000000FF);//B
	send_GRB(GPIO_PIN_1,0x000000FF);//B
	send_GRB(GPIO_PIN_1,0x000000FF);//B
}

void LED_On_set(uint32_t leds, uint32_t grb_colour)
{
  //4 leds
  send_GRB(leds, grb_colour);
  send_GRB(leds, grb_colour);
  send_GRB(leds, grb_colour);
  send_GRB(leds, grb_colour);
  
}


uint8_t l81_AT_ledOn_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 1U){  //motor must has two paramters color and status
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num, color,status. [1:red, 2:green, 3:blue ~9]\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
	
	uint32_t color = String2Int(param[0]);
		
	if ((color < 1U) || (color > 9u)) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong color num.[1:red, 2:green, 3:blue ~9]\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
		switch (color){
			case 1://red
				LED_On_set(LEDS_LEFT, LED_GRB_RED);
			  LED_On_set(LEDS_RIGHT, LED_GRB_RED);
				printf("AT+RES,ACK\r\n");
				printf("AT+RES,red on\r\n");
		    printf("AT+RES,end\r\n");
				break;
			case 2://green
				LED_On_set(LEDS_LEFT, LED_GRB_GREEN);
			  LED_On_set(LEDS_RIGHT, LED_GRB_GREEN);
				printf("AT+RES,ACK\r\n");
				printf("AT+RES,green on\r\n");
		    printf("AT+RES,end\r\n");			
				break;
			case 3://blue
				LED_On_set(LEDS_LEFT, LED_GRB_BLUE);
			  LED_On_set(LEDS_RIGHT, LED_GRB_BLUE);
				printf("AT+RES,ACK\r\n");
				printf("AT+RES,blue on\r\n");
		    printf("AT+RES,end\r\n");			
				break;
      case 4://orange
				LED_On_set(LEDS_LEFT, LED_GRB_ORANGE);
			  LED_On_set(LEDS_RIGHT, LED_GRB_ORANGE);
				printf("AT+RES,ACK\r\n");
				printf("AT+RES,orange on\r\n");
		    printf("AT+RES,end\r\n");			
				break;
      case 5://white
				LED_On_set(LEDS_LEFT, LED_GRB_WHITE);
			  LED_On_set(LEDS_RIGHT, LED_GRB_WHITE);
				printf("AT+RES,ACK\r\n");
				printf("AT+RES,white on\r\n");
		    printf("AT+RES,end\r\n");			
				break;
      case 6://yellow
				LED_On_set(LEDS_LEFT, LED_GRB_YELLOW);
			  LED_On_set(LEDS_RIGHT, LED_GRB_YELLOW);
				printf("AT+RES,ACK\r\n");
				printf("AT+RES,yellow on\r\n");
		    printf("AT+RES,end\r\n");			
				break;
      case 7://purple
				LED_On_set(LEDS_LEFT, LED_GRB_PURPLE);
			  LED_On_set(LEDS_RIGHT, LED_GRB_PURPLE);
				printf("AT+RES,ACK\r\n");
				printf("AT+RES,purple on\r\n");
		    printf("AT+RES,end\r\n");			
				break;
      case 8://cyan
				LED_On_set(LEDS_LEFT, LED_GRB_CYAN);
			  LED_On_set(LEDS_RIGHT, LED_GRB_CYAN);
				printf("AT+RES,ACK\r\n");
				printf("AT+RES,cyan on\r\n");
		    printf("AT+RES,end\r\n");			
				break;
      case 9://black
				LED_On_set(LEDS_LEFT, LED_GRB_BLACK);
			  LED_On_set(LEDS_RIGHT, LED_GRB_BLACK);
				printf("AT+RES,ACK\r\n");
				printf("AT+RES,black on\r\n");
		    printf("AT+RES,end\r\n");			
				break;
			default:
				break;
		}

    return 1U;	
}

uint8_t l81_AT_ledOff_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 0U){  //motor must has two paramters color and status
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,there is no params\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
		
		RESET_GRB(GPIO_PIN_0);

	  RESET_GRB(GPIO_PIN_1);

	
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,off\r\n");
		printf("AT+RES,end\r\n");
	
    return 1U;	
}

uint8_t l81_AT_get_date_version_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 0U){  //motor must has two paramters color and status
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,there is no params\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
		
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,ReleaseData: %d%d\r\n", DATE_INT, TIME_INT);
	printf("AT+RES,end\r\n");
	
  return 1U;	
}

uint8_t l81_AT_get_sys_version_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 0U){  //motor must has two paramters color and status
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,there is no params\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
		
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,%s\r\n", l81_version);
	printf("AT+RES,end\r\n");
	
  return 1U;	
}

uint8_t l81_AT_reset_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 0U){  //motor must has two paramters color and status
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,there is no params\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
		
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,MCU will reset after 1s\r\n");
	printf("AT+RES,end\r\n");
	
	delay_1ms(1000);
	NVIC_SystemReset();
	
  return 1U;	
}

uint8_t l81_AT_Poweroff_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 0U){  //motor must has two paramters color and status
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,there is no params\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
	
	//delay 6s make sure soc power off
	delay_1ms(10000);
	l81_dc_off();
	
  return 1U;	
}
/********************************utils**************************************/
/*!
    \brief      string switch to int
    \param[in]  the string of number list, eg "123"
    \param[out] none
    \retval     int
*/
int String2Int(char *str) 
{
    char flag = '+';
    long res = 0;
    
    if(*str=='-') 
    {
        ++str; 
        flag = '-'; 
    } 

    while(*str>=48 && *str<=57)
    {
        res = 10*res + *str++-48;
    } 
 
    if(flag == '-')
    {
        res = -res;
    }
 
    return (int)res;
}

/*!
    \brief      string switch to Hex
								ASCII 48->0, 57->9, 65->A, 70->F, 97->a, 102->f
    \param[in]  the string of number list, eg "0x123", must begin with "0x" or "0X"
    \param[out] none
    \retval     int
*/
uint32_t String2Hex(const char *str)
{
    uint32_t res = 0u;
    const char *p = str;
		uint8_t i = 8u; //only deal 8 byte

    if (!str)
			return 0u;
		
//    if (*str != '0' && *(str + 1) != 'x' && *(str + 1) != 'X')
//			p = str + 2;
        //      0~~9                                   A      ~        F                   a    ~            f
    while((*p>=48u && *p<=57u) || (*p>=65u && *p<=70u) || (*p>=97u && *p<=102u))
    {
			if (i == 0u)
				break;
			else
				i--;
			
       if(*p>=48u && *p<=57u)//      0~~9 
				 res = 16u*res + ((*p++)-48u);
       if(*p>=65u && *p<=70u)//A      ~        F 
         res = 16u*res + (((*p++)-65u) + 10u);
       if(*p>=97 && *p<=102)//a    ~            f
         res = 16u*res + (((*p++)-97u) + 10u);
    }

    return res;
}
/*!
    \brief      string switch to float
								ASCII 48->0, 57->9
    \param[in]  the string of number list, eg "8.123"
    \param[out] none
    \retval     int
*/
float String2float(char *str)
{
    char flag = '+';
    char dot_flag = 0;
    char *p = NULL;
    float integer = 0.0;
    float fractional = 0.0;
    float res = 0.0;
    float dot_base = 0.1; //fractional increase

    if (!str)
			return 0;

    if(*str=='-')
    {
			++str;
      flag = '-';
    }

    while((*str>=48 && *str<=57) || (*str == '.'))
    {
       if (dot_flag == 0){
				 if (*str != '.')
					 integer = 10*integer + ((*str++)-48);
         else{
           str++;
           dot_flag = 1;
         }
       }

       if (dot_flag == 1){
          fractional = fractional + dot_base*((*str++)-48);
          dot_base *= 0.1;
       
			 }
		 }

    res = integer + fractional;

    if(flag == '-')
    {
        res = -res;
    }

    return res;
}

void String2ASCII(char *str, uint32_t data[])
{
	
}

/*---------------------------------------delay timer 5--------------------------------------------*/
volatile static uint64_t counter_4us = 0;

/*!
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void l81_4usdelay_timer_config(void)
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
    rcu_periph_clock_enable(RCU_TIMER5);

    /* deinit a TIMER */
    timer_deinit(TIMER5);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler        = 63;
    timer_initpara.alignedmode      = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period           = 3;   //4us period
    timer_initpara.clockdivision    = TIMER_CKDIV_DIV1;
    timer_init(TIMER5, &timer_initpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER5);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER5, TIMER_INT_UP);

    nvic_irq_enable(TIMER5_IRQn, 1);
		
    /* enable a TIMER */
    timer_enable(TIMER5);
}

/*!
    \brief      this function handles TIMER1 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER5_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER5, TIMER_INT_FLAG_UP)) {
      /* clear channel 0 interrupt bit */
      timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
			
			counter_4us++;
		}
}

void delay_4us(uint32_t count)
{
	uint32_t start_time = counter_4us + count;
	
	while(start_time > counter_4us){};
}

uint32_t get_timer(void)
{
	return counter_4us;
}