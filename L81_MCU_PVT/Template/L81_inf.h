/*!
    \file    L81_inf.h
    \brief   the header for L81_inf.c
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

#ifndef L81_INF_H
#define L81_INF_H

#include "gd32l23x.h"

/*--------------------syttem version----------------------*/

#define l81_version "LRAM.1.1.31" //for pvt 

/*--------------------syttem version----------------------*/

/*-------------------get system time for version--------------------*/
#define YEAR ((((__DATE__ [7] - '0') * 10 + (__DATE__ [8] - '0')) * 10 + (__DATE__ [9] - '0')) * 10 + (__DATE__ [10] - '0'))

#define MONTH ( __DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? 1 : 6) \
: __DATE__ [2] == 'b' ? 2 \
: __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 3 : 4) \
: __DATE__ [2] == 'y' ? 5 \
: __DATE__ [2] == 'n' ? 6 \
: __DATE__ [2] == 'l' ? 7 \
: __DATE__ [2] == 'g' ? 8 \
: __DATE__ [2] == 'p' ? 9 \
: __DATE__ [2] == 't' ? 10 \
: __DATE__ [2] == 'v' ? 11 : 12)

#define DAY ((__DATE__ [4] == ' ' ? 0 : ((__DATE__ [4] - '0') * 10 )) + (__DATE__ [5] - '0'))

#define HOUR ((__TIME__ [0] == ' ' ? 0 : ((__TIME__ [0] - '0') * 10 )) + (__TIME__ [1] - '0'))

#define MINUTE ((__TIME__ [3] == ' ' ? 0 : ((__TIME__ [3] - '0') * 10 )) + (__TIME__ [4] - '0'))

#define SECOND ((__TIME__ [6] == ' ' ? 0 : ((__TIME__ [6] - '0') * 10 )) + (__TIME__ [7] - '0'))

#define DATE_INT (YEAR * 10000 + MONTH * 100 + DAY) 

#define TIME_INT (HOUR * 10000 + MINUTE * 100 + SECOND)

/*-------------------get system time for version--------------------*/
/**/
#define GPIO_SPEED_FREQ_HIGH 1
#define GPIO_SPEED_FREQ_LOW 0

#define LEDS_LEFT        GPIO_PIN_0
#define LEDS_RIGHT       GPIO_PIN_1

#define LEDS_RGB_ENCOD_EN   0     //1=rgb (for pvt1) 0=grb(for pvt2)
#if LEDS_RGB_ENCOD_EN
#define LED_GRB_BLUE      0X003662EC
#define LED_GRB_ORANGE    0X00FF8F1F
#define LED_GRB_WHITE     0X00FFFFFF
#define LED_GRB_YELLOW    0X00FFC300
#define LED_GRB_GREEN     0X0000B578
#define LED_GRB_PURPLE    0X00EB2F96
#define LED_GRB_RED       0X00FF2525
#define LED_GRB_CYAN      0X0007B9B9
#define LED_GRB_BLACK     0X00000000
#else
#define LED_GRB_BLUE      0X006236EC
#define LED_GRB_ORANGE    0X008FFF1F
#define LED_GRB_WHITE     0X00FFFFFF
#define LED_GRB_YELLOW    0X00C3FF00
#define LED_GRB_GREEN     0X00B50078
#define LED_GRB_PURPLE    0X002FEB96
#define LED_GRB_RED       0X0025FF25
#define LED_GRB_CYAN      0X00B907B9
#define LED_GRB_BLACK     0X00000000
#endif
/**/

/*USART1 MAX number for transfer*/
#define TRANSFER_NUM (100U)

void l81_ldo_gpio_config(void);
void l81_ldo_on(void);
void l81_ldo_off(void);
void l81_adc_gpio_config(void);
void l81_adc_config(void);
void l81_adc_init(void);
uint16_t l81_adc_channel_get_sample(uint8_t channel);


typedef enum{
	IDLE  = 0U,
	BUSY  = 1U,
  READY = 2U,
}USART_STATE_TYPE;

typedef enum{
	EMPTY  = 0U,
	FULL   = 1U,
}BUFFER_STATE_TYPE;

extern USART_STATE_TYPE tx_state;
extern USART_STATE_TYPE rx_state;

extern volatile  uint8_t com_rxbuffer[TRANSFER_NUM];
extern volatile  uint8_t com_txbuffer[TRANSFER_NUM];

extern BUFFER_STATE_TYPE txbuffer_state; //tx buffer state
extern BUFFER_STATE_TYPE rxbuffer_state; //rx buffer state

extern volatile uint8_t rx_count;
extern volatile uint8_t rx_len;

void l81_com_usart_config(void);
extern void l81_usart_init(void);
extern void l81_usart_transmit(uint8_t *data, uint16_t len);
extern void l81_usart_receive(void);
void l81_usart_receive_dma_config(void);
void l81_open_usart_send(void);
extern void RESET_GRB(uint32_t pin);
extern void send_GRB(uint32_t pin, uint32_t GRB);
extern void LED_Red_ON(void);
extern void LED_Green_ON(void);
extern void LED_Blue_ON(void);
void LED_On_set(uint32_t leds, uint32_t grb_colour);
extern uint8_t l81_AT_BAT_R_func(char params[]);
extern uint8_t l81_AT_ledOn_func(char params[]);
extern uint8_t l81_AT_ledOff_func(char params[]);
extern uint8_t l81_AT_get_date_version_func(char params[]);
extern uint8_t l81_AT_get_sys_version_func(char params[]);
extern uint8_t l81_AT_reset_func(char params[]);
extern uint8_t l81_AT_Poweroff_func(char params[]);

extern int String2Int(char *str);
extern uint32_t String2Hex(const char *str);

void l81_4usdelay_timer_config(void);
void delay_4us(uint32_t count);
uint32_t get_timer(void);

#endif  //L81_INF_H