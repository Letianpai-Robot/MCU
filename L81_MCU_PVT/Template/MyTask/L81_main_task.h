/*!
    \file    L81_main_task.h
    \brief   the header for L81_main_task.c
*/

/*
    Copyright (c) 2023, wxf
*/

#ifndef L81_MAIN_TASK_H
#define L81_MAIN_TASK_H

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "gd32l23x.h"
#include "systick.h"
#include "main.h"


/*define port*/
//config ear motor and led pwr
#define CTR_EARL_PWR_ON()     gpio_bit_set(GPIOC, GPIO_PIN_2)
#define CTR_EARL_PWR_OFF()    gpio_bit_reset(GPIOC, GPIO_PIN_2)

#define CTR_EARR_PWR_ON()     gpio_bit_set(GPIOC, GPIO_PIN_3)
#define CTR_EARR_PWR_OFF()    gpio_bit_reset(GPIOC, GPIO_PIN_3)

//config ear motor pwm stop and star 
//#define CTR_EARL_PWR_ON()    timer_channel_output_state_config(TIMER2, TIMER_CH_2, TIMER_CCX_ENABLE)   //gpio_bit_set(GPIOC, GPIO_PIN_2)
//#define CTR_EARL_PWR_OFF()   timer_channel_output_state_config(TIMER2, TIMER_CH_2, TIMER_CCX_DISABLE)  //gpio_bit_reset(GPIOC, GPIO_PIN_2)
//        
//#define CTR_EARR_PWR_ON()    timer_channel_output_state_config(TIMER2, TIMER_CH_3, TIMER_CCX_ENABLE)   //gpio_bit_set(GPIOC, GPIO_PIN_3)
//#define CTR_EARR_PWR_OFF()   timer_channel_output_state_config(TIMER2, TIMER_CH_3, TIMER_CCX_DISABLE)  //gpio_bit_reset(GPIOC, GPIO_PIN_3)



/**/

void main_task_init(void);

uint8_t l81_AT_contrl_func(char params[]);

#endif  //