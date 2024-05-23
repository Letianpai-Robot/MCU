/*!
    \file    L81_Motors.h
    \brief   the header for L81_Motors.c
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

#ifndef L81_MOTORS_H
#define L81_MOTORS_H

#include "gd32l23x.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"

typedef enum{
	MOTOR_NUM1  = 1U,
	MOTOR_NUM2  = 2U,
	MOTOR_NUM3  = 3U,
	MOTOR_NUM4  = 4U,
	MOTOR_NUM5  = 5U,
	MOTOR_NUM6  = 6U,
}MOTOR_NUM_TYPE;


extern void l81_motor_gpio_config(void);
extern void l81_motor_poweron(void);
extern void l81_motor_poweroff(void);
extern void l81_motor_timer_config(uint32_t timer_periph, rcu_periph_enum rcu_periph);
extern void motor_set_angle(MOTOR_NUM_TYPE motor, uint32_t angle);
extern uint16_t motor_set_pulse(MOTOR_NUM_TYPE motor, uint32_t pulse);
extern uint8_t l81_AT_MOTOR_W_func(char params[]);
extern uint8_t l81_AT_MOTOR_R_func(char params[]);
extern void l81_motor_init(void);


#endif  //L81_MOTORS_H