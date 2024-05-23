/*!
    \file    L81_cliff.h
    \brief   the header for L81_cliff.c
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

#ifndef L81_FACTORY_H
#define L81_FACTORY_H

#include "gd32l23x.h"

extern uint16_t test_pulse;

uint8_t l81_AT_Light_id_func(char params[]);
uint8_t l81_AT_Tmos_id_func(char params[]);
uint8_t l81_AT_Touch_id_func(char params[]);
uint8_t l81_AT_Tof_id_func(char params[]);
uint8_t l81_AT_AG_test_id_func(char params[]);
uint8_t l81_AT_TH_id_func(char params[]);

void l81_delay_timer_config(void);
void l81_delay_timer_nvic_config(void);
void t_delay_us(uint32_t count);

void motor_send_readcmd(void);
void motor_end_readcmd(void);
void motor_unlock_force(void);
void motor_store_mid_offset(void);
void motor_store_500_1500(void);
void motor_store_1500_2500(void);
void motor_reset(void);
void motor_write_pulse(uint16_t pulse);

extern volatile uint64_t angle_pulse;  //return value  is (angle_pulse * 4)us
#define GET_ANGLE ((uint32_t)angle_pulse * 4u)

uint8_t l81_AT_set_pulse_func(char params[]);
uint8_t l81_AT_motor_reset_func(char params[]);
uint8_t l81_AT_lowstore_func(char params[]);
uint8_t l81_AT_highstore_func(char params[]);
uint8_t l81_AT_midstore_func(char params[]);
uint8_t l81_AT_unlock_func(char params[]);
uint8_t l81_AT_read_func(char params[]);


uint8_t l81_AT_pre_calibration_func(char params[]);   //unlock
uint8_t l81_AT_save_90calibration_func(char params[]);  //stor 90
uint8_t l81_AT_save_0calibration_func(char params[]);   //stor 0~90
uint8_t l81_AT_save_180calibration_func(char params[]); //stor 90~180
#endif  //L81_FACTORY_H