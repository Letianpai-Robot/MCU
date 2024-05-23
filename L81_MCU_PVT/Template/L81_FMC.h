/*!
    \file    L81_FMC.h
    \brief   the header for L81_FMC.c
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

#ifndef L81_FMC_H
#define L81_FMC_H

#include "gd32l23x.h"

//need 3 word. acc x,y,z data, every has 8 charactors, if there is a negative flag, it should be align 4 byte.
#define Acc_len           (3u)              //word count, 3 word.
#define Gyro_len          (3u)
#define Tof_len           (2u)
#define Motor_len         (2u)              //thera are 6 motors, so motor will have 6 words
#define Ver_len           (4u)              //word count, 4 word means 16 bytes. version number need less than 16 bytes
#define SN_len            (4u)
#define Cliff_len         (4u)
#define Byte_MASK          (0xFF000000U) 

/*need 1 word (4 Byte) align */
#define USER_ADDR_START   (0x0803D000U)

#define ACCX_ADDR         (USER_ADDR_START +  0U       * sizeof(uint32_t))
#define ACCY_ADDR         (ACCX_ADDR       +  Acc_len  * sizeof(uint32_t))
#define ACCZ_ADDR         (ACCY_ADDR       +  Acc_len  * sizeof(uint32_t))
	
#define GYROX_ADDR        (ACCZ_ADDR       +  Acc_len  * sizeof(uint32_t))
#define GYROY_ADDR        (GYROX_ADDR      +  Gyro_len * sizeof(uint32_t))
#define GYROZ_ADDR        (GYROY_ADDR      +  Gyro_len * sizeof(uint32_t))
	
#define TOF_ADDR          (GYROZ_ADDR      +  Gyro_len * sizeof(uint32_t))

#define Motor1_ADDR       (TOF_ADDR        +  Tof_len  * sizeof(uint32_t))
#define Motor2_ADDR       (Motor1_ADDR     +  Motor_len  * sizeof(uint32_t))
#define Motor3_ADDR       (Motor2_ADDR     +  Motor_len  * sizeof(uint32_t))
#define Motor4_ADDR       (Motor3_ADDR     +  Motor_len  * sizeof(uint32_t))
#define Motor5_ADDR       (Motor4_ADDR     +  Motor_len  * sizeof(uint32_t))
#define Motor6_ADDR       (Motor5_ADDR     +  Motor_len  * sizeof(uint32_t))	

#define SN_ADDR           (Motor6_ADDR     +  Motor_len  * sizeof(uint32_t))
#define Ver_ADDR          (SN_ADDR         +  SN_len   * sizeof(uint32_t))
	
#define Cliff_ADDR        (Ver_ADDR        +  Ver_len  * sizeof(uint32_t))
	
#define USER_ADDR_END     (Cliff_ADDR      +  Cliff_len  * sizeof(uint32_t))
	/*note: if increase store addr should modifiy l81_fmc_program func backup size*/

int fmc_test(void);
extern void l81_fmc_read_words(uint32_t addr, uint8_t str[], uint8_t len);
extern uint32_t l81_fmc_read(uint32_t addr);
extern void l81_fmc_program(uint32_t addr, uint32_t data);
extern uint8_t l81_fmc_program_check(uint32_t addr, uint32_t data);

extern uint8_t l81_AT_FMC_W_func(char params[]);
extern uint8_t l81_AT_FMC_R_func(char params[]);
extern uint8_t l81_AT_VerW_func(char params[]);
extern uint8_t l81_AT_VerR_func(char params[]);
extern uint8_t l81_AT_SNW_func(char params[]);
extern uint8_t l81_AT_SNR_func(char params[]);
extern uint8_t l81_AT_FMC_get_user_addr_func(char params[]);
extern uint8_t l81_AT_FMC_dump_func(char params[]);

#endif  //L81_FMC_H
