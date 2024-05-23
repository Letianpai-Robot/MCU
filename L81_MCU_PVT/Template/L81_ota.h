/*!
    \file    L81_ota.h
    \brief   the header for L81_ota.c
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

#ifndef L81_OTA_H
#define L81_OTA_H

#include "gd32l23x.h"

#define OTA_FLAG_ADDR  ((uint32_t)0x0803B000U)
#define OTA_FLAG_FLAG  (0x12345678u)

#define boot_option_addr ((uint32_t)0x0803B004u)
#define A_Partition (0xAAAAAAAAu)
#define B_Partition (0xBBBBBBBBu)

#define A_Partition_addr ((uint32_t)0x0800D000u) 
#define B_Partition_addr ((uint32_t)0x08024000u) 

extern volatile uint32_t partition_base_addr;
extern uint32_t A_Partition_PageNum ;
extern uint32_t B_Partition_PageNum ;

/*hex line information*/
typedef struct hex_line_info{
	uint8_t  start_code;
	uint8_t  data_len;
	uint16_t data_addr;
	uint8_t  cmd_type;
	uint8_t  data[16];
	uint8_t  checksum;
}HexLineInfo_t;

typedef enum {
  flash_err  = 0u,
	line_end   = 1u,
	hex_end    = 2u,
	NONE       = 3u,
}OTA_Flash_status_e;

typedef enum {
	upgrade_err    = 0u,
	upgrade_running = 1u,
  upgrade_start   = 2u,
	upgrade_done    = 3u,
	upgrade_idle    = 4u,
}OTA_upgrade_status_e;

uint8_t l81_AT_OTA_func(char params[]);
uint8_t l81_AT_get_sys_status_func(char params[]);

#endif  //L81_OTA_H