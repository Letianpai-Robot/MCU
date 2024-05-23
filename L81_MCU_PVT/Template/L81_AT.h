/*!
    \file    L81_AT.h
    \brief   the header for L81_AT.c
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

#ifndef L81_AT_H
#define L81_AT_H

#include "gd32l23x.h"

#define ATcmd_NAME_MAX_LEN   16U   //(name max len) + (params max len) this should less than com_rxbuffer  or equel it.
#define ATcmd_PARAMS_MAX_LEN 80U

#define PARAM_MAX_NUM        7U    //Every AT command must has less than 6 params  1103 6->7
#define PER_PARAM_MAX_LEN    12U    //every param must less than 12 charator

typedef struct
{
	char *name;                       //AT command name,eg,AT+MOTORR, AT+SENSORW and so on
  uint8_t (*ATfunc)(char params[]);  // AT command handle function
}ATcmd_handler_type;                        //this used for registering an AT command in system

typedef struct{
	char *name;
	char *params;
}ATcmd_type;

typedef struct ATcmd_handler_list_type
{
	ATcmd_handler_type *ATcmd_handler;
  struct ATcmd_handler_list_type *prev;
  struct ATcmd_handler_list_type *next;
}ATcmd_handler_list_type;
			 
extern void ATcmd_server();
extern void ATcmd_init(void);
extern void ATcmd_register_handler(ATcmd_handler_type *ATcmd_handler);
extern void ATcmd_register(char *ATcmd_name, uint8_t (*ATfunc)(char params[]));
extern uint16_t ATcmd_split_params(char params[], char param[][PER_PARAM_MAX_LEN], uint8_t *param_num);
uint8_t l81_AT_CfgR_func(char params[]);
uint8_t l81_AT_CfgW_func(char params[]);

#endif  //L81_AT_H