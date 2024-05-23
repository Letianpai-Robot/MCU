/*!
    \file    L81_AT.c
    \brief   processing AT command 
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
#include <stdlib.h>
#include "L81_Motors.h"
#include "L81_inf.h"
#include "L81_AT.h"
#include "qmi8658.h"
#include "L81_cliff.h"
#include "L81_FMC.h"
#include "L81_factory.h"
#include "L81_ota.h"
#include "L81_IR.h"
#include "L81_rtc.h"
#include "qmi8658.h"

#include "L81_MoveAlg.h"
#include "algorithm.h"
#include "qmc6308iic.h"
#include "gxhtc3iic.h"
#include "L81_main_task.h"
#include "VI530x_API.h"
#include "aw9310x.h"
#include "L81_cliff_task.h"

#define FALSE 0u
#define TRUE  1u

static ATcmd_handler_list_type *ATcmd_handler_list = NULL;

/*record all the AT command and related functioin*/
static ATcmd_handler_type ATcmd_handler[] = {
	{"AT+MOTORW", &l81_AT_MOTOR_W_func},
	{"AT+MOTORR", &l81_AT_MOTOR_R_func},
	{"AT+AG",     &l81_AT_AG_R_func},
	{"AT+CLIFFR",  &l81_AT_CLIFF_R_func},
	{"AT+CLIFFW",  &l81_AT_CLIFF_W_func},
	{"AT+CLIFFD",  &l81_AT_CLIFF_Danger_func},
	{"AT+FMCW",    &l81_AT_FMC_W_func},
	{"AT+FMCR",    &l81_AT_FMC_R_func},
	{"AT+LADDR", &l81_AT_FMC_get_user_addr_func},
	{"AT+FMCDUMP",  &l81_AT_FMC_dump_func},
//	{"AT+SNW",  &l81_AT_SNW_func},
	{"AT+SNR",  &l81_AT_SNR_func},
//	{"AT+VerW",  &l81_AT_VerW_func},
	{"AT+VerR",    &l81_AT_get_sys_version_func},
	{"AT+DateVer",  &l81_AT_get_date_version_func},
	{"AT+AGID",    &l81_AT_AG_id_func},
	{"AT+LEDOn",    &l81_AT_ledOn_func},
	{"AT+LEDOff",    &l81_AT_ledOff_func},
	{"AT+BATR",    &l81_AT_BAT_R_func},
	{"AT+Lid",    &l81_AT_Light_id_func},
	{"AT+Pid",    &l81_AT_Tmos_id_func},
	{"AT+Tid",    &l81_AT_Touch_id_func},
	{"AT+Did",  &l81_AT_Tof_id_func},
//	{"AT+THid",  &l81_AT_TH_id_func},
	
	{"AT+Atestid",  &l81_AT_AG_test_id_func},
	
	{"AT+Mset",    &l81_AT_set_pulse_func},
	{"AT+Mreset",  &l81_AT_motor_reset_func},
	{"AT+Msavel",  &l81_AT_lowstore_func},
	{"AT+Msaveh",  &l81_AT_highstore_func},
	{"AT+Msavem",  &l81_AT_midstore_func},
	{"AT+Munlock",  &l81_AT_unlock_func},
	{"AT+Mread",  &l81_AT_read_func},
	
	{"AT+MOTA",  &l81_AT_OTA_func},
	{"AT+Gsys",  &l81_AT_get_sys_status_func},
	
	{"AT+Reset",  &l81_AT_reset_func},
	{"AT+PowerOff",  &l81_AT_Poweroff_func},
	
	{"AT+IRstart",  &l81_IR_start_func},
	{"AT+IRstop",  &l81_IR_stop_func},
	{"AT+TofR",  &l81_VI53_dem_func},
	{"AT+TofCalR", &l81_VI53_cal_read_raw_func},
	{"AT+TofCal",  &l81_VI53_cal_func},
	{"AT+TofSet",  &l81_VI53_TofSet_func},
//	{"AT+TofTest",  &l81_VI53_test_func},
	
  {"AT+MOVEW",  &l81_AT_MOVE_W_func},
	{"AT+EARW",  &l81_AT_EAR_W_func},
  
  {"AT+MCalp",  &l81_AT_pre_calibration_func},
  {"AT+MCalm",  &l81_AT_save_90calibration_func},
  {"AT+MCall",  &l81_AT_save_0calibration_func},
  {"AT+MCalh",  &l81_AT_save_180calibration_func},
  
	{"AT+AGCalR", &l81_AT_AG_R_raw_func},
	{"AT+AGCal",  &l81_AT_cal_W_func},
	
	{"AT+RTCw",  &l81_AT_RTC_setdatetime_func},
	{"AT+RTCr",  &l81_AT_RTC_getdatetime_func},
  
  {"AT+FiAGW",  &l81_AT_FILTE_AG_W_func},
  {"AT+FiAGR",  &l81_AT_FILTE_AG_R_func},
  
  //read cmd 0=id 1=value; eg: AT+MAGR,0\r\n;
  {"AT+MAGR",  &l81_AT_qmcr_func},
  {"AT+THR",  &l81_AT_gxhtcr_func},
  
  {"AT+FunCtr",  &l81_AT_contrl_func},
  {"AT+AWR",  &l81_AT_AW_R_func},
  {"AT+AWStart",  &l81_AT_AW_START_func},
  {"AT+AWStop",  &l81_AT_AW_STOP_func},
  {"AT+CfgR",  &l81_AT_CfgR_func},
  {"AT+CfgW",  &l81_AT_CfgW_func},
  
	{NULL, NULL}
};

/*!
    \brief      AT_check_command_valid function
								every AT command begin with AT+, so just judge AT+
    \param[in]  buf, uart recieved buffer (com_rxbuffer)
    \param[out] none
    \retval     1: valid,0 invalid
*/
int8_t AT_check_command_valid(void)
{
	char *cmd = (char *)com_rxbuffer;
	char *tmp = (char *)com_rxbuffer;
	uint16_t i = 0u;
	
	char A = *cmd;               //the first charactor is should be 'A'
	char T = *(cmd + 1U);        //the second charactor is should be 'T'
	char P = *(cmd + 2U);        //the third charactor is should be '+'
	
	if (A == 'A' && T == 'T' && P == '+'){
		for ( i = 0; i < TRANSFER_NUM; i++){
			if ((*cmd == 92) && (*(cmd + 1) == 'r') && 	(*(cmd + 2) == 92) && (*(cmd + 3) == 'n')){      //after find "\r\n", AT command finished,break, exit loop
				return 1U;
			}
			if (i == 96)   //less then TRANSFER_NUM 100
				break;
			cmd = tmp + i;
		}
		if (i == 96){   //less then TRANSFER_NUM 100
			if ((*cmd == 92) && (*(cmd + 1) == 'r') && 	(*(cmd + 2) == 92) && (*(cmd + 3) == 'n'))
				return 1u;
			else
				return 0u;
		}else
			return 0u;
	}	else
		return 0U;
}
/*!
    \brief      AT_check_len function
    \param[in]  buf, uart recieved buffer (com_rxbuffer)
    \param[out] none
    \retval     len: AT cmd length,0 no command
*/
int8_t AT_get_cmd_len(void)
{
	uint8_t i  = 0U;
	char *prev = (char *)com_rxbuffer;
	char *next = (char *)com_rxbuffer;
	
	if (AT_check_command_valid()){
		for(; i < TRANSFER_NUM ; i++){   //try to scan all the com_rxbuffer
			prev = next;
			next = next + i + 1U;
			
			if (*prev != '\r' && *next != '\n'){   //every AT cmd finished with "\r\n"
				continue;
			}
			else
				break;                               //after find finish flags, break, exit loop
		}
		return i;
	}	else
		return 0U; //no command, return 0
}
/*!
    \brief      AT_get_cmd_name function
    \param[in]  buf, name buffer
    \param[out] name buffer
    \retval     none
*/
uint16_t ATcmd_get_name(char name[])
{
	uint8_t name_len = 0U;
	
	char *p = (char *)com_rxbuffer;
	char *tmp = (char *)com_rxbuffer;

	for (; name_len < TRANSFER_NUM ; name_len++){ // try to scan all the com_rxbuffer
		p = tmp + name_len;
		if (*p == ',')                               // find the first ',' break, exit loop
			break;
		else if ( *p == 92)                          // if there isn't any parameters, it does't have the first ',', but it must be end with \r\n
			break;
	}
	
	if (name_len >= ATcmd_NAME_MAX_LEN)
		return FALSE;
	
	memcpy(name, (void *)com_rxbuffer, name_len);
	*(name + name_len) = '\0';
	
	return TRUE;
}

uint16_t ATcmd_get_params(char params[])
{
	uint8_t i  = 0U;
	uint8_t name_len = 0U;

	char *p_name = (char *)com_rxbuffer;
	char *p_params = (char *)com_rxbuffer;
	char *tmp = (char *)com_rxbuffer;   //protect thr original pointer

	for(; i < TRANSFER_NUM; i++){       // try to scan all the com_rxbuffer
		p_params = tmp + i;
		
		if (*p_name != ','){               //after find the first ',', parama begin
			p_name = p_params;
			name_len++;
		}

		if (*p_params == 92 && *(p_params + 1) == 'r' && \
			*(p_params + 2) == 92 && *(p_params + 3) == 'n'){      //after find "\r\n", AT command finished,break, exit loop
			break;
		}
	}
	
	if (i >= ATcmd_PARAMS_MAX_LEN)
		return FALSE;
	
		if (*p_name == 92)         //if p_name == '\', it means there hasn't parameters, eg, AT+TEST\r\n
			params[0] = '\0';        //In this case, params array is NULL, let the first item is '\0'
		else{
		//p_name pointer the first ','. total count remove name lenght, keep "\r\n"
		memcpy(params, (void *)(p_name + 1), (i - name_len + 4U));
	  *(params + (i - name_len + 4U + 1U)) = '\0';   // 4U is "\r\n" , 1U used for '\0'
		}
		
		return TRUE;
}

uint16_t ATcmd_split_params(char params[], char param[][PER_PARAM_MAX_LEN], uint8_t *param_num)
{
	char *p = params;
	char *tmp = params;
	uint8_t sub_param_len = 0U;
	uint8_t i = 0U, j = 0U;
	
	if (params[0] == '\0'){              //first to check the params is null or not. if the first item is '\0',
		*param_num = 0u;                   //it means param number is 0.
		return FALSE;
	}

	for (; i < ATcmd_PARAMS_MAX_LEN; ){                  
		p = tmp + i;
		if (*p != ','){
			sub_param_len++;
			i++;
		}else{
			if (sub_param_len >= PER_PARAM_MAX_LEN)
				return FALSE;
			memcpy(param[j], tmp, (sub_param_len));  //length should remove ','
			tmp = tmp + sub_param_len + 1;
			sub_param_len = 0U;
			i = 0U;
			j++;
		}
		
		if (sub_param_len >= PER_PARAM_MAX_LEN)
				return FALSE;

		//92 is the ASCII of '\'
		if (*p == 92 && *(p + 1U) == 'r' && \
			*(p + 2U) == 92 && *(p + 3U) == 'n'){
			memcpy(param[j], tmp, (sub_param_len - 1U));  // length should remove '\t', after find finish flag, break, exit loop
			break;
		}
	}
	
	*param_num = (j + 1);   // j is begin from 0, so the number of param is j + 1;
	if(*param_num >= PARAM_MAX_NUM)
		return FALSE;
	
	return TRUE;
}
/*!
    \brief      AT_check_command_valid function
    \param[in]  buf, uart recieved buffer (com_rxbuffer)
    \param[out] 0: valid,-1 invalid
    \retval     none
*/
static uint16_t ATcmd_init_cmdData(ATcmd_type *cmd, char name[], char params[])
{
	uint16_t ret;
	ret = ATcmd_get_name(name);
	if (ret == FALSE)
		return ret;
	ret = ATcmd_get_params(params);
	if (ret == FALSE)
		return ret;
	cmd->name = name;
	cmd->params = params;
	return TRUE;
//	printf("Get AT command \r\n name:%s\r\n params:%s\r\n", name, params);
}

ATcmd_handler_type * ATcmd_match_handler(ATcmd_type *cmd)
{
	ATcmd_handler_list_type *p = ATcmd_handler_list;
	
	for(; p->next != NULL; p = p->next){
		if (p->ATcmd_handler != NULL){
		  if (!strcmp(p->ATcmd_handler->name, cmd->name)){
			   return p->ATcmd_handler;
		  }
		}
	}
	
	//check last node
	if (p->ATcmd_handler != NULL){
	  if (!strcmp(p->ATcmd_handler->name, cmd->name)){
		  return p->ATcmd_handler;
	  }else{
		  return NULL;
	  }
  }
	
	return NULL;
}

void ATcmd_execute(ATcmd_type *cmd)
{
	ATcmd_handler_type *handler = ATcmd_match_handler(cmd);
	if (handler){
	  handler->ATfunc(cmd->params);
	}
}

void ATcmd_server()
{
	uint16_t ret;
	ATcmd_type cmd;
	char name[ATcmd_NAME_MAX_LEN]   = {'\0'};
	char params[ATcmd_PARAMS_MAX_LEN] = {'\0'};
	
	//check rx has AT command or not
	if (!AT_check_command_valid())
		return;
	ret = ATcmd_init_cmdData(&cmd, name, params);
	if (ret == FALSE)
		return;
	ATcmd_execute(&cmd);
}

void ATcmd_register_handler(ATcmd_handler_type *ATcmd_handler)
{
	ATcmd_handler_list_type *p = ATcmd_handler_list;
	ATcmd_handler_list_type *newItem = (ATcmd_handler_list_type *)malloc(sizeof(ATcmd_handler_list_type));
	
	//insert newItem to ATcmd_handler_list last node
	for(; p->next != NULL; p = p->next){}
		
	p->next = newItem;
	newItem->prev = p;
	newItem->next =NULL;
	newItem->ATcmd_handler = ATcmd_handler;
}

void ATcmd_register(char *ATcmd_name, uint8_t (*ATfunc)(char params[]))
{
	ATcmd_handler_type *ATcmd_handler = malloc(sizeof(ATcmd_handler_type));
	
	ATcmd_handler->name = malloc(16); //max name size is 16
	memcpy(ATcmd_handler->name, ATcmd_name, strlen(ATcmd_name));
	ATcmd_handler->ATfunc = ATfunc;
	
	ATcmd_register_handler(ATcmd_handler);	
}

/*this must be called by main function*/
void ATcmd_init(void)
{
	uint8_t i = 0U;
	uint8_t ATcmd_num = sizeof(ATcmd_handler)/sizeof(ATcmd_handler_type);
	
	ATcmd_handler_list = (ATcmd_handler_list_type *)malloc(sizeof(ATcmd_handler_list_type));
	ATcmd_handler_list->ATcmd_handler = NULL;
	ATcmd_handler_list->prev = NULL;
	ATcmd_handler_list->next = NULL;
	
	for (; i < ATcmd_num; i++){
		if (ATcmd_handler[i].name && ATcmd_handler[i].ATfunc)
			ATcmd_register(ATcmd_handler[i].name, ATcmd_handler[i].ATfunc);
	}
}

uint8_t l81_AT_CfgR_func(char params[])
{
	char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

	ATcmd_split_params(params, param, &param_num);
	
  uint32_t Cfg_cmd = 0;
//	uint32_t Cfg_value = 0;
	
	if ((param_num < 1U) || (param_num > 1U))    
  {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num,[1,1]\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
  } 
	Cfg_cmd = String2Int(param[0]);
//	Cfg_value = String2Int(param[1]);
	
	switch(Cfg_cmd)
	{
		case 0:
			return l81_AT_CfgR_func_CLIFF_DENGER_NUM();
			break;
//		case 1:
//			
//			break;
		
		default:
			printf("AT+RES,ACK\r\n");
			printf("AT+RES,Err,wrong param Cfg_cmd,[0,0]\r\n");
			printf("AT+RES,end\r\n");
			return 0U;
			break;
	}
	
//	printf("AT+RES,ACK\r\n");
//	printf("AT+RES,end\r\n");
	return 1U;
	
}
uint8_t l81_AT_CfgW_func(char params[])
{

	char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

	ATcmd_split_params(params, param, &param_num);
	
  uint32_t Cfg_cmd = 0;
	uint32_t Cfg_value = 0;
	
	if ((param_num < 2U) || (param_num > 2U))    
  {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num,[2,2]\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
  } 
	Cfg_cmd = String2Int(param[0]);
	Cfg_value = String2Int(param[1]);
	
	switch(Cfg_cmd)
	{
		case 0:
			return l81_AT_CfgW_func_CLIFF_DENGER_NUM(Cfg_value);
			break;
//		case 1:
//			
//			break;
		
		default:
			printf("AT+RES,ACK\r\n");
			printf("AT+RES,Err,wrong param Cfg_cmd,[0,0]\r\n");
			printf("AT+RES,end\r\n");
			return 0U;
			break;
	}
	
//	printf("AT+RES,ACK\r\n");
//	printf("AT+RES,end\r\n");
	return 1U;
}
