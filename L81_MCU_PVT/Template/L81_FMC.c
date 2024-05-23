/*!
    \file    L81_FMC.c
    \brief   the basical interface for read/write flash
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
#include <stdlib.h>
#include "main.h"
#include <string.h>
#include "L81_FMC.h"
#include "L81_AT.h"
#include "L81_inf.h"

#define FMC_PAGE_SIZE                ((uint16_t)0x1000U)
#define FMC_WRITE_START_ADDR         ((uint32_t)0x0803D000U)
#define FMC_WRITE_END_ADDR           ((uint32_t)0x0803DFFFU)

#define FMC_PROGRAM_TYPE_WORD        ((uint8_t)0x00U)
#define FMC_PROGRAM_TYPE_FAST        ((uint8_t)0x01U)

uint32_t address = 0x00U;
uint32_t data0   = 0x01234567U;
uint32_t data1   = 0xd583179bU;

/* calculate the number of page to be programmed/erased */
uint32_t PageNum = (FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR + 1) / FMC_PAGE_SIZE;
/* calculate the number of page to be programmed/erased */
uint32_t WordNum = ((FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR + 1) >> 2);

/* data buffer for fast programming */
static uint64_t data_buffer[DOUBLE_WORDS_CNT_IN_ROW] = {
    0x0000000000000000U, 0x1111111111111111U, 0x2222222222222222U, 0x3333333333333333U,
    0x4444444444444444U, 0x5555555555555555U, 0x6666666666666666U, 0x7777777777777777U,
    0x8888888888888888U, 0x9999999999999999U, 0xAAAAAAAAAAAAAAAAU, 0xBBBBBBBBBBBBBBBBU,
    0xCCCCCCCCCCCCCCCCU, 0xDDDDDDDDDDDDDDDDU, 0xEEEEEEEEEEEEEEEEU, 0xFFFFFFFFFFFFFFFFU,
    0x0011001100110011U, 0x2233223322332233U, 0x4455445544554455U, 0x6677667766776677U,
    0x8899889988998899U, 0xAABBAABBAABBAABBU, 0xCCDDCCDDCCDDCCDDU, 0xEEFFEEFFEEFFEEFFU,
    0x2200220022002200U, 0x3311331133113311U, 0x6644664466446644U, 0x7755775577557755U,
    0xAA88AA88AA88AA88U, 0xBB99BB99BB99BB99U, 0xEECCEECCEECCEECCU, 0xFFDDFFDDFFDDFFDDU
};

/*!
    \brief      erase fmc pages from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_erase_pages(void)
{
    uint32_t EraseCounter;

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);

    /* erase the flash pages */
    for(EraseCounter = 0; EraseCounter < PageNum; EraseCounter++) {
        fmc_page_erase(FMC_WRITE_START_ADDR + (FMC_PAGE_SIZE * EraseCounter));
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

/*!
    \brief      program fmc word by word from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_program(uint8_t program_type)
{
    /* unlock the flash program/erase controller */
    fmc_unlock();

    address = FMC_WRITE_START_ADDR;

    /* program flash */
    while(address < FMC_WRITE_END_ADDR) {
        if(FMC_PROGRAM_TYPE_WORD == program_type) {
            fmc_word_program(address, data0);
            address += sizeof(uint32_t);
        } else if(FMC_PROGRAM_TYPE_FAST == program_type) {
            fmc_fast_program(address, data_buffer);
            address += DOUBLE_WORDS_CNT_IN_ROW * sizeof(uint64_t);
        }

        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
    }

    /* lock the main FMC after the program operation */
    fmc_lock();
}

/*!
    \brief      check fmc erase result
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_erase_pages_check(void)
{
    uint32_t i;
    uint32_t *ptrd;

    ptrd = (uint32_t *)FMC_WRITE_START_ADDR;

    /* check flash whether has been erased */
    for(i = 0; i < WordNum; i++) {
        if(0xFFFFFFFF != (*ptrd)) {
            break;
        } else {
            ptrd++;
        }
    }
}

/*!
    \brief      check fmc program result
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_program_check(uint8_t program_type)
{
    uint32_t i;

    if(FMC_PROGRAM_TYPE_WORD == program_type) {
        uint32_t *ptrd;
        ptrd = (uint32_t *)FMC_WRITE_START_ADDR;

        /* check flash whether has been programmed */
        for(i = 0; i < WordNum; i++) {
            if((*ptrd) != data0) {
                break;
            } else {
                ptrd++;
            }
        }
    } else if(FMC_PROGRAM_TYPE_FAST == program_type) {
        uint64_t *ptrd;
        ptrd = (uint64_t *)FMC_WRITE_START_ADDR;

        /* check flash whether has been programmed */
        for(i = 0; i < WordNum / 2; i++) {
            if((*ptrd) != data_buffer[i % DOUBLE_WORDS_CNT_IN_ROW]) {
                break;
            } else {
                ptrd++;
            }
        }
    }
}


uint32_t l81_fmc_read(uint32_t addr)
{
	uint32_t *ptrd = (uint32_t *)addr;

	return *ptrd;
}

void l81_fmc_read_words(uint32_t addr, uint8_t str[], uint8_t len)
{
	uint32_t *ptr = NULL;
	uint8_t ver;
	uint8_t i = 0u, j = 1u, ver_count = 0u;
	
	ptr = (uint32_t *)addr;    //get version address
	i = 0u;
	j = 1u;//Byte_MASK          (0xFF000000U) 
	while(*ptr & (Byte_MASK >> (i * 8u))){             // from the high byte to check every byte is '\0' or not
		// 4u: 1 word has 4 bytes, 8u: 1 byte has 8 bitsshiff high byte to the lowest byte and store as uint8_t type
		ver = (uint8_t)((*ptr & (Byte_MASK >> (i * 8u))) >> (4u - i - 1u) * 8u);    
		str[ver_count] = ver;                       //store in verison data buffer
		ver_count++;
		i++;
		if (i == 4u){                              //very word has 4 btyes, when i equals 4u, it means one word has scanned finished.
			ptr = (uint32_t *)(addr + (j * sizeof(uint32_t)));      //increase address, there are 4 word to store version data.
			i = 0u;
			j++;
			if (j > len)                       //if 4 word version address range was scanned finish, break loop.
				break;
		}
	}	
}

/*!
    \brief      program fmc word by word from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void l81_fmc_program(uint32_t addr, uint32_t data)
{		
	uint32_t backup[92u] = {'\0'};  // rjq-0706  76->92
	uint32_t backup_addr = USER_ADDR_START;
	uint32_t *ptr = (uint32_t *)USER_ADDR_START;
	uint8_t i = 0u;

	//step 1, backup original data
	while (backup_addr <= USER_ADDR_END){
		backup[i] = *((uint32_t *)backup_addr);
		//printf("backup %x, data: %x\n", backup_addr, backup[i] );
		backup_addr += sizeof(uint32_t);
		i++;
	}
	
	//step 2, erease pages
	  fmc_erase_pages();
    fmc_erase_pages_check();
	
	
	//step3, write back
    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* program flash */
    if(addr >= FMC_WRITE_START_ADDR && addr < FMC_WRITE_END_ADDR) {
			backup_addr = USER_ADDR_START;
			i = 0u;
			
			//there is bug here, please note!!!
			while (backup_addr <= USER_ADDR_END){
				if (backup_addr != addr){
					fmc_word_program(backup_addr, backup[i]);
				}	else{
					fmc_word_program(addr, data);
				}
				backup_addr += sizeof(uint32_t);
				i++;
			}
    }

    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
    
    /* lock the main FMC after the program operation */
    fmc_lock();
}

/*!
    \brief      check fmc program result
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint8_t l81_fmc_program_check(uint32_t addr, uint32_t data)
{
   uint32_t *ptrd = (uint32_t *)addr;

   /* check flash whether has been programmed */
   if((*ptrd) != data) {
		 return 0U;
   } else {
		 return 1U;
   }
}
#if 0
/*only be used for AT FMC write command, includes normal write, SN write and version write*/
static uint8_t _l81_fmc_write_str(uint32_t addr, char str[])
{
	uint8_t byte_count = 0u;
	uint8_t word_count = 0u;
	uint8_t byte_remainder = 0u;   // byte_cound % 4u.
	uint32_t data = 0u;
	char *p = NULL;  //to operate param[1]
	uint8_t i = 0u, j = 0u;
	
	//all data will be treated as string to store in flash, this will store ASCII in flash
	byte_count = strlen(str);         //param 1 store write data
	byte_remainder  = byte_count % 4u;
	if (0u != byte_remainder)
		word_count = (byte_count / 4u) + 1u;  //must 4 byte align
	else
		word_count = byte_count / 4u;
	
	p = str;
	
	j = 0u;
	data = 0u;
  for (i = 0u; i < byte_count; i++){
		if (j < 4u){                         //one word has 4 bytes
			data = (data << 8u) + *p++;        //every byte has 8 bits, store ascii directly
      j++;
    } else {
    	l81_fmc_program(addr, data);
	    if (!l81_fmc_program_check(addr, data)){
				printf("AT+RES,ACK\r\n");
		    printf("AT+RES,Err,write data 0x%x in addr 0x%x failed\r\n", data, addr);
				printf("AT+RES,end\r\n");
		    return 0U;
	    } 
      j = 0;
      i--;
      data = 0;
			addr += sizeof(uint32_t);  // increase addr
    }
  }
  if (byte_remainder){
		data <<= ((4u - byte_remainder) * 8u);
    l81_fmc_program(addr, data);
	  if (!l81_fmc_program_check(addr, data)){
			printf("AT+RES,ACK\r\n");
		  printf("AT+RES,Err,write data 0x%x in addr 0x%x failed\r\n", data, addr);
			printf("AT+RES,end\r\n");
		  return 0U;
	  } 		
  }
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,end\r\n");
	
	return 1u;
}
#endif
uint8_t l81_AT_FMC_W_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0u;
	uint8_t byte_count = 0u;
	uint8_t word_count = 0u;
	uint8_t byte_remainder = 0u;   // byte_cound % 4u.
	uint32_t data = 0u;
	char *p = NULL;  //to operate param[1]
	uint8_t i = 0u, j = 0u;
	
	
  ATcmd_split_params(params, param, &param_num);

	if (param_num != 2U){  //must have 2 parameters, addr,data
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
	
	uint32_t addr = String2Hex(param[0u]);  //param 0 store write address
	
	if (addr < USER_ADDR_START || addr > USER_ADDR_END) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong address: 0x%x! the right address range is [0x%x, 0x%x]\r\n", addr, USER_ADDR_START, USER_ADDR_END);
		printf("AT+RES,end\r\n");
		return 0U;
	}		
	
	//all data will be treated as string to store in flash, this will store ASCII in flash
	byte_count = strlen(param[1u]);         //param 1 store write data
	byte_remainder  = byte_count % 4u;
	if (0u != byte_remainder)
		word_count = (byte_count / 4u) + 1u;  //must 4 byte align
	else
		word_count = byte_count / 4u;
	
	p = param[1u];
	
	j = 0u;
	data = 0u;
  for (i = 0u; i < byte_count; i++){
		if (j < 4u){                         //one word has 4 bytes
			data = (data << 8u) + *p++;        //every byte has 8 bits, store ascii directly
      j++;
    } else {
    	l81_fmc_program(addr, data);
	    if (!l81_fmc_program_check(addr, data)){
				printf("AT+RES,ACK\r\n");
		    printf("AT+RES,Err,write data 0x%x in addr 0x%x failed\r\n", data, addr);
				printf("AT+RES,end\r\n");
		    return 0U;
	    } 
      j = 0;
      i--;
      data = 0;
			addr += sizeof(uint32_t);  // increase addr
    }
  }
  if (byte_remainder){
		data <<= ((4u - byte_remainder) * 8u);
    l81_fmc_program(addr, data);
	  if (!l81_fmc_program_check(addr, data)){
			printf("AT+RES,ACK\r\n");
		  printf("AT+RES,Err,write data 0x%x in addr 0x%x failed\r\n", data, addr);
			printf("AT+RES,end\r\n");
		  return 0U;
	  } 		
  }
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,end\r\n");
	
	return 1u;
}

uint8_t l81_AT_FMC_R_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint8_t version[16] = {'\0'};
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 1U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command only have one params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	uint32_t addr = String2Hex(param[0u]);  //param 0 store read address
	
	if (addr < USER_ADDR_START || addr > USER_ADDR_END) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong address: 0x%x! the right address range is [0x%x, 0x%x]\r\n", addr, USER_ADDR_START, USER_ADDR_END);
		printf("AT+RES,end\r\n");
		return 0U;
	}	
	
	switch (addr){
		case ACCX_ADDR:
			l81_fmc_read_words(ACCX_ADDR, version, Acc_len);
			break;
		case ACCY_ADDR:
			l81_fmc_read_words(ACCY_ADDR, version, Acc_len);
			break;
		case ACCZ_ADDR:
			l81_fmc_read_words(ACCZ_ADDR, version, Acc_len);
			break;	
		case GYROX_ADDR:
			l81_fmc_read_words(GYROX_ADDR, version, Gyro_len);
			break;	
		case GYROY_ADDR:
			l81_fmc_read_words(GYROY_ADDR, version, Gyro_len);
			break;	
		case GYROZ_ADDR:
			l81_fmc_read_words(GYROZ_ADDR, version, Gyro_len);
			break;	
		case TOF_ADDR:
			l81_fmc_read_words(TOF_ADDR, version, Tof_len);
			break;
		case Motor1_ADDR:
			l81_fmc_read_words(Motor1_ADDR, version, Motor_len);
			break;
		case Motor2_ADDR:
			l81_fmc_read_words(Motor2_ADDR, version, Motor_len);
			break;
		case Motor3_ADDR:
			l81_fmc_read_words(Motor3_ADDR, version, Motor_len);
			break;
		case Motor4_ADDR:
			l81_fmc_read_words(Motor4_ADDR, version, Motor_len);
			break;
		case Motor5_ADDR:
			l81_fmc_read_words(Motor5_ADDR, version, Motor_len);
			break;
		case Motor6_ADDR:
			l81_fmc_read_words(Motor6_ADDR, version, Motor_len);
			break;
		case SN_ADDR:
			l81_fmc_read_words(SN_ADDR, version, Ver_len);
			break;		
		case Ver_ADDR:
			l81_fmc_read_words(Ver_ADDR, version, Ver_len);
			break;			
		default:
			printf("AT+RES,ACK\r\n");
			printf("AT+RES,Err,input a wrong address\r\n");
			printf("AT+RES,end\r\n");
			return 0u;
		  break;
	}
	
	if (version[0] == 0xFF){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,addr 0x%x has not data\r\n", addr);
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,%s\r\n", version);
	printf("AT+RES,end\r\n");

    return 1U;	
}
#if 0
uint8_t l81_AT_VerW_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0u;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 1U){  //must have 1 parameters, version number
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
	
	return _l81_fmc_write_str(Ver_ADDR, param[1]);
}
#endif
uint8_t l81_AT_VerR_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint8_t version[16] = {'\0'};   // 4 words, so there are 4 * 4 bytes = 16 bytes
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	l81_fmc_read_words(Ver_ADDR, version, Ver_len);
	
	if (version[0] == 0xFF){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,no version data\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,%s\r\n", version);
	printf("AT+RES,end\r\n");

    return 1U;			
}
#if 0
uint8_t l81_AT_SNW_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0u;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 1U){  //must have 1 parameters, version number
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
	
	return _l81_fmc_write_str(SN_ADDR, param[1]);
}
#endif

uint8_t l81_AT_SNR_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint8_t version[16] = {'\0'};   // 4 words, so there are 4 * 4 bytes = 16 bytes
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	l81_fmc_read_words(SN_ADDR, version, Ver_len);
	
	if (version[0] == 0xFF){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,no SN data\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,%s\r\n", version);
	printf("AT+RES,end\r\n");

    return 1U;			
}

uint8_t l81_AT_FMC_get_user_addr_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint8_t version[16] = {'\0'};   // 4 words, so there are 4 * 4 bytes = 16 bytes
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,ACCX_ADDR:0x%x\r\n",     ACCX_ADDR);
  printf("AT+RES,ACCY_ADDR:0x%x\r\n",     ACCY_ADDR);
	printf("AT+RES,ACCZ_ADDR:0x%x\r\n",     ACCZ_ADDR);
	printf("AT+RES,GYROX_ADDR:0x%x\r\n",    GYROX_ADDR);
	printf("AT+RES,GYROY_ADDR:0x%x\r\n",    GYROY_ADDR);
	printf("AT+RES,GYROZ_ADDR:0x%x\r\n",    GYROZ_ADDR);
	printf("AT+RES,TOF_ADDR:0x%x\r\n",      TOF_ADDR);
	printf("AT+RES,Motor1_ADDR:0x%x\r\n",      Motor1_ADDR);
	printf("AT+RES,Motor2_ADDR:0x%x\r\n",      Motor2_ADDR);
	printf("AT+RES,Motor3_ADDR:0x%x\r\n",      Motor3_ADDR);
	printf("AT+RES,Motor4_ADDR:0x%x\r\n",      Motor4_ADDR);
	printf("AT+RES,Motor5_ADDR:0x%x\r\n",      Motor5_ADDR);
	printf("AT+RES,Motor6_ADDR:0x%x\r\n",      Motor6_ADDR);
	printf("AT+RES,SN_ADDR:0x%x\r\n",       SN_ADDR);
	printf("AT+RES,Ver_ADDR:0x%x\r\n",      Ver_ADDR);
	printf("AT+RES,USER_ADDR_END:0x%x\r\n", USER_ADDR_END);
	printf("AT+RES,end\r\n");

    return 1U;			
}

uint8_t l81_AT_FMC_dump_func(char params[])
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
	uint32_t addr = USER_ADDR_START;
	for( ; addr < USER_ADDR_END; addr += sizeof(uint32_t)){
		printf("AT+RES,addr:0x%x, data:0x%x\r\n", addr, *((uint32_t *)addr));
	}
	printf("AT+RES,end\r\n");
    return 1U;			
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int fmc_test(void)
{
	uint32_t data = 0u;
#if 0	
    /* step1: erase pages and check if it is successful. If not, light the LED1. */
    fmc_erase_pages();
    fmc_erase_pages_check();
	
		printf("fmc test write 0x12345678\n");
		l81_fmc_program(ACCX_ADDR, 0x12345678);
		l81_fmc_program_check(ACCX_ADDR, 0x12345678);
		data = l81_fmc_read(ACCX_ADDR);
		printf("fmc test read data %x\n", data);
	
	 fmc_erase_pages();
    fmc_erase_pages_check();
	
	printf("fmc test write 0x12345679\n");
		l81_fmc_program(ACCX_ADDR, 0x12345679);
		l81_fmc_program_check(ACCX_ADDR, 0x12345679);
		data = l81_fmc_read(ACCX_ADDR);
		printf("fmc test read data %x\n", data);

printf("%x\n", ACCX_ADDR);
printf("%x\n", ACCY_ADDR);
printf("%x\n", ACCZ_ADDR);
printf("%x\n", GYROX_ADDR);
printf("%x\n", GYROY_ADDR);
printf("%x\n", GYROZ_ADDR);
printf("%x\n", TOF_ADDR);
printf("%x\n", Ver_ADDR);
printf("%x\n", USER_ADDR_END);

#endif
    /* step2: program and check if it is successful. If not, light the LED2. */
//    fmc_program(FMC_PROGRAM_TYPE_WORD);
//    fmc_program_check(FMC_PROGRAM_TYPE_WORD);

    /* step3: erase pages and check if it is successful. If not, light the LED1. */
//    fmc_erase_pages();
//    fmc_erase_pages_check();

    /* step4: fast program and check if it is successful. If not, light the LED2. */
//   fmc_program(FMC_PROGRAM_TYPE_FAST);
//    fmc_program_check(FMC_PROGRAM_TYPE_FAST);
	
	return 1u;
}