/*!
    \file    L81_ota.c
    \brief   the basical interface for update MCU firmware
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
#include "L81_ota.h"

#define FALSE 0u
#define TRUE  1u

/*flash information*/
#define ota_PAGE_SIZE                ((uint16_t)0x1000U)
#define A_Partition_START_ADDR         ((uint32_t)0x0800D000U)
#define A_Partitoin_END_ADDR           ((uint32_t)0x08022FFFU)
#define B_Partition_START_ADDR         ((uint32_t)0x08024000U)
#define B_Partitoin_END_ADDR           ((uint32_t)0x08039FFFU)

/* calculate the number of page to be programmed/erased */
uint32_t A_Partition_PageNum = (A_Partitoin_END_ADDR - A_Partition_START_ADDR + 1u) / ota_PAGE_SIZE;
/* calculate the number of page to be programmed/erased */
uint32_t A_Partiton_WordNum = ((A_Partitoin_END_ADDR - A_Partition_START_ADDR + 1u) >> 2u);

/* calculate the number of page to be programmed/erased */
uint32_t B_Partition_PageNum = (B_Partitoin_END_ADDR - B_Partition_START_ADDR + 1u) / ota_PAGE_SIZE;
/* calculate the number of page to be programmed/erased */
uint32_t B_Partiton_WordNum = ((B_Partitoin_END_ADDR - B_Partition_START_ADDR + 1u) >> 2u);

volatile static HexLineInfo_t line;

volatile OTA_upgrade_status_e ota_upgrade = upgrade_idle;
#if 0
void l81_read_flash(void)
{
	uint32_t addr = 0x0800D000u;
	
	for( ; addr < 0x0800D100u; addr += sizeof(uint32_t)){
		printf("addr:0x%x, data:0x%x\r\n", addr, *((uint32_t *)addr));
	}
}


/*!
    \brief      erase fmc pages from OTA_WRITE_START_ADDR to OTA_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ota_erase_pages(uint32_t start_addr, uint32_t page_num)
{
    uint32_t EraseCounter;

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);

    /* erase the flash pages */
    for(EraseCounter = 0; EraseCounter < page_num; EraseCounter++) {
        fmc_page_erase(start_addr + (ota_PAGE_SIZE * EraseCounter));
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}
/*!
    \brief      check fmc erase result
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ota_erase_pages_check(uint32_t start_addr, uint32_t page_num)
{
    uint32_t i;
    uint32_t *ptrd;

    ptrd = (uint32_t *)start_addr;

    /* check flash whether has been erased */
    for(i = 0u; i < page_num; i++) {
        if(0xFFFFFFFFu != (*ptrd)) {
            break;
        } else {
            ptrd++;
        }
    }
}

uint8_t ota_check_upgrade_flag(void)
{
	uint32_t *ptrd = (uint32_t *)OTA_FLAG_ADDR;
	
	if ((*ptrd) == OTA_FLAG_FLAG)
		return TRUE;
	else
		return FALSE;
}

void ota_clean_upgrade_flag(void)
{
	uint32_t *ptrd = NULL;
	uint32_t boot_option_bak = 0u;
	
	//1. store boot partition
	boot_option_bak = *((uint32_t *)boot_option_addr);

	//2. erase pages
  /* unlock the flash program/erase controller */
  fmc_unlock();

  /* clear all pending flags */
  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);

  /* erase the flash pages */
  fmc_page_erase(OTA_FLAG_ADDR);
  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);

  /* lock the main FMC after the erase operation */
  fmc_lock();		
	
	//to make sure flash internal sync
	delay_1ms(2);
	
	//3. restore boot optition flag
	fmc_unlock();
	
	//restore bootpartition
	fmc_word_program(boot_option_addr, boot_option_bak);
	
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
    
  /* lock the main FMC after the program operation */
  fmc_lock();		
}

int8_t ota_check_command_valid(void)
{
	char *cmd = (char *)com_rxbuffer;
	
	char start_code = *cmd;               //the first charactor is should be ':'
	
	if (start_code == ':')
		return TRUE;
	else
		return FALSE;
}

uint8_t ota_checksum(void)
{
	uint16_t sum = 0u;
	uint16_t i = 0u;
	
	for (i = 0u; i < line.data_len; i++){
		sum += line.data[i];
	}
	sum += line.data_len;
	sum += (line.data_addr >> 8u);
	sum += (line.data_addr & 0x00FFu);
	sum += line.cmd_type;
	
	printf("checksum %x\n", (((~sum) & 0xffu)+ 1u));
	
	if (line.checksum == ((((~sum) & 0xffu)+ 1u) & 0x0ffu))
		return TRUE;
	else
		return FALSE;
}

uint8_t ota_get_line_info(void)
{
	uint8_t i = 0u, j = 0u;
	char tmp_str[5u];
	char hex_data[40u] = {'\0'};
	char *data_p = NULL;
	
	char *rx_p = (char *)com_rxbuffer;
	
	if (!ota_check_command_valid())
		return FALSE;
	
//	printf("get line: %s\n", rx_p);
	
	memset((void *)tmp_str, '\0', sizeof(tmp_str));
	memcpy((void *)tmp_str, (void *)(rx_p + 1u), 2u);
	line.data_len = String2Hex(tmp_str);
	
	memset((void *)tmp_str, '\0', sizeof(tmp_str));
	memcpy((void *)tmp_str, (void *)(rx_p + 3u), 4u);
	line.data_addr = String2Hex(tmp_str);
		
	memset((void *)tmp_str, '\0', sizeof(tmp_str));
	memcpy((void *)tmp_str, (void *)(rx_p + 7u), 2u);
	line.cmd_type = String2Hex(tmp_str);
	
	memset((void *)tmp_str, '\0', sizeof(tmp_str));
	memcpy((void *)tmp_str, (void *)(rx_p + line.data_len * 2u + 9u), 2u);
	line.checksum = String2Hex(tmp_str);
	
	memset((void *)hex_data, '\0', sizeof(hex_data));
	memcpy((void *)hex_data, (void *)(rx_p + 9u), line.data_len * 2u);
	
	printf("line len: %x, addr: %x, type: %x, checksum: %x\n", line.data_len, line.data_addr, line.cmd_type, line.checksum);

//	printf("data %s\n", hex_data);
	
	data_p = hex_data;
	
	for(i = 0u; i < line.data_len; i++){
		memset((void *)tmp_str, '\0', sizeof(tmp_str));
		for (j = 0u; j < 2u; j++){
			tmp_str[j] = *(data_p + j);
		}
		line.data[i] = String2Hex(tmp_str);
		data_p = data_p + 2u;
		
		printf("%x ", line.data[i]);
	}
	printf("\n");
	
	if (ota_checksum())
		return TRUE;
	else
		return FALSE;
}

//linear offset from firmware hex file. line.cmd_type whill indicate this.
volatile static uint32_t linear_offset_addr = 0u;
//indicate which partition will be upgraded by ota, it will be set in AT+MOTA command.
volatile uint32_t partition_base_addr = 0u;

OTA_Flash_status_e ota_flash_hex_line(void)
{
	uint32_t tmp_addr = line.data_addr; //store offset base address in every line
	uint32_t data = 0u;
	uint8_t byte_index = 0u;
	uint8_t i = 0u, flash_failed = 0u;
	OTA_Flash_status_e finish = NONE;
	
	uint32_t *ptrd = NULL;
	
	//according cmd type 04 to get base address
	if(line.cmd_type == 0x04u){
		linear_offset_addr = 0u;
		linear_offset_addr |= line.data[0] << 24u;
		linear_offset_addr |= line.data[1] << 16u;
		printf("flash_base_addr %x\n", linear_offset_addr);
	}
	
	if(line.cmd_type == 0x01u){
		finish = hex_end;
		printf("finish ota update\n");
	}
	
	i = 0u;
	data = 0u;
	if (line.cmd_type == 0x00u){
		for(byte_index = 0u; byte_index < line.data_len; byte_index++){
			data |= line.data[byte_index] << (8u * i);
			i++;
			if (i == 4u){
				printf("data %x will write in addr %x\n", data, (linear_offset_addr + tmp_addr));
				i = 0u;
				fmc_word_program((linear_offset_addr + tmp_addr), data);
				
				//check 
				ptrd = (uint32_t *)(linear_offset_addr + tmp_addr);

				if((*ptrd) != data) {
					printf("flash wrong addr %x, data %x\n", (linear_offset_addr + tmp_addr), data);
					return flash_err;
				}
				
				tmp_addr += sizeof(uint32_t); //increase 4bytes
				data = 0u;
			}
		}
	}
	
	if (finish == hex_end)
		return hex_end;
	else
		return line_end;
}
#endif
uint8_t l81_AT_OTA_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0u;
  uint32_t i;
  uint32_t *ptrd;
	uint32_t boot_option_bak = 0u;
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command hava no params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}

	//1. erase pages
  /* unlock the flash program/erase controller */
  fmc_unlock();

  /* clear all pending flags */
  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);

  /* erase the flash pages */
  fmc_page_erase(OTA_FLAG_ADDR);
  fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);

  /* lock the main FMC after the erase operation */
  fmc_lock();	
	
	//2. check erase
  ptrd = (uint32_t *)OTA_FLAG_ADDR;

  /* check flash whether has been erased, ota area only has one page*/
  for(i = 0u; i < 1u; i++) {
		if(0xFFFFFFFFu != (*ptrd)) {
			printf("AT+RES,ACK\r\n");
			printf("AT+RES,Err,write ota flag failed\r\n");
			printf("AT+RES,end\r\n");			
			return 0u;
    } else {
      ptrd++;
    }
  }	
	
	//3. write ota flag
	fmc_unlock();
	
	//set ota upgrade flag
	fmc_word_program(OTA_FLAG_ADDR, OTA_FLAG_FLAG);
	
	fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
    
  /* lock the main FMC after the program operation */
  fmc_lock();
	
	//4. check write data
	ptrd = (uint32_t *)OTA_FLAG_ADDR;
	if ((*ptrd) != OTA_FLAG_FLAG){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,write ota flag failed\r\n");
		printf("AT+RES,end\r\n");			
		return 0u;
	}	else {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,OTA upgrade ready\r\n");	
		printf("AT+RES,end\r\n");
		delay_1ms(500);
		NVIC_SystemReset();
	  return 1U;
	}
}

uint8_t l81_AT_get_sys_status_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0u;
  uint32_t i;
  uint32_t *ptrd;
	char bootp;
	
	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command hava no params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}

	printf("AT+RES,ACK\r\n");
	printf("AT+RES,app\r\n");
	printf("AT+RES,end\r\n");			
	return 0u;
}