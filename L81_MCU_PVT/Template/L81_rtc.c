/*!
    \file    L81_cliff.c
    \brief   the basical interface for fall sensor (ITR1502SR40A)
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

#include <stdlib.h>
#include <stdio.h>
#include "gd32l23x_rtc.h"
#include "L81_rtc.h"
#include "L81_AT.h"
#include "systick.h"

#define BKP_VALUE    0x32F0
uint32_t RTCSRC_FLAG = 0;

/*!
    \brief      set the datetime to RTC
    \param[in]  decimal;year,month,date.hour,min,sec
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
uint8_t l81_AT_RTC_setdatetime_func(char params[])
{
	rtc_parameter_struct   rtc_initpara;
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	
	ATcmd_split_params(params, param, &param_num);

	
	if (param_num == 6U){
				rcu_periph_clock_enable(RCU_PMU);
				rcu_periph_clock_enable(RCU_BKP);
			pmu_backup_write_enable();
			/* get RTC clock entry selection */
			RTCSRC_FLAG = GET_BITS(RCU_BDCTL, 8, 9);
				rcu_osci_on(RCU_LXTAL);
			rcu_osci_stab_wait(RCU_LXTAL);
			rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);
			rcu_periph_clock_enable(RCU_RTC);
			rtc_register_sync_wait();

			/* check if RTC has aready been configured */
			//if((BKP_VALUE != RTC_BKP0) || (0x00 == RTCSRC_FLAG)) {
			rtc_initpara.factor_asyn = 0x7F;
			rtc_initpara.factor_syn = 0xFF;
			if (atoi(param[0])>=0 && atoi(param[0])<100)
				{
				rtc_initpara.year = (((atoi(param[0])/10)<<4) + (atoi(param[0])%10));//0x17;
				}else
					{
					printf("AT+RES,ACK\r\n");
					printf("AT+RES,Err,param0 error\r\n");
					printf("AT+RES,end\r\n");
					return 0U;
					}
			
				if (atoi(param[1])>0 && atoi(param[1])<13)
					{
					rtc_initpara.month = (((atoi(param[1])/10)<<4) + (atoi(param[1])%10));
					}else
						{
						printf("AT+RES,ACK\r\n");
						printf("AT+RES,Err,param1 error\r\n");
						printf("AT+RES,end\r\n");
						return 0U;
						}
		
					if (atoi(param[2])>0 && atoi(param[2])<32)
						{
						rtc_initpara.date = (((atoi(param[2])/10)<<4) + (atoi(param[2])%10));
						}else
							{
							printf("AT+RES,ACK\r\n");
							printf("AT+RES,Err,param2 error\r\n");
							printf("AT+RES,end\r\n");
							return 0U;
							}
			rtc_initpara.display_format = RTC_24HOUR;
			rtc_initpara.am_pm = RTC_AM;

			
			if (atoi(param[3])>=0 && atoi(param[3])<24)
				{
				rtc_initpara.hour = (((atoi(param[3])/10)<<4) + (atoi(param[3])%10));
				}else
					{
					printf("AT+RES,ACK\r\n");
					printf("AT+RES,Err,param3 error\r\n");
					printf("AT+RES,end\r\n");
					return 0U;
					}

				
				if (atoi(param[4])>=0 && atoi(param[4])<60)
					{
					rtc_initpara.minute = (((atoi(param[4])/10)<<4) + (atoi(param[4])%10));
					}else
						{
						printf("AT+RES,ACK\r\n");
						printf("AT+RES,Err,param4 error\r\n");
						printf("AT+RES,end\r\n");
						return 0U;
						}
					
					if (atoi(param[5])>=0 && atoi(param[5])<60)
						{
						rtc_initpara.second = (((atoi(param[5])/10)<<4) + (atoi(param[5])%10));
						}else
							{
							printf("AT+RES,ACK\r\n");
							printf("AT+RES,Err,param5 error\r\n");
							printf("AT+RES,end\r\n");
							return 0U;
							}
						
		
			/* RTC current time configuration */
			if(ERROR == rtc_init(&rtc_initpara)) {
				;//printf("\n\r RTC time configuration failed!  %d\n\r",ERROR);
			} else {
				//printf("\n\r** RTC time configuration success! **\n\r");
				RTC_BKP0 = 0x32F0;
			}
			rcu_all_reset_flag_clear();
			
	delay_1ms(300);
		
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}else
	{
							printf("AT+RES,ACK\r\n");
							printf("AT+RES,Err,param error\r\n");
							printf("AT+RES,end\r\n");
	}

    return 1U;		
}

/*!
    \brief      get the datetime from RTC
    \param[in]  decimal;year,month,date.hour,min,sec
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
uint8_t l81_AT_RTC_getdatetime_func(char params[])
{
	rtc_parameter_struct   rtc_initpara;
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	
	ATcmd_split_params(params, param, &param_num);

					rcu_periph_clock_enable(RCU_PMU);
				rcu_periph_clock_enable(RCU_BKP);
			pmu_backup_write_enable();
			/* get RTC clock entry selection */
			RTCSRC_FLAG = GET_BITS(RCU_BDCTL, 8, 9);
				rcu_osci_on(RCU_LXTAL);
			rcu_osci_stab_wait(RCU_LXTAL);
			rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);
			rcu_periph_clock_enable(RCU_RTC);
			rtc_register_sync_wait();
		
			/* check if RTC has aready been configured */
			if((BKP_VALUE != RTC_BKP0) || (0x00 == RTCSRC_FLAG)) {;
		} else {
				// detect the reset source 
				if(RESET != rcu_flag_get(RCU_FLAG_PORRST)) {
					;//printf("power on reset occurred....\n\r");
				} else if(RESET != rcu_flag_get(RCU_FLAG_EPRST)) {
					;//printf("external reset occurred....\n\r");
				}
				//printf("no need to configure RTC....\n\r");
			}
		
			rcu_all_reset_flag_clear();
	    rtc_current_time_get(&rtc_initpara);
			delay_1ms(300);

//printf("Current time: %0.2x/%0.2x/%0.2x %0.2x:%0.2x:%0.2x \n\r", \
  //         rtc_initpara.year, rtc_initpara.month, rtc_initpara.date,rtc_initpara.hour, rtc_initpara.minute, rtc_initpara.second);
	if (param_num == 0U){
		printf("AT+RES,ACK\r\n");
		

		printf("AT+RES,rtc: %0.2x/%0.2x/%0.2x %0.2x:%0.2x:%0.2x\r\n", \
           rtc_initpara.year, rtc_initpara.month, rtc_initpara.date,rtc_initpara.hour, rtc_initpara.minute, rtc_initpara.second);
		//printf("AT+RES,rtc,%d/%d/%d %d/%d/%d\r\n",((rtc_initpara.year>>4)*10 + (rtc_initpara.year&0x0f)), 
		//((rtc_initpara.month>>4)*10 + (rtc_initpara.month&0x0f)),((rtc_initpara.date>>4)*10 + (rtc_initpara.date&0x0f)),
		//((rtc_initpara.hour>>4)*10 + (rtc_initpara.hour&0x0f)),((rtc_initpara.minute>>4)*10 + (rtc_initpara.minute&0x0f)),
		//((rtc_initpara.second>>4)*10 + (rtc_initpara.second&0x0f)));
		printf("AT+RES,end\r\n");
		return 0U;
	}

    return 1U;		
}

