#include <math.h>
#include "tmos_algo.h"

#define SYSTEM_EXTERNAL_POWER_MODE

#define ST_ABS(x) (((x) > 0.f) ? (x) : -(x))  

#define TOBJ_FIFO_LEN	30
#define TOBJ_BUF_LEN		300
#define TOBJ_BUF_LEN_GAIN	10

typedef struct{
	uint8_t first_enter;
	uint8_t presence_flag;
	uint16_t frozen_max_tobj_cnt;
	uint16_t frozen_max_tobj_cnt_thr;
	uint16_t tobj_human_radiation_thr;
	int16_t presence_frozen_exit_thr;
	int16_t tobj_buf_idx;
	int16_t tobj_buf_len;
	int16_t tobj_buf[TOBJ_BUF_LEN];
	int16_t rt_tobj;
	int16_t rt_max_tobj;
	int16_t frozen_max_tobj;
	int16_t frozen_min_tobj;
	int16_t frozen_mm_delta;
#ifdef SYSTEM_EXTERNAL_POWER_MODE
	int16_t tobj_fifo_idx;
	int16_t tobj_fifo_len;
	int16_t tobj_fifo[TOBJ_FIFO_LEN];
#endif
}tmos_recovery_task_t;
tmos_recovery_task_t tmos_recovery_task;

tmos_algo_sw_output_t tmos_algo_sw_output;

#ifdef SYSTEM_EXTERNAL_POWER_MODE
static void inner_fifo_fill(int16_t rt_obj)
{
	int16_t ii;
	if(tmos_recovery_task.tobj_fifo_idx < tmos_recovery_task.tobj_fifo_len)
	{
		tmos_recovery_task.tobj_fifo[tmos_recovery_task.tobj_fifo_idx] = rt_obj;
		tmos_recovery_task.tobj_fifo_idx++;
	}
	else
	{
		for(ii = 0; ii < (tmos_recovery_task.tobj_fifo_len-1); ii++)
		{
			tmos_recovery_task.tobj_fifo[ii] = tmos_recovery_task.tobj_fifo[ii+1];
		}
		tmos_recovery_task.tobj_fifo[ii] = rt_obj;
	}
	
}
#endif


void  tmos_algo_sw_init(tmos_algo_sw_params_t *params)
{
	tmos_recovery_task.first_enter = 1;
	tmos_recovery_task.tobj_human_radiation_thr = params->tobj_human_radiation_thr; 
	tmos_recovery_task.frozen_max_tobj_cnt = 0;
	tmos_recovery_task.frozen_max_tobj_cnt_thr = ((params->odr * TOBJ_BUF_LEN_GAIN > TOBJ_BUF_LEN) ? TOBJ_BUF_LEN : (params->odr * TOBJ_BUF_LEN_GAIN));;
	tmos_recovery_task.tobj_buf_len = tmos_recovery_task.frozen_max_tobj_cnt_thr/2;
	tmos_recovery_task.presence_frozen_exit_thr = params->presence_frozen_exit_thr;
	tmos_recovery_task.tobj_buf_idx = 0;
	tmos_algo_sw_output.recovery_flag = 0;
#ifdef SYSTEM_EXTERNAL_POWER_MODE
	tmos_recovery_task.tobj_fifo_idx = 0;
	tmos_recovery_task.tobj_fifo_len = ((params->odr > TOBJ_FIFO_LEN) ? TOBJ_FIFO_LEN : params->odr);
	
#endif
}


void tmos_algo_health_detection()
{
	int16_t i;

	if((tmos_recovery_task.presence_flag == 0) || (tmos_recovery_task.first_enter == 1))
	{
		tmos_recovery_task.first_enter = 0;
		tmos_algo_sw_output.recovery_flag = 0;
		tmos_recovery_task.frozen_max_tobj_cnt = 0;
		tmos_recovery_task.tobj_buf_idx = 0;
		tmos_recovery_task.rt_max_tobj = tmos_recovery_task.rt_tobj;
		tmos_recovery_task.frozen_max_tobj = tmos_recovery_task.frozen_min_tobj = tmos_recovery_task.rt_tobj;
	#ifdef SYSTEM_EXTERNAL_POWER_MODE
		inner_fifo_fill(tmos_recovery_task.rt_tobj);
		tmos_recovery_task.frozen_min_tobj = tmos_recovery_task.tobj_fifo[0];
	#endif
	}
	else if(tmos_recovery_task.presence_flag == 1)
	{
		if(tmos_recovery_task.frozen_max_tobj_cnt < tmos_recovery_task.frozen_max_tobj_cnt_thr)
		{
			if(tmos_recovery_task.frozen_max_tobj < tmos_recovery_task.rt_tobj)
			{
				tmos_recovery_task.frozen_max_tobj = tmos_recovery_task.rt_tobj;
			}
			tmos_recovery_task.frozen_max_tobj_cnt++;
			if((tmos_recovery_task.frozen_max_tobj - tmos_recovery_task.frozen_min_tobj) < tmos_recovery_task.tobj_human_radiation_thr)
			{
				tmos_recovery_task.frozen_max_tobj_cnt--;
			}
			tmos_recovery_task.frozen_mm_delta = tmos_recovery_task.frozen_max_tobj - tmos_recovery_task.frozen_min_tobj;
		}
		
		if(tmos_recovery_task.rt_max_tobj < tmos_recovery_task.rt_tobj)
		{
			tmos_recovery_task.rt_max_tobj = tmos_recovery_task.rt_tobj;
		}

		if((tmos_recovery_task.rt_max_tobj - tmos_recovery_task.rt_tobj) > tmos_recovery_task.frozen_mm_delta*0.9)
		{
			tmos_recovery_task.tobj_buf[tmos_recovery_task.tobj_buf_idx] = tmos_recovery_task.rt_tobj;
			if(tmos_recovery_task.tobj_buf_idx < tmos_recovery_task.tobj_buf_len)
			{
				tmos_recovery_task.tobj_buf_idx++;
			}
		}
		else
		{
			tmos_recovery_task.tobj_buf_idx = 0;
		}
		if(tmos_recovery_task.tobj_buf_idx >= tmos_recovery_task.tobj_buf_len)
		{
			tmos_recovery_task.tobj_buf_idx = 0;
			for(i=1; i<tmos_recovery_task.tobj_buf_len; i++)
			{
				if(ST_ABS(tmos_recovery_task.tobj_buf[0] - tmos_recovery_task.tobj_buf[i]) > tmos_recovery_task.presence_frozen_exit_thr)
				{
					break;
				}
			}
			if(i>=tmos_recovery_task.tobj_buf_len)
			{
				tmos_algo_sw_output.recovery_flag = 1;
				tmos_recovery_task.first_enter = 1;
			}
			else
			{
				tmos_algo_sw_output.recovery_flag = 0;
			}
		}
	}
}

void tmos_algo_sw_processing(tmos_algo_sw_input_t* in, tmos_algo_sw_output_t* out)
{
	tmos_recovery_task.presence_flag = in->p_flag;
	tmos_recovery_task.rt_tobj = in->tobj;

	tmos_algo_health_detection();
	out->recovery_flag = tmos_algo_sw_output.recovery_flag;
	out->frozen_max_tobj = tmos_recovery_task.frozen_max_tobj;
	out->frozen_min_tobj = tmos_recovery_task.frozen_min_tobj;
	out->rt_max_tobj = tmos_recovery_task.rt_max_tobj;
}

