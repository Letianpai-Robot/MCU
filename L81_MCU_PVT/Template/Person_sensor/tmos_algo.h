#ifndef __TMOS_ALGO_H__
#define __TMOS_ALGO_H__
#include <stdint.h>
#include <stddef.h>

typedef struct{
	uint8_t p_flag;  //presence_flag
	int16_t tobj;
}tmos_algo_sw_input_t;
typedef struct{
	int16_t rt_max_tobj;
	int16_t frozen_max_tobj;
	int16_t frozen_min_tobj;
	uint8_t recovery_flag;
	uint8_t sw_p_flag;
}tmos_algo_sw_output_t;

typedef struct{
	int16_t odr;
	int16_t presence_frozen_exit_thr;
	uint16_t tobj_human_radiation_thr;
}tmos_algo_sw_params_t;

void  tmos_algo_sw_init(tmos_algo_sw_params_t *params);
void tmos_algo_sw_processing(tmos_algo_sw_input_t* in, tmos_algo_sw_output_t* out);
#endif