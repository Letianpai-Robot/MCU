
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VI530x_ALGORITHM_H
#define __VI530x_ALGORITHM_H

/* Includes ------------------------------------------------------------------*/
#include "VI530x_User_Handle.h"
#include "stdlib.h"
#include "math.h"


#define PILEUP_A (9231000)
#define PILEUP_B (4896)
#define PILEUP_C (1922)
#define PILEUP_D (10)



int16_t VI530x_YQ20_Calculate_Pileup_Bias(uint32_t peak1, uint32_t noise, uint32_t integral_times);
float VI530x_V10_Calculate_Pileup_Bias(uint32_t peak2, uint32_t noise, uint32_t integral_times);

uint8_t VI530x_Calculate_Confidence(uint16_t noise,uint32_t peak1,uint32_t integral_times);

uint8_t VI530x_Calculate_Xtalk_Ratio(int16_t tof, uint16_t count, uint32_t noise, uint8_t confidence, uint32_t peak);
uint8_t VI530x_Calibration_Xtalk_Ratio(uint8_t xtalk_ratio);


#endif 



