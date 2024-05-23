

#include <stdio.h>
#include "KalmanFilter.h"

#define KALMAN_FILTER_DEBUG_EN            0     //debug EN=1 DEN=0
#if KALMAN_FILTER_DEBUG_EN
#define GyroOulaLog(...) printf(__VA_ARGS__)
#else
#define GyroOulaLog(...) 
#endif
#include <math.h>

//define q r
#define Q 0.001//0.001
#define R 0.0001//0.0001

//
#define SCOPE 50
float Accumulated_Error = 1;
float kalman_adc_old = 0;

float kalman_x(float ADC_Value) 
{
  float Old_Input;
  float Old_Error_All;
  float H;
  float kalman_adc;

  if (fabs(ADC_Value - kalman_adc_old) / SCOPE > 0.25) 
  {
    Old_Input = ADC_Value * 0.382 + kalman_adc_old * 0.618;
  }
  else 
  {
    Old_Input = kalman_adc_old;
  }

  Old_Error_All = sqrt(pow(Accumulated_Error, 2) + pow(Q, 2));

  H = pow(Old_Error_All, 2) / (pow(Old_Error_All, 2) + pow(R, 2));

  kalman_adc = Old_Input + H * (ADC_Value - Old_Input);

  Accumulated_Error = sqrt(pow(1 - H, 2) * pow(Old_Error_All, 2));

  kalman_adc_old = kalman_adc;

  return kalman_adc;
}

void kalman_x_init(void)
{
  Accumulated_Error = 1;
  kalman_adc_old = 0;
}


#define Q_y 0.001//0.001
#define R_y 0.0001//0.0001
float Accumulated_Error_y = 1;
float kalman_adc_old_y = 0;

float kalman_y(float ADC_Value) 
{
  float Old_Input;
  float Old_Error_All;
  float H;
  float kalman_adc;

  if (fabs(ADC_Value - kalman_adc_old_y) / SCOPE > 0.25) 
  {
    Old_Input = ADC_Value * 0.382 + kalman_adc_old_y * 0.618;
  }
  else 
  {
    Old_Input = kalman_adc_old_y;
  }

  Old_Error_All = sqrt(pow(Accumulated_Error_y, 2) + pow(Q_y, 2));

  H = pow(Old_Error_All, 2) / (pow(Old_Error_All, 2) + pow(R_y, 2));

  kalman_adc = Old_Input + H * (ADC_Value - Old_Input);

  Accumulated_Error_y = sqrt(pow(1 - H, 2) * pow(Old_Error_All, 2));

  kalman_adc_old_y = kalman_adc;

  return kalman_adc;
}

void kalman_y_init(void)
{
  Accumulated_Error_y = 1;
  kalman_adc_old_y = 0;
}