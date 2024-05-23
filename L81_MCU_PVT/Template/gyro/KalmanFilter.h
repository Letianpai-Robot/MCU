

#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include <stdint.h>
//#include <stddef.h>s

//#include "main.h"
//#include "gd32l23x_i2c.h"
//#include "gd32l23x_gpio.h"

/**********************************************************/
//PORT DEFINE


/**********************************************************/
void kalman_x_init(void);
float kalman_x(float ADC_Value);

void kalman_y_init(void);
float kalman_y(float ADC_Value);



#endif


