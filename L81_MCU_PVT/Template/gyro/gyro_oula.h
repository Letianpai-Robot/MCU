

#ifndef __GYRO_OULA_H__
#define __GYRO_OULA_H__

#include <stdint.h>
//#include <stddef.h>

//#include "main.h"
//#include "gd32l23x_i2c.h"
//#include "gd32l23x_gpio.h"

/**********************************************************/
//PORT DEFINE


/**********************************************************/
void q_init(void);
void six_Update(float ax, float ay, float az, float gx, float gy, float gz, float* pitch, float* roll, float* yaw);

#endif


