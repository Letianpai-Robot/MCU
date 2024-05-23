

#include <stdio.h>
#include "gyro_oula.h"

#define gyro_oula_DEBUG_EN            0     //debug EN=1 DEN=0
#if gyro_oula_DEBUG_EN
#define GyroOulaLog(...) printf(__VA_ARGS__)
#else
#define GyroOulaLog(...) 
#endif

#if 1
#include <math.h>

// control varie.  0~100     0.000001 - 2 
#define Kp 0.1
// acuulatation error
#define Ki 0
#define halfT 0.005 //Half the cycle time

float q0 = 0.7, q1 = 0, q2 = 0, q3 = 0.7;
float exInt = 0, eyInt = 0, ezInt = 0;

void q_init(void)
{
  q0 = 0.7;
  q1 = 0;
  q2 = 0; 
  q3 = 0.7;
}


#if 0 // chaochen-0407
float speedx = 0;
float speedy = 0;
float loc_x = 0;
float loc_y = 0;
#endif

float constrain(float value, float min, float max) 
{
  if (value < min) 
  {
    return min;
  } 
  else if (value > max) 
  {
    return max;
  } 
  else 
  {
    return value;
  }
}

int print_yaw_num = 0;

void six_Update(float ax, float ay, float az, float gx, float gy, float gz, float* pitch, float* roll, float* yaw) 
{
    // Normalize measurement
//    float norm = sqrt(ax * ax + ay * ay + az * az);
//    ax = ax / norm;
//    ay = ay / norm;
//    az = az / norm;
  
#if 0 //chaochen-0407
  //
   speedx  = speedx + ax * 2* halfT;
   speedy  = speedy + ay * 2* halfT;

   loc_x = loc_x + 2*(speedx * halfT + az  * halfT  * halfT);
   loc_y = loc_y + 2*(speedy * halfT + (ay-1.7)  * halfT  * halfT);
     
    GyroOulaLog("X=%f,Y=%f\r\n",speedx,speedy);
  #endif
    // Estimate direction of gravity
//    float vx = 2 * (q1 * q3 - q0 * q2);
//    float vy = 2 * (q0 * q1 + q2 * q3);
//    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

//    // Cross product of error and reference direction sensors measured
//    float ex = (ay * vz - az * vy);
//    float ey = (az * vx - ax * vz);
//    float ez = (ax * vy - ay * vx);

//    // Proportional Integral (PI) correction
//    exInt += ex * Ki;
//    eyInt += ey * Ki;
//    ezInt += ez * Ki;

//    // Avoid integral windup
//    exInt = constrain(exInt, -0.1, 0.1);
//    eyInt = constrain(eyInt, -0.1, 0.1);
//    ezInt = constrain(ezInt, -0.1, 0.1);

//    // Correct gyro measurements
//    gx = gx + Kp * ex + exInt;
//    gy = gy + Kp * ey + eyInt;
//    gz = gz + Kp * ez + ezInt;

//    // Integrate quaternion
//    float q0_pre = q0, q1_pre = q1, q2_pre = q2, q3_pre = q3;
//    q0 = q0_pre + (-q1_pre * gx - q2_pre * gy - q3_pre * gz) * halfT;
//    q1 = q1_pre + (q0_pre * gx + q2_pre * gz - q3_pre * gy) * halfT;
//    q2 = q2_pre + (q0_pre * gy - q1_pre * gz + q3_pre * gx) * halfT;
//    q3 = q3_pre + (q0_pre * gz + q1_pre * gy - q2_pre * gx) * halfT;


    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    // Normalize quaternion
    float norm_q = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm_q;
    q1 /= norm_q;
    q2 /= norm_q;
    q3 /= norm_q;

    // Convert quaternion to Euler angles
//    *pitch = asin(-2*q1*q3+2*q0*q2)*57.3;
//    *roll = atan2(2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1)*57.3;
//    *yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;
//    *pitch = -asin(-2*q1*q3+2*q0*q2)*57.3;
//    *roll = atan2(2*q2*q3+2*q0*q1, -2*q1*q1-2*q2*q2+1)*57.3;
    *yaw = -atan2(2*(q1*q2 + q0*q3), -2*q2*q2-2*q3*q3+1)*57.3;
		
//	if(print_yaw_num++ % 100 == 0)
//	printf("AT+INT,avg %f, %f, %f, %f, %f, %f, %f,%0.2f\r\n",q0, q1, q2, q3, gx, gy, gz, *yaw);
	//  test printf gyro yaw value for chengbotao	
	
//	if(print_yaw_num++ % 20 == 0)
//	printf("AT+INT,avg ax: %f, ay: %f, az: %f, gx: %f, gy: %f, gz: %f  ,%0.2f\r\n",ax, ay, az, gx, gy, gz, *yaw);
    //printf("AT+INT,avg %f,%f,%f  ,%0.2f\r\n", gx, gy, gz, *yaw);
		//printf("AT+INT,avg gx: %f, gy: %f, gz: %f\r\n",gx, gy, gz);
}
#endif