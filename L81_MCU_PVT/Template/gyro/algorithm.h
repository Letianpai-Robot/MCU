

#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__

#include <stdint.h>
#include <stddef.h>

#include "qmi8658.h"

//#include "main.h"
//#include "gd32l23x_i2c.h"
//#include "gd32l23x_gpio.h"

/**********************************************************/
//PORT DEFINE
#define GYRO_READ_ACC(acc_buf_add)  qmi8658_read_acc_data(acc_buf_add)
#define GYRO_READ_GYRO(gyro_buf_add)  qmi8658_read_gyro_data(gyro_buf_add)
#define GYRO_READ_ACC_ANG(acc_buf_add,gyro_buf_add)  qmi8658_read_sensor_data(acc_buf_add,gyro_buf_add) //read test err:no gyro data;


/**********************************************************/
#define ALGORIHM_CYCLE            10   //ms

#define ALGORIHM_AVG_CYCLE        10    //ms
//int ALGORIHM_AVG_NUM = 300;
//#define ALGORIHM_AVG_NUM          300
#define ALGORIHM_AVG_READY        1
#define ALGORIHM_AVG_NOT          0

#define ALGORIHM_STANDARD_Ax      9.65
#define ALGORIHM_STANDARD_Ay      0
#define ALGORIHM_STANDARD_Az      1.7

#define ALGORIHM_STANDARD_Gx      0
#define ALGORIHM_STANDARD_Gy      0
#define ALGORIHM_STANDARD_Gz      0

#define ALGORIHM_ERR              1
#define ALGORIHM_OK               0

#define ALGORIHM_ON               1
#define ALGORIHM_OFF              0


typedef struct
{
  float x;
  float y;
  float z;
}T_GYRO_FLOAT_DATA_TYPDEF;

typedef union
{
  float a_data[3];
  T_GYRO_FLOAT_DATA_TYPDEF t_data;
}U_GYRO_DATA_TYPDEF;


extern float yaw_dat; //



void algorithm_task_start(void);//alg task start
void algorithm_task_stop(void);//alg task stop

void algo_task_stop(void);  //stop 
void algo_q_init(void);     //init q0 q1 q2 q3


uint8_t algo_yaw_read(float *p_data); //read yaw (0~360)

void algo_task_init(void); //start angle avg and init  
uint8_t l81_AT_FILTE_AG_W_func(char params[]); //at cmd fun
uint8_t l81_AT_FILTE_AG_R_func(char params[]); //at cmd fun

void algo_avg_start(void);
void get_algo_avg(void);  //0821  rjq++

#endif


