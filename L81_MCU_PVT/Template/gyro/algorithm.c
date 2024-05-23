

#include <stdio.h>
#include "L81_TimTask.h"
#include "gyro_oula.h"

#include "L81_AT.h"   //for at cmd fun
#include "L81_inf.h"  //for at cmd fun

#include "algorithm.h"
#include "KalmanFilter.h"
#include "L81_FMC.h"
#include "L81_MoveAlg.h"

#define ALGORITHM_DEBUG_EN            0     //debug EN=1 DEN=0
#if ALGORITHM_DEBUG_EN
#define AlgoLog(...) printf(__VA_ARGS__)
#else
#define AlgoLog(...) 
#endif

#define ALGOSTRAIGHT_DEBUG_EN            0     //debug EN=1 DEN=0
#if ALGOSTRAIGHT_DEBUG_EN
#define AlgostraightLog(...) printf(__VA_ARGS__)
#else
#define AlgostraightLog(...) 
#endif




U_GYRO_DATA_TYPDEF u_acc_data = {0};  // Acceleration
U_GYRO_DATA_TYPDEF u_ang_rate_data = {0}; //Angular Rate
U_GYRO_DATA_TYPDEF u_ang_rate_kal = {0}; //

uint8_t avg_flag = 0;
uint32_t avg_count = 0;
U_GYRO_DATA_TYPDEF u_acc_avg = {0};       // Acceleration avg
U_GYRO_DATA_TYPDEF u_ang_rate_avg = {0};  //Angular Rate avg

int ALGORIHM_AVG_NUM = 300;
float algo_x = 0;

typedef struct
{
  uint8_t algo_en;    //1=en 0=den
  uint8_t Euler_en;   //1=en 0=den
  uint8_t avg_flag;   //1=avg_ok 0=avg_err;
}T_ALGO_SET_PARAM_TYPDEF;


float pitch_dat;
float roll_dat;
float yaw_dat = 90;
T_ALGO_SET_PARAM_TYPDEF t_algo_set = {0};
//float yaw_chenge;
//int num = 0;

////wxf-test tim
//#include "L81_inf.h"
//uint32_t last_tim_count = 0;
//uint32_t cur_tim_count = 0;



#define WAGGLE_CHECK_TIME     1000 //MS
#define WAGGLE_ACC_MIN        -20.0f
#define WAGGLE_ACC_MAX        20.0f
#define WAGGLE_CHECK_NUM      10
typedef struct
{
  uint8_t check_en;
  uint8_t check_flag;
  
  int8_t acc_x;
  int8_t acc_y;
  int8_t acc_z;

//  uint8_t waggle_flag;
  
  uint16_t tim_count; //
  uint16_t waggle_count; //
  
}T_WAGGLE_PARAM_TYPDEF;

T_WAGGLE_PARAM_TYPDEF tWaggleParam = {0};



#define FAIL_DOWN_COUNG_NUM         100
typedef enum
{
  DOWN_UPRITHT = 0U,
  DOWN_DOWN,
  DOWN_LIE_DOWN,
  DOWN_CLIMB_DOWN,
  DOWN_LEFT_SIDE,
  DOWN_RIGHT_SIDE,
  
}E_FAIL_DOWN_FLAG_TYPDEF;

typedef struct
{
  uint16_t acc_count_x1; //+x
  uint16_t acc_count_y1; //+y    
  uint16_t acc_count_z1; //+z
  
  uint16_t acc_count_x2; //-x
  uint16_t acc_count_y2; //-y   
  uint16_t acc_count_z2; //-z
  
  E_FAIL_DOWN_FLAG_TYPDEF e_down_last_flag;
  E_FAIL_DOWN_FLAG_TYPDEF e_down_flag;
}T_FALL_DOWN_PARAM_TYPDEF;

T_FALL_DOWN_PARAM_TYPDEF  tFailDownParam = {0};



//if p_value < min :return -1
//if p_value > max :return 1
//if min < p_value < max :return 0
int8_t comp_value(float *p_value, float min, float max)
{
  
  if(*p_value >= max)
  {
    return 1;
  }
  
  if(*p_value <= min)
  {
    return -1;
  }
  
  return 0;
}




//waggle check
void waggle_check(U_GYRO_DATA_TYPDEF *pt_angle)
{
  
  T_WAGGLE_PARAM_TYPDEF *pt_waggle = &tWaggleParam;

  pt_waggle->acc_x = comp_value(&pt_angle->t_data.x, WAGGLE_ACC_MIN, WAGGLE_ACC_MAX);
  pt_waggle->acc_y = comp_value(&pt_angle->t_data.y, WAGGLE_ACC_MIN, WAGGLE_ACC_MAX);
  pt_waggle->acc_z = comp_value(&pt_angle->t_data.z, WAGGLE_ACC_MIN, WAGGLE_ACC_MAX);
  
  
  if((pt_waggle->acc_x != 0) || (pt_waggle->acc_y != 0) || (pt_waggle->acc_z != 0))
  {
    pt_waggle->check_flag = 1;
    pt_waggle->waggle_count ++;
    
//    AlgoLog("L81_count=%d\r\n",pt_waggle->waggle_count);
  }
  
  if(pt_waggle->check_flag)
  {
    if(pt_waggle->tim_count < WAGGLE_CHECK_TIME)
    {
      pt_waggle->tim_count += ALGORIHM_CYCLE;
    }
    else
    {
      pt_waggle->check_flag = 0;
      pt_waggle->tim_count = 0;
      if(pt_waggle->waggle_count > WAGGLE_CHECK_NUM)
      {
        printf("AT+INT,waggle,1\r\n");
        
      }
      pt_waggle->waggle_count = 0;
    }
  }
  
}


//cycle time: ALGORIHM_CYCLE 
//refe:fall down check
void fall_down_check(U_GYRO_DATA_TYPDEF *pt_acc)
{
  T_FALL_DOWN_PARAM_TYPDEF  *pt_down = &tFailDownParam;
  
  //x
  if(pt_acc->t_data.x > 8)
  {
//      pt_down->acc_count_x1 = 0;
    pt_down->acc_count_y1 = 0;
    pt_down->acc_count_z1 = 0;
    pt_down->acc_count_x2 = 0;
    pt_down->acc_count_y2 = 0;
    pt_down->acc_count_z2 = 0;
    
    if(pt_down->acc_count_x1 < FAIL_DOWN_COUNG_NUM)
    {
      pt_down->acc_count_x1++;
    }
    else
    {
      pt_down->e_down_flag = DOWN_UPRITHT;
    }
  }
  else if(pt_acc->t_data.x < -8)
  {

    pt_down->acc_count_x1 = 0;
    pt_down->acc_count_y1 = 0;
    pt_down->acc_count_z1 = 0;
//    pt_down->acc_count_x2 = 0;
    pt_down->acc_count_y2 = 0;
    pt_down->acc_count_z2 = 0;
    
    if(pt_down->acc_count_x2 < FAIL_DOWN_COUNG_NUM)
    {
      pt_down->acc_count_x2++;
    }
    else
    {
      pt_down->e_down_flag = DOWN_DOWN;
    }
  }
  
  //z
  if(pt_acc->t_data.z > 8)
  {
    pt_down->acc_count_x1 = 0;
    pt_down->acc_count_y1 = 0;
//    pt_down->acc_count_z1 = 0;
    pt_down->acc_count_x2 = 0;
    pt_down->acc_count_y2 = 0;
    pt_down->acc_count_z2 = 0;
    
    if(pt_down->acc_count_z1 < FAIL_DOWN_COUNG_NUM)
    {
      pt_down->acc_count_z1++;
    }
    else
    {
      pt_down->e_down_flag = DOWN_LIE_DOWN;
    }
  }
  else if(pt_acc->t_data.z < -8)
  {
    pt_down->acc_count_x1 = 0;
    pt_down->acc_count_y1 = 0;
    pt_down->acc_count_z1 = 0;
    pt_down->acc_count_x2 = 0;
    pt_down->acc_count_y2 = 0;
//    pt_down->acc_count_z2 = 0;
    
    if(pt_down->acc_count_z2 < FAIL_DOWN_COUNG_NUM)
    {
      pt_down->acc_count_z2++;
    }
    else
    {
      pt_down->e_down_flag = DOWN_CLIMB_DOWN;
    }
  }
  
  //y
  if(pt_acc->t_data.y > 8)
  {
    pt_down->acc_count_x1 = 0;
//    pt_down->acc_count_y1 = 0;
    pt_down->acc_count_z1 = 0;
    pt_down->acc_count_x2 = 0;
    pt_down->acc_count_y2 = 0;
    pt_down->acc_count_z2 = 0;
    
    if(pt_down->acc_count_y1 < FAIL_DOWN_COUNG_NUM)
    {
      pt_down->acc_count_y1++;
    }
    else
    {
      pt_down->e_down_flag = DOWN_LEFT_SIDE;
    }
  }
  else if(pt_acc->t_data.y < -8)
  {
    pt_down->acc_count_x1 = 0;
    pt_down->acc_count_y1 = 0;
    pt_down->acc_count_z1 = 0;
    pt_down->acc_count_x2 = 0;
//    pt_down->acc_count_y2 = 0;
    pt_down->acc_count_z2 = 0;
    
    if(pt_down->acc_count_y2 < FAIL_DOWN_COUNG_NUM)
    {
      pt_down->acc_count_y2++;
    }
    else
    {
      pt_down->e_down_flag = DOWN_RIGHT_SIDE;
    }
  }
  
  //up data
  if(pt_down->e_down_last_flag != pt_down->e_down_flag)
  {
    pt_down->e_down_last_flag = pt_down->e_down_flag;
    printf("AT+INT,down,%d\r\n",(pt_down->e_down_flag));
  }
  
}


//get algo value
void algo_get(void)
{
  
  #if 1
  GYRO_READ_ACC(u_acc_data.a_data);
  GYRO_READ_GYRO(u_ang_rate_data.a_data);
  #else
  GYRO_READ_ACC_ANG(u_acc_data.a_data, u_ang_rate_data.a_data); //test err
  #endif
  
  
//  last_tim_count = get_timer();
//  cur_tim_count = get_timer();
  
  
//  AlgoLog("tim=%d\r\n",(cur_tim_count-last_tim_count)<<2);
  
//  AlgoLog("ax=%f,ay=%f,az=%f\r\n",u_acc_data.t_data.x, u_acc_data.t_data.y, u_acc_data.t_data.z);
//  AlgoLog("gx=%f,gy=%f,gz=%f\r\n\r\n",u_ang_rate_data.t_data.x, u_ang_rate_data.t_data.y, u_ang_rate_data.t_data.z);

}
U_GYRO_DATA_TYPDEF u_acc_avg_last = {0};       // Acceleration avg
U_GYRO_DATA_TYPDEF u_ang_rate_avg_last = {0};  //Angular Rate avg
//get algo avg value 
void algo_avg_get(void)
{
  avg_count ++;
  if(avg_count > ALGORIHM_AVG_NUM)
  {
		avg_count = 0;
    l81_tim_task_den(ID_GYRO_AVG);
    
    u_acc_avg.t_data.x = u_acc_avg.t_data.x/ALGORIHM_AVG_NUM;
    u_acc_avg.t_data.y = u_acc_avg.t_data.y/ALGORIHM_AVG_NUM;
    u_acc_avg.t_data.z = u_acc_avg.t_data.z/ALGORIHM_AVG_NUM;
    
    u_ang_rate_avg.t_data.x = u_ang_rate_avg.t_data.x/ALGORIHM_AVG_NUM;
    u_ang_rate_avg.t_data.y = u_ang_rate_avg.t_data.y/ALGORIHM_AVG_NUM;
    u_ang_rate_avg.t_data.z = u_ang_rate_avg.t_data.z/ALGORIHM_AVG_NUM;
    
    u_acc_avg_last.t_data.x = u_acc_avg.t_data.x/ALGORIHM_AVG_NUM;
    u_acc_avg_last.t_data.y = u_acc_avg.t_data.y/ALGORIHM_AVG_NUM;
    u_acc_avg_last.t_data.z = u_acc_avg.t_data.z/ALGORIHM_AVG_NUM;
    
    u_ang_rate_avg_last.t_data.x = u_ang_rate_avg.t_data.x/ALGORIHM_AVG_NUM;
    u_ang_rate_avg_last.t_data.y = u_ang_rate_avg.t_data.y/ALGORIHM_AVG_NUM;
    u_ang_rate_avg_last.t_data.z = u_ang_rate_avg.t_data.z/ALGORIHM_AVG_NUM;
		
		
    t_algo_set.avg_flag = ALGORIHM_AVG_READY;
    
    AlgoLog("avg %f,%f,%f - %f,%f,%f \r\n",u_acc_avg.t_data.x,u_acc_avg.t_data.y,u_acc_avg.t_data.z, \
    u_ang_rate_avg.t_data.x,u_ang_rate_avg.t_data.y,u_ang_rate_avg.t_data.z);
		AlgostraightLog("avg %f,%f,%f - %f,%f,%f \r\n",u_acc_avg.t_data.x,u_acc_avg.t_data.y,u_acc_avg.t_data.z, \
    u_ang_rate_avg.t_data.x,u_ang_rate_avg.t_data.y,u_ang_rate_avg.t_data.z);
    
		
		kalman_y_init();
		return;
  }
//	is_moving();
//  if(is_moving() == 1)  //0829  rjq++
//	{
//		AlgostraightLog("algo_avg_get stop!\r\n");
//		if(avg_count <= 3)
//		{
//			l81_tim_task_den(ID_GYRO_AVG);
//			t_algo_set.avg_flag = ALGORIHM_AVG_READY;
//			avg_count = 0;    
//			u_acc_avg.t_data.x = u_acc_avg_last.t_data.x;
//			u_acc_avg.t_data.y = u_acc_avg_last.t_data.y;
//			u_acc_avg.t_data.z = u_acc_avg_last.t_data.z;
//			
//			u_ang_rate_avg.t_data.x = u_ang_rate_avg_last.t_data.x;
//			u_ang_rate_avg.t_data.y = u_ang_rate_avg_last.t_data.y;
//			u_ang_rate_avg.t_data.z = u_ang_rate_avg_last.t_data.z;
//			return;
//		}
//		else{
//			ALGORIHM_AVG_NUM = avg_count;
//			return;
//		}
//	}
  u_acc_avg.t_data.x += (u_acc_data.t_data.x - ALGORIHM_STANDARD_Ax);
  u_acc_avg.t_data.y += (u_acc_data.t_data.y - ALGORIHM_STANDARD_Ay);
  u_acc_avg.t_data.z += (u_acc_data.t_data.z - ALGORIHM_STANDARD_Az);
  
  u_ang_rate_avg.t_data.x += kalman_y(u_ang_rate_data.t_data.x - ALGORIHM_STANDARD_Gx);
  u_ang_rate_avg.t_data.y += (u_ang_rate_data.t_data.y - ALGORIHM_STANDARD_Gy);
  u_ang_rate_avg.t_data.z += (u_ang_rate_data.t_data.z - ALGORIHM_STANDARD_Gz);

}


//algo avg task init
void algo_avg_start(void)
{
  t_algo_set.avg_flag = ALGORIHM_AVG_NOT;
  avg_count = 0;
  u_acc_avg.t_data.x = 0;
  u_acc_avg.t_data.y = 0;  
  u_acc_avg.t_data.z = 0;  

  u_ang_rate_avg.t_data.x = 0;  
  u_ang_rate_avg.t_data.y = 0;  
  u_ang_rate_avg.t_data.z = 0;  

}


int print_num = 0;
float algo_val_x = 0;
float algo_val_y = 0;
float algo_val_z = 0;
float algo_val_0 = 0;
void get_algo_avg()
{
	
//	if(algo_val_x > 180) algo_val_x-=180;
//	if(algo_val_x < -180) algo_val_x+=180;
//	AlgostraightLog("algo_val_x = %f\r\n",algo_val_x);
	if(algo_val_0 > 180) algo_val_0-=360;
	if(algo_val_0 < -180) algo_val_0+=360;
	
	AlgostraightLog("algo_val_0 = %f\r\n",algo_val_0);
//  AlgostraightLog("yaw_dat = %0.2f\r\n",yaw_dat);
}
void clear_algo_avg()
{
  algo_val_x = 0;
  algo_val_y = 0;
  algo_val_z = 0;
  algo_val_0 = 0;
	
}

//algo task
void algo_only_x_task()
{
	qmi8658_read_gyro_data_x(algo_x);
	
	//  //avg_flag
  if(t_algo_set.avg_flag != ALGORIHM_AVG_READY)
  {
    algo_avg_get();
    return;
  }
  if(t_algo_set.Euler_en ==  ALGORIHM_ON)
	{
		algo_val_x += algo_x *0.005;
		algo_val_0 += (algo_x - u_ang_rate_avg.t_data.x)*0.005;
		if(print_num++  % 200 == 0)
			get_algo_avg();
	}
}
	
//algo task
void algo_task(void *param)
{
  #if 1
//  if(avg_flag == ALGORIHM_AVG_NOT)
//  {
//    return;
//  }
  
//  last_tim_count = get_timer();
  algo_only_x_task();
  algo_get();

  //waggle check
  waggle_check(&u_acc_data);
  
  //fall down check
  fall_down_check(&u_acc_data);

  //avg_flag
  if(t_algo_set.avg_flag != ALGORIHM_AVG_READY)
  {
    algo_avg_get();
    return;
  }
	
	if(u_ang_rate_data.t_data.x > 5 || u_ang_rate_data.t_data.x < -5)
		AlgostraightLog("data.x : %f\r\n" , u_ang_rate_data.t_data.x);
//	algo_val_x -= kalman_x(u_ang_rate_data.t_data.x - u_ang_rate_avg.t_data.x)*0.5729;
//	algo_val_0 -= (u_ang_rate_data.t_data.x - u_ang_rate_avg.t_data.x)*0.5729;
//	print_num++;
//	if(print_num  % 300 == 0)
//	{
//		if(algo_val_0 > 180) algo_val_0-=360;
//		if(algo_val_0 < -180) algo_val_0+=360;
//		printf("num=%d,gx=%f,avgx=%f,algo_val=%f,avg,%f\r\n",print_num/300,u_ang_rate_data.t_data.x, u_ang_rate_avg.t_data.x,algo_val_0,yaw_dat-90);
//		AlgostraightLog("num=%d,gx=%f,avgx=%f,algo_val=%f,avg,%f\r\n",print_num/300,u_ang_rate_data.t_data.x, u_ang_rate_avg.t_data.x,algo_val_0,yaw_dat-90);
//
//		//		get_algo_avg();
//	}
//	return;
  if(t_algo_set.Euler_en ==  ALGORIHM_ON)
  {
    //start Euler Angle algorithm
    u_acc_data.t_data.x -=  u_acc_avg.t_data.x;
    u_acc_data.t_data.y -=  u_acc_avg.t_data.y;
    u_acc_data.t_data.z -=  u_acc_avg.t_data.z;
    
    u_ang_rate_data.t_data.x -= u_ang_rate_avg.t_data.x;
    u_ang_rate_data.t_data.y -= u_ang_rate_avg.t_data.y;
    u_ang_rate_data.t_data.z -= u_ang_rate_avg.t_data.z;
  //  AlgoLog("ax=%f,ay=%f,az=%f\r\n",u_acc_data.t_data.x, u_acc_data.t_data.y, u_acc_data.t_data.z);
  //  AlgoLog("gx=%f,gy=%f,gz=%f\r\n",u_ang_rate_data.t_data.x, u_ang_rate_data.t_data.y, u_ang_rate_data.t_data.z);
    
  //  six_Update(u_acc_data.t_data.y, u_acc_data.t_data.z, u_acc_data.t_data.x, \
  //             u_ang_rate_data.t_data.y, u_ang_rate_data.t_data.z, u_ang_rate_data.t_data.x, \
  //             &pitch_dat, &roll_dat, &yaw_dat);
    
    //lalman x y
    u_ang_rate_kal.t_data.x = kalman_x(u_ang_rate_data.t_data.x);
    u_ang_rate_kal.t_data.y = kalman_y(u_ang_rate_data.t_data.y);
    
    six_Update(u_acc_data.t_data.y, u_acc_data.t_data.z, u_acc_data.t_data.x, \
               u_ang_rate_kal.t_data.y, u_ang_rate_data.t_data.z, u_ang_rate_kal.t_data.x, \
               &pitch_dat, &roll_dat, &yaw_dat);
    

    yaw_dat += 180; //angle turn  (-180~180) -> (0~360)
    
    AlgoLog("%0.2f\r\n",yaw_dat);
  }
//  cur_tim_count = get_timer();
//  AlgoLog("tim=%d\r\n",(cur_tim_count-last_tim_count)<<2);



{
//  AlgoLog("a_xyz %0.2f, %0.2f, %0.2f ",u_acc_data.t_data.x, u_acc_data.t_data.y, u_acc_data.t_data.z);
//  AlgoLog("g_xyz %0.2f, %0.2f, %0.2f\r\n",u_ang_rate_kal.t_data.x, u_ang_rate_kal.t_data.y, u_ang_rate_data.t_data.z);
//  AlgoLog("ang %0.2f, %0.2f, %0.2f\r\n\r\n",pitch_dat, roll_dat, yaw_dat);
  
}

#endif
}

//return:err =1 ok=0
uint8_t read_backup_algo_avg(void)
{
  uint32_t l81_fmc_read(uint32_t addr);
  
  union u_dat
  {
    float f_dat;
    uint32_t u32_dat;
  };
  
  union u_dat u_dat32 = {0};

  u_dat32.u32_dat = l81_fmc_read(ACCX_ADDR);
  u_acc_avg.t_data.x = u_dat32.f_dat;
  if(u_dat32.u32_dat == 0xffffffff)
  {
    return ALGORIHM_ERR;
  }
  
  u_dat32.u32_dat = l81_fmc_read(ACCY_ADDR);
  u_acc_avg.t_data.y = u_dat32.f_dat;
  if(u_dat32.u32_dat == 0xffffffff)
  {
    return ALGORIHM_ERR;
  }
  
  u_dat32.u32_dat = l81_fmc_read(ACCZ_ADDR);
  u_acc_avg.t_data.z = u_dat32.f_dat;
  if(u_dat32.u32_dat == 0xffffffff)
  {
    return ALGORIHM_ERR;
  }
  
  u_dat32.u32_dat = l81_fmc_read(GYROX_ADDR);
  u_ang_rate_avg.t_data.x = u_dat32.f_dat;
  if(u_dat32.u32_dat == 0xffffffff)
  {
    return ALGORIHM_ERR;
  }
  
  u_dat32.u32_dat = l81_fmc_read(GYROY_ADDR);
  u_ang_rate_avg.t_data.y = u_dat32.f_dat;
  if(u_dat32.u32_dat == 0xffffffff)
  {
    return ALGORIHM_ERR;
  }
  
  u_dat32.u32_dat = l81_fmc_read(GYROZ_ADDR);
  u_ang_rate_avg.t_data.z = u_dat32.f_dat;
  if(u_dat32.u32_dat == 0xffffffff)
  {
    return ALGORIHM_ERR;
  }
//  u_acc_avg_last  = u_acc_avg;
//	u_ang_rate_avg_last = u_ang_rate_avg;
			u_acc_avg_last.t_data.x = u_acc_avg.t_data.x;
			u_acc_avg_last.t_data.y = u_acc_avg.t_data.y;
			u_acc_avg_last.t_data.z = u_acc_avg.t_data.z;
			
			u_ang_rate_avg_last.t_data.x = u_ang_rate_avg.t_data.x;
			u_ang_rate_avg_last.t_data.y = u_ang_rate_avg.t_data.y;
			u_ang_rate_avg_last.t_data.z = u_ang_rate_avg.t_data.z;
	
//  AlgoLog("acc_xyz %0.2f, %0.2f, %0.2f ",u_acc_avg.t_data.x, u_acc_avg.t_data.y, u_acc_avg.t_data.z);
//  AlgoLog("ang_xyz %0.2f, %0.2f, %0.2f ",u_ang_rate_avg.t_data.x, u_ang_rate_avg.t_data.y, u_ang_rate_avg.t_data.z);
//  AlgoLog("acc_xyz %f, %f, %f ",u_acc_avg.t_data.x, u_acc_avg.t_data.y, u_acc_avg.t_data.z);
//  AlgoLog("ang_xyz %f, %f, %f ",u_ang_rate_avg.t_data.x, u_ang_rate_avg.t_data.y, u_ang_rate_avg.t_data.z);
  
  return 0;
}


//start algo task and init algo_avg_value
void algo_task_init(void)
{

  //init flag
  t_algo_set.algo_en = ALGORIHM_OK;
  t_algo_set.avg_flag = ALGORIHM_AVG_NOT;
  t_algo_set.Euler_en = ALGORIHM_AVG_NOT;
  
  //read backup avg
  if(ALGORIHM_OK == read_backup_algo_avg())
  {
    t_algo_set.avg_flag = ALGORIHM_AVG_READY;
  }
  
  l81_tim_task_creat(ID_GYRO_DEAL, TIM_TASK_CYCLE_ALL, ALGORIHM_CYCLE, NULL, algo_task);
  l81_tim_task_en(ID_GYRO_DEAL);
  
//  algo_avg_start();
}

//stop algo task
void algo_task_stop(void)
{
  l81_tim_task_den(ID_GYRO_DEAL);
}

//q0~q4 init
void algo_q_init(void)
{
  q_init();
}

uint8_t algo_yaw_read(float *p_data)
{
//  float *pf_yaw = &yaw_dat;
  if(p_data == NULL)
  {
    return ALGORIHM_ERR;
  }
  if(avg_flag == ALGORIHM_AVG_READY)
  {
    *p_data = yaw_dat;
    return ALGORIHM_OK;
  }
  
  return ALGORIHM_ERR;
}

//alg task stop
void algorithm_task_stop(void)
{
  algo_task_stop();
  algo_q_init();
  kalman_x_init();
  kalman_y_init();
}

//alg task start
void algorithm_task_start(void)
{
  algo_task_init();
  algo_q_init();
  kalman_x_init();
  kalman_y_init();
}

uint8_t l81_AT_FILTE_AG_W_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 1U && param_num != 2U)
  {  //motor must has one paramters, (motor number)
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
	
	uint32_t cmd = String2Int(param[0]);
	uint32_t ag_num = 300;
	if(param_num >= 2)
		ag_num = String2Int(param[1]);
		
	if (cmd > 4U) 
  {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong cmd %d, cmd is[0,4]\r\n", cmd);
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
   
  if(0U == cmd) // gyro alg start
  {                  
    t_algo_set.Euler_en = ALGORIHM_ON;
		clear_algo_avg();
    printf("AT+RES,ACK\r\n");
    printf("AT+RES,FiAGW,%d\r\n",cmd);
    printf("AT+RES,end\r\n");
    
  }	
  else if(1U == cmd) // gyro alg stop
  {       
    t_algo_set.Euler_en = ALGORIHM_OFF;
    algo_q_init();
    kalman_x_init();
    kalman_y_init();
    printf("AT+RES,ACK\r\n");
    printf("AT+RES,FiAGW,%d\r\n",cmd);
    printf("AT+RES,end\r\n");

  }
  else if(2U == cmd) // gyro avg init
  {  
    algo_avg_start();
//    algo_q_init();
//		clear_algo_avg();
    kalman_x_init();
    kalman_y_init();
	if(param_num >= 2)
		ALGORIHM_AVG_NUM = ag_num;
    printf("AT+RES,ACK\r\n");
	if(param_num >= 2)
    	printf("AT+RES,FiAGW,%d,%d\r\n",cmd,ALGORIHM_AVG_NUM);
    printf("AT+RES,end\r\n");
  
  }
  else if(3U == cmd) // gyro alg task stop
  {  
    algorithm_task_stop();
    printf("AT+RES,ACK\r\n");
    printf("AT+RES,FiAGW,%d\r\n",cmd);
    printf("AT+RES,end\r\n");
  
  }
  else if(4U == cmd) // gyro alg task start
  {  
    algorithm_task_start();
    printf("AT+RES,ACK\r\n");
    printf("AT+RES,FiAGW,%d\r\n",cmd);
    printf("AT+RES,end\r\n");
  
  }
  
		
    return 1U;	
}

uint8_t l81_AT_FILTE_AG_R_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
  U_GYRO_DATA_TYPDEF *ptu_gyro_acc = &u_acc_data;
  U_GYRO_DATA_TYPDEF *ptu_gyro_rate = &u_ang_rate_data;
  float *pf_yaw = &yaw_dat;
//	float acc[3];
//	float gyro[3];

  ATcmd_split_params(params, param, &param_num);

	if (param_num > 1U)
  {  //motor must has one paramters, (motor number)
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
	
  
  if(t_algo_set.avg_flag == ALGORIHM_AVG_READY)
  {
    printf("AT+RES,ACK\r\n");
//      l81_tim_task_den(ID_GYRO_AVG);
    printf("AT+RES,yaw,%0.2f\r\n", *pf_yaw);
//      l81_tim_task_en(ID_GYRO_AVG);
    printf("AT+RES,end\r\n");
  }
  else
  {
    printf("AT+RES,ACK\r\n");
    printf("AT+RES,Err,wrong,gyro avg not ready\r\n");
    printf("AT+RES,end\r\n");
  }
		
    return 1U;	
}




