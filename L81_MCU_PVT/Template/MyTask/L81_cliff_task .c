/*!
    \file    L81_cliff_task.c
    \brief    
*/

/*
    Copyright (c) 2023, wxf 
*/


#include "L81_cliff_task.h"
#include "L81_MoveAlg.h"
#include "L81_cliff.h"
#include "aw9310x.h"
#include "L81_FMC.h"

/************************************ brief*******************************************/
/* 

*/
/****************************************************************************************************/
#define CLIFF_DEBUG_EN            0   //debug EN=1 DEN=0
#if CLIFF_DEBUG_EN
#define CliffLog(...) printf(__VA_ARGS__)
#else
#define CliffLog(...) 
#endif


//#define READ_CLIFF_FLAG()     0
#define READ_SUSPEND_FLAG()   0
T_CLIFF_PARAM_TYPDEF tCliffParam = {0};

volatile uint32_t CLIFF_DENGER_NUM=100;

#define  CLIFF_L_F   (1<<0)
#define  CLIFF_L_B   (1<<1)
#define  CLIFF_R_B   (1<<2)
#define  CLIFF_R_F   (1<<3)

#define  CLIFF_L (CLIFF_L_F|CLIFF_L_B)
#define  CLIFF_R (CLIFF_R_B|CLIFF_R_F)
#define  CLIFF_F (CLIFF_L_F|CLIFF_R_F)
#define  CLIFF_B (CLIFF_L_B|CLIFF_R_B)

#define CLIFF_NONE    0X00
#define CLIFF_ALL     0X0F

typedef enum
{
  E_CLIFF_NONE = 0X00,
  E_CLIFF_L1_F,
  E_CLIFF_L1_B,
  E_CLIFF_R1_B,
  E_CLIFF_R1_F,
  E_CLIFF_L2,
  E_CLIFF_B2,
  E_CLIFF_R2,
  E_CLIFF_F2,
  E_CLIFF_OTH,
}E_CLIFF_FLAG_TYPDEF;


/*read cliff port */
#define CLIFF_DANGER_VALUE  3400U         // 3400 for dvt2, 3500 for foot add 1mm
#define CLIFF_DANGER_SUSPEND_VALUE  3400U // 3400 for dvt2, 3500 for foot add 1mm

/**
    \brief      read cliff status 
    \param[in]  none
    \param[out] p_cliff_status 
    \retval     uint8_t err 0=ok 1=err/busy
*/


uint8_t print_cliff_num = 0;
uint8_t cliff_suspend_status_read(uint8_t *p_cliff_status, uint8_t *p_suspend_status)
{
  uint16_t leftF = 0u, leftB = 0u, rightF = 0u, rightB = 0u;
	
  uint8_t cliff_flag = 0;
  float i_ir = 0.25;
	if(cliff_status == cliff_run)
  {
		l81_cliff_get_adc(&leftB, &leftF, &rightF, &rightB);    
		CliffLog("AT+RES,cliff,%d,%d,%d,%d\r\n", leftF, leftB, rightF, rightB);
		if (print_cliff_num++ % 50 == 0)
			if(is_printf_cliff == 1)  //1124 rjq++
				printf("cliff,%d,%d,%d,%d\r\n", leftF, leftB, rightF, rightB);
		
		uint32_t addr = Cliff_ADDR;
		uint32_t data_lf = l81_fmc_read(addr + 1 * sizeof(uint32_t));
		uint32_t data_lb = l81_fmc_read(addr);
		uint32_t data_rf = l81_fmc_read(addr + 2 * sizeof(uint32_t));
		uint32_t data_rb = l81_fmc_read(addr + 3 * sizeof(uint32_t));
//		CliffLog("AT+RES,FMC,%d,%d,%d,%d\r\n", data_lf, data_lb, data_rf, data_rb);
		if(data_lf == -1 || data_lf < CLIFF_DANGER_VALUE) data_lf = CLIFF_DANGER_VALUE + CLIFF_DENGER_NUM;
		if(data_lb == -1 || data_lb < CLIFF_DANGER_VALUE) data_lb = CLIFF_DANGER_VALUE + CLIFF_DENGER_NUM;
		if(data_rf == -1 || data_rf < CLIFF_DANGER_VALUE) data_rf = CLIFF_DANGER_VALUE + CLIFF_DENGER_NUM;
		if(data_rb == -1 || data_rb < CLIFF_DANGER_VALUE) data_rb = CLIFF_DANGER_VALUE + CLIFF_DENGER_NUM;
//		CliffLog("AT+RES,FMC,%d,%d,%d,%d\r\n", data_lf, data_lb, data_rf, data_rb);
#if CLIFF_DEBUG_EN
//		uint16_t ALL_num = leftB + leftF + rightF + rightB;
//		
//    CliffLog("%d,%d,%d,%d\r\n",leftF,leftB,rightF,rightB);
//    if(leftF > CLIFF_DANGER_VALUE - 300 + (ALL_num - leftF)/3*i_ir)
//    {
//      CliffLog("%d\r\n",leftF);
//      CliffLog("%f\r\n",CLIFF_DANGER_VALUE - 300 + (ALL_num - leftF)/3*i_ir);
//      cliff_flag |= CLIFF_L_F;
//    }
//    
//    if(leftB > CLIFF_DANGER_VALUE - 300 + (ALL_num - leftB)/3*i_ir)
//    {
//      CliffLog("%d\r\n",leftB);
//      CliffLog("%f\r\n",CLIFF_DANGER_VALUE - 300 + (ALL_num - leftB)/3*i_ir);
//      cliff_flag |= CLIFF_L_B;
//    }
//    
//    if(rightB > CLIFF_DANGER_VALUE - 300 + (ALL_num - rightB)/3*i_ir)
//    {
//      CliffLog("%d\r\n",rightB);
//      CliffLog("%f\r\n",CLIFF_DANGER_VALUE - 300 + (ALL_num - rightB)/3*i_ir);
//      cliff_flag |= CLIFF_R_B;
//    }
//    
//    if(rightF > CLIFF_DANGER_VALUE - 300 + (ALL_num - rightF)/3*i_ir)
//    {
//      CliffLog("%d\r\n",rightF);
//      CliffLog("%f\r\n",CLIFF_DANGER_VALUE - 300 + (ALL_num - rightF)/3*i_ir);
//      cliff_flag |= CLIFF_R_F;
//    }
#endif

		if(leftF > data_lf - CLIFF_DENGER_NUM)
    {
      CliffLog("leftF denger %d > %d\r\n",leftF , data_lf - CLIFF_DENGER_NUM);
      cliff_flag |= CLIFF_L_F;
    }
    
    if(leftB > data_lb - CLIFF_DENGER_NUM)
    {
      CliffLog("leftB denger %d > %d\r\n",leftB , data_lb - CLIFF_DENGER_NUM);
      cliff_flag |= CLIFF_L_B;
    }
    if(rightF > data_rf - CLIFF_DENGER_NUM)
    {
      CliffLog("rightF denger %d > %d\r\n",rightF , data_rf - CLIFF_DENGER_NUM);
      cliff_flag |= CLIFF_R_F;
    }
    
    if(rightB > data_rb - CLIFF_DENGER_NUM)
    {
      CliffLog("rightB denger %d > %d\r\n",rightB , data_rb - CLIFF_DENGER_NUM);
      cliff_flag |= CLIFF_R_B;
    }
		
    
    switch(cliff_flag)
    {
      
      case CLIFF_NONE:
      {
        *p_cliff_status = (uint8_t)E_CLIFF_NONE;
        *p_suspend_status = (uint8_t)E_CLIFF_NONE;
      }
      break;
      case CLIFF_L_F:
      {
        *p_cliff_status = (uint8_t)E_CLIFF_L1_F;
      }
      break;
      case CLIFF_L_B:
      {
        *p_cliff_status = (uint8_t)E_CLIFF_L1_B;
      }
      break;
      case CLIFF_R_B:
      {
        *p_cliff_status = (uint8_t)E_CLIFF_R1_B;
      }
      break;
      case CLIFF_R_F:
      {
        *p_cliff_status = (uint8_t)E_CLIFF_R1_F;
      }
      break;
      case CLIFF_L:
      {
        *p_cliff_status = (uint8_t)E_CLIFF_L2;
      }
      break;
      case CLIFF_R:
      {
        *p_cliff_status = (uint8_t)E_CLIFF_R2;
      }
      break;
      case CLIFF_F:
      {
        *p_cliff_status = (uint8_t)E_CLIFF_F2;
      }
      break;
      case CLIFF_B:
      {
        *p_cliff_status = (uint8_t)E_CLIFF_B2;
      }
      break;
      case CLIFF_ALL:
      {
        if(leftF > CLIFF_DANGER_SUSPEND_VALUE && leftB > CLIFF_DANGER_SUSPEND_VALUE && rightB > CLIFF_DANGER_SUSPEND_VALUE && rightF > CLIFF_DANGER_SUSPEND_VALUE)
        {
          *p_suspend_status = (uint8_t)FLAG_ON;
        }
      }
      break;
      
      default:
        *p_cliff_status = (uint8_t)E_CLIFF_OTH;
        break;
    }        
		if(leftF > CLIFF_DANGER_SUSPEND_VALUE && leftB > CLIFF_DANGER_SUSPEND_VALUE && rightB > CLIFF_DANGER_SUSPEND_VALUE && rightF > CLIFF_DANGER_SUSPEND_VALUE)
    {
			*p_suspend_status = (uint8_t)FLAG_ON;
		}
    return FLAG_OK;
	}
  else
  {
    return FLAG_ERR; //err or busy
  }
}

/**/





void cliff_param_init(void)
{
  memset(&tCliffParam,0,sizeof(T_CLIFF_PARAM_TYPDEF));
}

void cliff_check_en(void)
{
  T_CLIFF_PARAM_TYPDEF *pt_cliff_param = &tCliffParam;
  pt_cliff_param->cliff_check_en = FLAG_ON;
}

void cliff_check_den(void)
{
  T_CLIFF_PARAM_TYPDEF *pt_cliff_param = &tCliffParam;
  pt_cliff_param->cliff_check_en = FLAG_OFF;
}




void cliff_task_task(void *param)
{
  T_CLIFF_PARAM_TYPDEF *pt_cliff_param = &tCliffParam;
  uint8_t move_opt_flag = 0;
  uint8_t err = 0;
 
  if(pt_cliff_param ->cliff_check_en == FLAG_OFF)
  {
    return;
  }
  
  
  //read cliff flag ,read suspend flag
  err = cliff_suspend_status_read(&pt_cliff_param->cliff_cur_flag, &pt_cliff_param->suspend_cur_flag);
  

  //cliff 
  if(pt_cliff_param->cliff_last_flag != pt_cliff_param->cliff_cur_flag)
  {
    pt_cliff_param->cliff_last_flag = pt_cliff_param->cliff_cur_flag;
    
    if(FLAG_OFF != pt_cliff_param->cliff_cur_flag)    // stop move
    {
      //move opt 
      move_opt_flag = move_cliff_control(&pt_cliff_param->last_move_flag);
    }
    
    //up cliff flag on
    printf("AT+INT,cliff,%d\r\n",pt_cliff_param->cliff_cur_flag);
  }
  
  
  //suspend
  if(pt_cliff_param->suspend_last_flag != pt_cliff_param->suspend_cur_flag)
  {
    pt_cliff_param->suspend_last_flag = pt_cliff_param->suspend_cur_flag;
    
    //up cliff flag on
    printf("AT+INT,suspend,%d\r\n",pt_cliff_param->suspend_cur_flag);
  }
  
}

     

void cliff_task_init(void)
{
  //cliff init
  cliff_param_init();
  CLIFF_DENGER_NUM=100;
	
  //cliff en
  cliff_check_en();
  
  //task init
  l81_tim_task_creat(ID_CLIFF, TIM_TASK_CYCLE_ALL, CLIFF_CYCLE_TIM_CHECK, NULL, cliff_task_task);

  //start task
  l81_tim_task_en(ID_CLIFF);

  CliffLog("cliff tast init\r\n");
	
	//  //task init   AW_CYCLE_TIM_CHECK=50
//  l81_tim_task_creat(ID_AWset, TIM_TASK_CYCLE_ALL, AW_CYCLE_TIM_CHECK, NULL, aw9310x_click_cb);
//  l81_tim_task_creat(ID_AW, TIM_TASK_CYCLE_ALL, AW_CYCLE_TIM_CHECK, NULL, KeyNumoutToSOC);
//  //start task
//  l81_tim_task_en(ID_AWset);
//  l81_tim_task_en(ID_AW);
	
  l81_tim_task_creat(ID_AW, TIM_TASK_CYCLE_ALL, AW_CYCLE_TIM_CHECK, NULL, AW_check_task);
  l81_tim_task_en(ID_AW);
	


//  CliffLog("ID_AW tast init\r\n");
}


uint8_t l81_AT_CfgR_func_CLIFF_DENGER_NUM(void)
{
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,%d\r\n",CLIFF_DENGER_NUM);
	printf("AT+RES,end\r\n");
	return 1u;
}
uint8_t l81_AT_CfgW_func_CLIFF_DENGER_NUM(uint32_t danger_num)
{
	if(danger_num < 50 || danger_num > 300)
	{
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param danger_num,[50,300]\r\n");
		printf("AT+RES,end\r\n");
		return 0u;
	}
	CLIFF_DENGER_NUM = danger_num;
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,%d\r\n",CLIFF_DENGER_NUM);
	printf("AT+RES,end\r\n");
	return 1u;
}



