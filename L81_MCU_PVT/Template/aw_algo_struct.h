#ifndef __AW_ALGO_STRUCT_H__
#define __AW_ALGO_STRUCT_H__

#include "aw_type.h"

#define AW_MAX_SENSOR_NUM			(6)
#define CNT_MAX						(0xffff)
#define AFE_DATA_MAX_SHIFT_BITS		(10)
#define DIFF_OFFSET_SHIFT_BITS		(20)
#define AW_PROXTH_NUM_MAX			(4)

/* This structure is used to receive parameters from outside */
struct aw_algo_params {
	AW_U8 ssen; // channal enable scan or not
	AW_U8 slide_en; // Slide enable or not
	AW_U8 slide_shape; // Linear slide or ring slide
	AW_U8 slide_btn_en; // Slide reused button or not
	AW_U8 slide_mode; // normal slide or con slide
	AW_U8 slide_num; // The number of pattern for slide
	AW_U8 slide_btn_fun; // single click/double click/treblic click/long press/super long press
	AW_U8 slide_resolution;

	AW_U8 slide_touch_sel; // single pattern state judge or all patterns state judge
	AW_U8 slide_touch_deb; // slide in and out touch state debounce
	AW_U32 slide_touch_th; // slide touch threshold

	AW_U8 con_slide_mode_sel; // report slide event or report position
	AW_U8 con_slide_in_deb;
	AW_U8 con_slide_distance_th;
	AW_U8 con_slide_in_time_th;

	AW_U8 norm_slide_judge_sel; // use move_distance or pos offset or diff offset
	AW_U8 norm_slide_th;
	AW_U16 norm_slide_speed_th;

	AW_U16 click_time_th; // Time threshold of click when touch
	AW_U16 click_interval_th; // Time threshold of doubel/treble click when leave

	AW_U16 long_press_th; // Time threshold of long press
	AW_U16 super_long_press_th; // Time threshold of super long press
	AW_U16 s_super_long_press_th; // Time threshold of super super long press

	AW_U8 button_fun_en; // Bit[i]->CHi, i = 0~5
	AW_U8 button_fun[AW_MAX_SENSOR_NUM];
	AW_U8 button_th_sel[AW_MAX_SENSOR_NUM]; // Proxth0/Proxth1/Proxth2/Proxth3

	AW_U8 wear_fun_en; // Bit[i]->CHi, i = 0~5
	AW_U8 soft_wear_th_sel[AW_MAX_SENSOR_NUM];
	AW_U8 hard_wear_th_sel[AW_MAX_SENSOR_NUM];
};
typedef struct aw_algo_params AW_ALGO_PATAM_T;

/* This structure is used to send data to UI */
struct aw_data_send {
	AW_U8 AnyWearProxST;
	AW_U8 AnyWearTouchST;
	AW_U8 WearST;

	AW_U8 SlideProxST;
	AW_U8 SlideTouchST;
	AW_U8 NormDirST;
	AW_U8 NormSpeedST;
	AW_U8 NormSlideST;
	AW_U8 ConSlideST;
	AW_U8 ConDirectionST;

	AW_U8 AnyButtonProxST;
	AW_U8 AnyButtonTouchST;
	AW_U16 SSuperLongPressST;
	AW_U16 SuperLongPressST;
	AW_U16 LongPressST;
	AW_U16 TrebleClickST;
	AW_U16 DoubleClickST;
	AW_U16 SingleClickST;

	AW_U8 SlidePosition;
	AW_U8 DiffOffset;
	AW_U8 PosOffset;
	AW_U8 MoveDistance;

	AW_U32 irq_state;
	AW_S32 Diff[AW_MAX_SENSOR_NUM];
};
typedef struct aw_data_send AW_DATA_SEND_T;

/* This structure is used to save flag or inter data*/
struct aw_data_store {
	AW_U8 SlideTouchCloseCnt[AW_MAX_SENSOR_NUM]; // slide touch single judge
	AW_U8 SlideTouchFarCnt[AW_MAX_SENSOR_NUM];
	AW_U8 SSlideTouchCloseCnt; // slide touch state judge diff sum
	AW_U8 SSlideTouchFarCnt;

	AW_S32 DiffMax[AW_MAX_SENSOR_NUM];
	AW_S32 WDiff[AW_MAX_SENSOR_NUM][AW_MAX_SENSOR_NUM];
	AW_U8  DiffMaxPos[AW_MAX_SENSOR_NUM];
	AW_U8  SoftWearState;
	AW_U8  HardWearState;
	AW_U8  WearChannalEn;
	AW_U8  PositionMax;
	AW_U8  PositionMin;
	AW_U8  PreConPosition;
	AW_U16  PositionMaxTouchCnt;
	AW_U16  PositionMinTouchCnt;
	AW_U8  SlideChannalEn;
	AW_U8  BtnChannalEn;
	AW_U8  PreWearState;
	AW_U8  TouchState; // slide touch state
	AW_U8  PreTouchState;
	AW_U8  BtnState;
	AW_U8  PreBtnState;
	AW_U8  BtnTouchCnt[AW_MAX_SENSOR_NUM];
	AW_U8  BtnFarCnt[AW_MAX_SENSOR_NUM];
	AW_U8  BtnClickCnt[AW_MAX_SENSOR_NUM];
	AW_U8  EnterConSlideCnt;
	AW_U8  ConClickCnt;
	AW_U8  ConSlideFlag;
	AW_U8  ConSlideStartFlag;
	AW_U8  ConSlideEndFlag;

	AW_U16  LongPressFlag;
	AW_U16  SuperLongPressFlag;
	AW_U16  SSuperLongPressFlag;
	AW_U8  ClickJudgeFlag;
	AW_U16 FarCnt;
	AW_U16 TouchCnt;
	AW_U8  ThStat[AW_PROXTH_NUM_MAX];
	AW_S32 Diff[AW_MAX_SENSOR_NUM];
	AW_DATA_SEND_T data_send;
};
typedef struct aw_data_store AW_DATA_STORE_T;

/* Get algorithm parameter structure address */
AW_ALGO_PATAM_T *aw_get_algo_param(void);
/* Get the structure of the intermediate parameters of the algorithm */
AW_DATA_STORE_T *aw_get_data_store(void);
/* Set algo param */
AW_BOOL aw_set_algo_param(AW_ALGO_PATAM_T *p_param_lib);
/* Get algo param*/
void aw_get_alg_param(AW_ALGO_PATAM_T *p_param_lib);
/* Get algo fun result*/
void aw_get_algo_fun_result(AW_DATA_SEND_T *p_data);
/* Algo function */
AW_BOOL aw_algo_fun(AW_S32 *diff, AW_U8 *th_stat, AW_DATA_SEND_T *p_data, AW_DATA_STORE_T *p_store);
/* Get algo version */
AW_U32 aw_get_algo_ver(void);
#endif
