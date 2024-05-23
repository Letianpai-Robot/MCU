#ifndef __AW9310X_H
#define __AW9310X_H
#include "aw_algo_struct.h"
#include "L81_AT.h"

#ifdef __cplusplus
extern "C" {
#endif
// #define AW_OS_USED  // driver will use OS if this macro is defined.

#ifdef AW_OS_USED
/***
 * this macro is used for app(Awinic Sensor Debug Tool) debug. Commonication between app and driver is realized by SPP.
 * notice that, OS must be used if spp need to be used.
 */
// #define AW_SPP_USED
#endif
/* if this macro is defined, driver will get event(single\double.. click) base on algorithm, if not, event is got base on interrupt. */
// #define AW_ALGO_USED
extern void TRACE(char *format, ...);
#define AW_factory_USED
//#define AW9310X_USE_HAL_DRIVER_LOG
#ifdef AW9310X_USE_HAL_DRIVER_LOG
//#include "hal_trace.h"
#define AW9310X_DBG(s, ...)    (TRACE("[AWINIC DBG] %s %s " s, __func__,__TIME__, ##__VA_ARGS__))
//#define AW9310X_DBG(s, ...)    (TRACE_DUMMY(0, s, ##__VA_ARGS__))
#define AW9310X_INF(s, ...)    (TRACE("[AWINIC INF] %s %s " s, __func__,__TIME__, ##__VA_ARGS__))
#define AW9310X_ERR(s, ...)    (TRACE("[AWINIC ERR] %s %s " s, __func__,__TIME__, ##__VA_ARGS__))
#define AW_DEBUG(s, ...)     	(TRACE("[AWINIC DEMO DBG] %s %s " s, __func__,__TIME__, ##__VA_ARGS__))
#else
#define AW9310X_DBG(s, ...)    TRACE_DUMMY(0, s, ##__VA_ARGS__)
#define AW9310X_INF(s, ...)    TRACE_DUMMY(0, s, ##__VA_ARGS__)
#define AW9310X_ERR(s, ...)    TRACE_DUMMY(0, s, ##__VA_ARGS__)
#define AW_DEBUG(s, ...)     	(TRACE_DUMMY(0, s, ##__VA_ARGS__)
#endif

#define AW9310X_BYTE_TO_INT(p)   (AW_U32)(((((AW_U32)(*p)) << AW_BIT24) | (((AW_U32)*(p - 1))) << AW_BIT16) | (((AW_U32)*(p - 2)) << AW_BIT8) | *(p - 3));

#define AW9310X_CLICK_CHANNEL                   (0)
#define AW9310X_PRESS_CHANNEL                   (0)
#define AW9310X_WEAR_CHANNEL_0                  (3)
#define AW9310X_WEAR_CHANNEL_1                  (4)

#define AW_EN                                   (0x01)
#define SLIDE_EN                                (1)
#define LINGER_SLIDER                           (0)
#define SLIDER_REUSED                           (1)
#define NORMAL_SLIDE                            (0)
#define SLIDE_NUM                               (3)
#define PACKS_IN_ONE_COMM		        		(3)
#define AW9310X_DATA_CLEAR                      (0)
#define AW9310X_CHANNEL_MAX						(5)
#define AW9310X_CPU_WORK_MASK					(1)
#define AW9310X_ALGO_RUN_INTERVAL               (20)
#define AW9310X_ADDR_DEV0						(0x12)
#define AW9310X_I2C_RETRIES                     (5)
#define AW9310X_CHIP_ID							(0xa961)
#define AW9310X_READ_IRQ_STAT                   (2)
#define AW9310X_FAR_AWAY_APPORACH_ENABLE        (0x00000006)
#define AW9310X_FAR_AWAY_APPORACH_DISABLE       (0x00000000)
#define AW9310X_POLLING_MAX_TIME                (500)
#define AW_CHIP_SAR                             (2)
#define AW_SAR_STATUS_LEN                       (8)
#define AW_SAR_REG_LEN                          (16) //reg len : 16bits
#define AW_SAR_DATA_LEN                         (32) //data len : 32bits
#define AW_CURVE_DATA_LEN                       (32) //curve len : 32bits
#define AW_COMM_CYCLE                           (90)
#define DATA_MAX_LEN                            (512)
#define AW_SPP_REG_MAX_NUM                      (10)
#define SAR_REG_DATA_LEN                        (4)
#define SAR_APP_CURVE_CNT                       (2)
#define AW_SAR_SPP_ONE_PACK_LEN                 (SAR_APP_CURVE_CNT * AW_CURVE_DATA_LEN + AW_SAR_STATUS_LEN)
#define PROXTH0_CH_0_1_2_EN                     (100000)
#define PROXTH0_CH_3_4_5_EN                     (100000)
#define PROXTH1_CH_0_1_2_EN                     (200000)
#define PROXTH1_CH_3_4_5_EN                     (200000)
#define CLICK_TIME                              (30)
#define CLICK_INTERVAL                          (30)
#define LONG_PRESS_TH                           (100)
#define SUPER_LONG_PRESS_TH                     (200)
#define AW_SEND_INTN_DATA_LEN                   (32 * 3 + 4 + 2)
#define AW_MAX_CLICK_SUPPORT                    (3)
#define AW_LONG_PRESS_CNT	                    (2)
#define AW_SUPER_LONG_PRESS_CNT		            (5)
#define AW9310X_SLEEP            				(0x0002)
#define AW9310X_GPIO_DIRIN       				(0)
#define AW9310X_GPIO_DIRIN_EN    				(0x001f)
#define AW9310X_GPIO_PUU_EN      				(0x001f)
#define AW9310X_GPIO_PUD_DIS     				(0)
#define AW9310X_GPIO_PUU_DIS     				(0)
#define AW9310X_GPIO_PUD_EN      				(0x001f)
#define AW9310X_CAP_MIN                         (-6.5f)
#define AW9310X_CAP_MAX                         (6.5f)
#define AW9310X_OFFSET_MAX                      (1000)
#define AW9310X_OFFSET_MIN                      (100)
#define AW9310X_DIFF_PROX_MIN                   (0)
#define AW9310X_DIFF_PROX_MAX                   (1000000)
#define AW9310X_REDUNDANCY_DATA_LEN             (10)
#define AW9310XA_FLAG                           (0x03000B00)

enum aw9310x_event_param {
	AW9310X_PRESS_INTERVAL = 1000,
	AW9310X_CLICK_INTERVAL = 500,
};

enum aw9310x_signal {
	AW9310X_ALGO_RUN = 1, // software algorithm
	AW9310X_SPP_CURVE_RUN, // get data by spp
	AW9310X_SPP_GET_INTN_DATA_APPROACH_RUN,
	AW9310X_SPP_GET_INTN_DATA_FAR_AWAY_RUN,
	AW9310X_SPP_GET_INTN_EVENT_RUN,
};

enum aw9310x_ret_state {
	AW_OK,
	AW_ERROR,
	AW_BUSY,
	AW_TIMEOUT,
	AW_FAIL,
};

enum aw9310x_sar_vers {
	AW9310X = 2,
	AW9310XA = 6,
};

enum aw9310x_operation_mode {
	AW9310X_ACTIVE_MODE = 1,
	AW9310X_SLEEP_MODE,
	AW9310X_DEEPSLEEP_MODE,
	AW9310XB_DEEPSLEEP_MODE,
};

enum aw9310x_event_list {
	AW_APP_EVENT_SINGLE_CLICK,
	AW_APP_EVENT_DOUBLE_CLICK,
	AW_APP_EVENT_TRIPLE_CLICK,
	AW_APP_EVENT_LONG_PRESS,
	AW_APP_EVENT_SUPER_LONG_PRESS,
	AW_APP_EVENT_WEAR,
	AW_APP_EVENT_RIGHT_SLIDE,
	AW_APP_EVENT_LEFT_SLIDE,
};

/**********************************************
*spereg addr offset
**********************************************/
enum aw9310x_spereg_addr_offset {
	AW_CL1SPE_CALI_OS = 20,
	AW_CL1SPE_DEAL_OS = 60,
	AW_CL2SPE_CALI_OS = 4,
	AW_CL2SPE_DEAL_OS = 4,
};

enum aw9310x_irq_stat {
	AW_APPROACH = 0x01,
	AW_FAR_AWAY = 0x02,
};

enum aw9310x_bit {
	AW_BIT0,
	AW_BIT1,
	AW_BIT2,
	AW_BIT3,
	AW_BIT7 = 7,
	AW_BIT8 = 8,
	AW_BIT12 = 12,
	AW_BIT16 = 16,
	AW_BIT24 = 24,
	AW_BIT28 = 28,
	AW_BIT32 = 32,
};

enum aw9310x_word {
	AW_BIT_0_1_EN = 0x03,
	AW_ONE_BYTE_1 = 0xff,
	AW_BIT_10_EN = 0x00000400,
	AW_BIT_11_EN = 0x00000800,
	AW_BIT_12_EN = 0x00001000,
	AW_HALF_WORD_1 = 0xffff,
	AW_THREE_BYTE_1 = 0xffffff,
	AW_ONE_WORD_1 = 0xffffffff,
};

enum aw9310x_algo_flag {
	AW_ALGO_CLICK_PRESS_FLAG = 0x00001000,
	AW_ALGO_SLIDE_FLAG = 0x00000800,
	AW_ALGO_WEAR_FLAG = 0x00000400,
};

enum {
	AW_APP_CMD_GET_DEVICE_INFO = 0x00,
	AW_APP_CMD_GET_CURVE_DATA = 0x01,
	AW_APP_CMD_READ_REG = 0x02,
	AW_APP_CMD_WRITE_REG = 0x03,
	AW_APP_CMD_GET_ALGO_PARA = 0x04,
	AW_APP_CMD_SET_ALGO_PARA = 0x05,
	AW_APP_CMD_GET_INTN_DATA = 0x06,
	AW_APP_CMD_GET_INTN_EVENT = 0x07,
	AW_APP_CMD_STOP_GET_CURVE_DATA = 0x08,
	AW_APP_CMD_START_GET_CURVE_DATA = 0x09,
	AW_APP_CMD_OFFSET_CALI = 0xa0,
	AW_APP_CMD_DIFF_TO_AIR = 0xa1,
	AW_APP_CMD_DIFF_APPROACH = 0xa2,
	AW_APP_CMD_SHORT_CIRCUIT = 0xa3,
	AW_APP_CMD_SET_DIFF_TH = 0xa4,
};

typedef enum {
	APK_VALID_DATA_HEADER = 3,
	APK_HEADER = 0x3A,
	OFFSET = 0x82,
	NOISE,
	SIGNAL_RAW_0,
	SIGNAL_RAW_1,
	VERIFY_0,
	VERIFY_1,
	DYNAMIC_CALI = 0x8C,
	APK_END0 = 0x0D,
	APK_END1 = 0x0A,
} CALI_FLAG_T;

struct aw9310x_event {
	AW_U32 approach_state; //1: approach  0: far away
	AW_U32 approach_cnt;
	AW_U32 far_away_cnt;
};

typedef AW_U32 (*aw9310x_i2c_w_t)(AW_U16, AW_U32);
typedef AW_U32 (*aw9310x_i2c_r_t)(AW_U16, AW_U32 *);
typedef void (*aw9310x_irq_init_t)(void(*)(void));
typedef void (*aw9310x_create_thread_t)(void (*)(void));
typedef void (*aw9310x_set_signal_t)(AW_U32);
typedef AW_U32 (*aw9310x_wait_signal_forever_t)(void);
typedef void (*aw9310x_delay_t)(AW_U32);
typedef void (*aw9310x_log_print_t)(char *, ...);
typedef void (*aw9310x_timer_created_t)(void (*)(void *));
typedef void (*aw9310x_timer_start_t)(AW_U32);
typedef void (*aw9310x_timer_stop_t)(void);
typedef void (*aw9310x_spp_write_t)(AW_U8 *, AW_U16);
typedef void (*aw9310x_spp_init_t)(void (*)(AW_U8 *, AW_U16));
typedef void (*aw9310x_flash_read_t)(AW_U32 , AW_U8 *, AW_U32);
typedef void (*aw9310x_flash_write_t)(AW_U32 , AW_U8 *, AW_U32);

struct aw9310x_i2c_func {
	aw9310x_i2c_r_t i2c_r;
	aw9310x_i2c_w_t i2c_w;
};

struct aw9310x_timer_func {
	aw9310x_timer_created_t timer_create;
	aw9310x_timer_start_t timer_start;
	aw9310x_timer_stop_t timer_stop;
};

struct aw9310x_thread_func {
	aw9310x_create_thread_t create_thread;
	aw9310x_set_signal_t set_signal;
	aw9310x_wait_signal_forever_t wait_signal_forever;
};

struct aw9310x_spp_func {
	aw9310x_spp_write_t spp_write;
	aw9310x_spp_init_t spp_init;
};

struct aw9310x_flash_ops_func {
	aw9310x_flash_read_t  flash_read;
	aw9310x_flash_write_t flash_write;
};

struct aw9310x_func {
	struct aw9310x_i2c_func i2c_func;
	aw9310x_irq_init_t irq_init;
#ifdef AW_OS_USED
	struct aw9310x_thread_func irq_thread_func;
#endif
	aw9310x_delay_t delay;
	aw9310x_log_print_t log_print;

#if defined(AW_OS_USED) && (defined(AW_ALGO_USED) || defined(AW_SPP_USED))
	struct aw9310x_thread_func work_thread_func;
#endif
#ifdef AW_ALGO_USED
	struct aw9310x_timer_func algo_timer;
#else
	struct aw9310x_timer_func click_timer;
	struct aw9310x_timer_func press_timer;
#endif
#ifdef AW_SPP_USED
	struct aw9310x_spp_func spp_func;
#endif
	struct aw9310x_flash_ops_func flash_func;
};

struct aw9310x {
	AW_U8 vers;
	AW_U8 old_mode;
	AW_U8 mode;
	AW_U8 channel_num;
	volatile AW_U8 is_getting_curve_data;
	AW_U8 satu_flag[6];
	AW_U8 chip_name[9];
	AW_U8 events[8];
	AW_U32 g_det_cnt;
	AW_U32 irq_status;
	AW_U32 satu_data[6];
	struct aw_algo_params algo_param;
#ifndef AW_ALGO_USED
	struct aw9310x_event click_event;
	struct aw9310x_event press_event;
#endif
	struct aw9310x_event wear_event[2];
};

typedef struct aw_prf
{
	AW_U8 cmd;
	AW_U8 dat[DATA_MAX_LEN];
	AW_U8 len;
} aw_prf_t;


extern struct aw9310x_func aw9310x_demo_func;

void aw9310x_operation_mode_set(AW_U8 mode);
AW_S32 aw9310x_auto_cali(void);
AW_U8 aw9310x_read_diff(AW_S32 *diff);
//driver init, this func is called by main thread.
AW_U32 aw9310x_init(struct aw9310x_func *hw_func);
void aw9310x_diff_approach(void);
void aw9310x_diff_to_air(void);
void aw9310x_offset_cali(void);
void aw9310x_short_circuit_detect(void);

void aw9310x_click_timer_cb(void);
void aw9310x_click_cb(void *param);
void aw9310x_press_timer_cb(void);
void aw9310x_irq_cb(void);

uint8_t l81_AT_AW_R_func(char params[]);
uint8_t l81_AT_AW_START_func(char params[]);
uint8_t l81_AT_AW_STOP_func(char params[]);
void AW_read_task(void );
void KeyNumoutToSOC(void *param );
//int KeyNumoutToSOC();
void AW_check_task(void *param );

extern AW_U8 clickType;
#ifdef __cplusplus
}
#endif

#endif
