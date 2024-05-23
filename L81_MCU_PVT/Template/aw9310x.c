/*
* ©2022 awinic All Rights Reserved
* Description: aw9310x mcu driver code.
*/
#include "string.h"
#include "stdint.h"
#include "stdio.h"
#include "aw9310x.h"
#include "aw_type.h"
#include "aw9310x_reg.h"
#include "aw_algo_struct.h"
#include <stdio.h>
#include <stdarg.h>
#include "i2c_inf.h"
#include "L81_inf.h"
#include "systick.h"

//#define AW_ALGO_USED
#define USE_MULIT_TIMER			0
void TRACE(char *format, ...)
{
	va_list args_list;
	va_start(args_list , format); //va_start 的第一个参数为va_list变量，第二个参数为函数的最后一个固定参数
	char buff[1024];
	vsnprintf(buff, 1023 , format , args_list);
	printf("%s\n",buff);
	va_end(args_list);
}

void TRACE_DUMMY(char *format, ...)
{
	__NOP();
}

#define AW9310X_DRIVER_VERSION "v1.2.0"

struct aw9310x_func *g_aw9310x_func;
struct aw9310x g_aw9310x;

static uint8_t  key_state_curr 		= 0;
static uint8_t  aw_state_change 	= 0;
static void aw9310x_wear_judge(AW_U32 chl);
static void aw9310x_get_irq_stat(void);
#ifdef AW_ALGO_USED
static void aw9310x_detect(void);
#endif

#ifdef AW_SPP_USED
static AW_U8 aw9310x_read_app_data(AW_S32 *app_data);
#endif

#if defined (AW_SPP_USED) && defined(AW_ALGO_USED)
static void aw9310x_app_send_curve_dat(void);
#endif

#if defined(AW_OS_USED) && (defined(AW_ALGO_USED) || defined(AW_SPP_USED))
static void aw9310x_work_set_signal(AW_U32 sig);
#endif

#if defined(AW_SPP_USED) && (!defined(AW_ALGO_USED))
void aw9310x_send_intn_event(void);
void aw9310x_send_intn_data(enum aw9310x_irq_stat state, AW_U32 irq_reg);
#endif

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void aw_timer_nvic_config(void)
{
    //nvic_irq_enable(TIMER2_IRQn, 0);
    nvic_irq_enable(TIMER6_IRQn, 0);
}

/*!
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void aw_click_timer_config(void)
{
    /* TIMER1 configuration: input capture mode -------------------
    the external signal is connected to TIMER1 CH0 pin (PA0)
    the rising edge is used as active edge
    the TIMER1 CH0CV is used to compute the frequency value
    ------------------------------------------------------------ */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;

    /* enable the peripherals clock */
    rcu_periph_clock_enable(RCU_TIMER6);

    /* deinit a TIMER */
    timer_deinit(TIMER6);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
	//    tim_struct.prescaler = (108 - 1);  // 预分频：108MHz / 108 = 1MHz
    //tim_struct.period = (1000 - 1);  // 周期：1000 / 1MHz = 1ms
	
    timer_initpara.prescaler        = 639;				//64m/640=100k
    timer_initpara.alignedmode      = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period           = 999;				//1000/100k=10ms
    timer_initpara.clockdivision    = TIMER_CKDIV_DIV1;
    timer_init(TIMER6, &timer_initpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER6);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER6, TIMER_INT_FLAG_UP);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER6, TIMER_INT_UP);

    /* enable a TIMER */
    timer_enable(TIMER6);
/////////////////////////////////////////////////////////////////////////		
}


static void aw9310x_click_timer_create(void (*timer_cb)(void))
{
	aw_click_timer_config();//aw9310x_click_timer_id = hwtimer_alloc((HWTIMER_CALLBACK_T)timer_cb, NULL);
}
#if USE_MULIT_TIMER
static void aw9310x_click_timer_start(AW_U32 ms)
{
	//hwtimer_start(aw9310x_click_timer_id, MS_TO_TICKS((int)ms));
	    timer_enable(TIMER6);
}
static void aw9310x_click_timer_stop(void)
{
	timer_disable(TIMER6);//hwtimer_stop(aw9310x_click_timer_id);
}

// timer for press
//static HWTIMER_ID aw9310x_press_timer_id;
/*!
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void aw_press_timer_config(void)
{
	    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;

		    /* enable the peripherals clock */
    rcu_periph_clock_enable(RCU_TIMER2);

    /* deinit a TIMER */
    timer_deinit(TIMER2);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler        = 6399;
    timer_initpara.alignedmode      = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period           = 9999;
    timer_initpara.clockdivision    = TIMER_CKDIV_DIV1;
    timer_init(TIMER2, &timer_initpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);
}

static void aw9310x_press_timer_create(void (*timer_cb)(void))
{
	aw_press_timer_config();//aw9310x_press_timer_id = hwtimer_alloc((HWTIMER_CALLBACK_T)timer_cb, NULL);
}

static void aw9310x_press_timer_start(AW_U32 ms)
{
	;//hwtimer_start(aw9310x_press_timer_id, MS_TO_TICKS((int)ms));
	    /* enable a TIMER */
    timer_enable(TIMER2);

}

static void aw9310x_press_timer_stop(void)
{
	;//hwtimer_stop(aw9310x_press_timer_id);
    /* enable a TIMER */
    timer_disable(TIMER2);
}
#endif
static void aw9310x_delay(AW_U32 ms)
{
	delay_1ms(ms);
}

static AW_U32 aw9310x_i2c_read(AW_U16 reg_addr, AW_U32 *reg_data)
{
		uint32_t ret = 0;
	uint8_t reg_val[4] = {0};

//	ret = master_burst_read(0x12, reg_addr, reg_val, 4,1);
	sw_i2c_send2read_16bit(0x12, reg_addr, reg_val, 4);
	*reg_data = ((uint32_t)reg_val[0] << 24) | ((uint32_t)reg_val[1] << 16) |
				((uint32_t)reg_val[2] << 8) | ((uint32_t)reg_val[3] << 0);
//ret = 1;
	return ret == 0 ? AW_OK : AW_ERROR;
}

static AW_U32 aw9310x_i2c_write(AW_U16 reg_addr, AW_U32 reg_data)
{
	uint8_t j;
		uint32_t ret = 0;
	uint8_t reg_val[4] = {0};

	
	//7bit address should left shifft 1 bit
	AW_U8 slave_addr =(0x12<<1);//<<= 1;
	
	sw_i2c_start();
	
	sw_i2c_writeByte(slave_addr);
	
	if (!sw_i2c_wait_ack())
		goto err;
	
	reg_val[0] = reg_addr >> 8 & 0xff;
	reg_val[1] = reg_addr >> 0 & 0xff;
	for (j = 0u; j < 2; j++){
		sw_i2c_writeByte(*(reg_val + j));
		if (!sw_i2c_wait_ack())
			goto err;		
	}
	
	reg_val[0] = reg_data >> 24 & 0xff;
	reg_val[1] = reg_data >> 16 & 0xff;
	reg_val[2] = reg_data >> 8 & 0xff;
	reg_val[3] = reg_data >> 0 & 0xff;
	for (j = 0u; j < 4; j++){
		sw_i2c_writeByte(*(reg_val + j));
		if (!sw_i2c_wait_ack())
			goto err;		
	}
	err:
	sw_i2c_stop();	
	
	return ret == 0 ? AW_OK : AW_ERROR;
	
}

static AW_S32 aw9310x_i2c_write_bits(AW_U16 reg_addr16, AW_U32 mask, AW_U32 reg_data32)
{
	AW_U32 reg_val;
	AW_S32 ret = 0;

	ret = aw9310x_i2c_read(reg_addr16, &reg_val);
	if (ret != AW_OK) {
		AW9310X_ERR("reg_addr16 = %x reg_val = %x\n i2c_read error",  reg_addr16, reg_val);
		return ret;
	}

	reg_val &= mask;
	reg_val |= (reg_data32 & (~mask));
	aw9310x_i2c_write(reg_addr16, reg_val);
	if (ret != AW_OK) {
		AW9310X_ERR("reg_addr16 = %x reg_val = %x\n i2c_write error",  reg_addr16, reg_val);
		return ret;
	}

	return AW_OK;
}

static void aw9310x_irq_init(void (*irq_cb)())
{
	    /* enable the WAKEUP key gpio clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* configure button pin as input */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_12);
	    /* enable the SYSCFG clock */
    rcu_periph_clock_enable(RCU_SYSCFG);
    /* enable and set key EXTI interrupt to the specified priority */
    nvic_irq_enable(EXTI10_15_IRQn, 1U);

    /* connect key EXTI line to key GPIO pin */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN12);

    /* configure key EXTI line */
    exti_init(EXTI_12, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_12);	
}

#ifdef AW_OS_USED
static void aw9310x_irq_thread_create(void (*thread_cb)(void))
{
	g_aw9310x_func->irq_thread_func.create_thread(thread_cb);
}

static void aw9310x_irq_set_signal(AW_U32 sig)
{
	g_aw9310x_func->irq_thread_func.set_signal(sig);
}

static AW_U32 aw9310x_wait_irq_signal_forever(void)
{
	return g_aw9310x_func->irq_thread_func.wait_signal_forever();
}

static void aw9310x_irq_thread_cb(void)
{
	AW_U32 ret = 0;

	while(1) {
		ret = 0;
		ret = aw9310x_wait_irq_signal_forever();
		if (ret == AW9310X_READ_IRQ_STAT) {
			aw9310x_get_irq_stat();
		}
	}
}
#endif

static AW_S32 aw9310x_read_chipid(void)
{
	AW_U8 cnt = 0;
	AW_S32 ret = -1;
	AW_U32 data = 0;

	while (cnt < AW9310X_I2C_RETRIES) {
		ret = aw9310x_i2c_read(REG_CHIP_ID, &data);
		if (ret < 0) {
			AW9310X_ERR("read CHIP ID failed: %d", ret);
		}
		data = data >> AW_BIT16;
		if ((AW_U16)data == AW9310X_CHIP_ID) {
			AW9310X_INF("read CHIP ID ok, 0x%04x", data);
			return AW_OK;
		} else {
			AW9310X_ERR("unsupport dev (0x%04x)", data);
		}
		cnt++;
		aw9310x_delay(10);
	}

	return -AW_ERROR;
}

static AW_U32 aw9310x_sw_reset(void)
{
	AW_U32 ret = -1;

	AW9310X_INF( "enter");

	ret = aw9310x_i2c_write(REG_HOSTCTRL2, 0);
	if (ret < 0) {
		AW9310X_ERR("REG_HOSTCTRL2 write err");
		return ret;
	}
	aw9310x_delay(20);

	return AW_OK;
}

static AW_U32 aw9310x_param_load(void)
{
	AW_S32 ret = 0;

	for (AW_U8 i = 0; i < sizeof(aw9310x_reg_default) / sizeof(aw9310x_reg_default[0]); i = i + 1) {
		ret = aw9310x_i2c_write(aw9310x_reg_default[i].addr, aw9310x_reg_default[i].data);
		if (ret < 0) {
			AW9310X_ERR("i2c write error addr: 0x%04x, data: 0x%08x",
				aw9310x_reg_default[i].addr, aw9310x_reg_default[i].data);
			return ret;
		}
		AW9310X_DBG("i2c write addr: 0x%04x, data: 0x%08x",
			aw9310x_reg_default[i].addr, aw9310x_reg_default[i].data);
	}

	return ret;
}

AW_S32 aw9310x_auto_cali(void)
{
	AW_U32 data_en = 0;
	AW_S32 ret = -1;

	ret = aw9310x_i2c_read(REG_SCANCTRL0, &data_en);
	if (ret != AW_OK) {
		AW9310X_ERR("i2c_read REG_SCANCTRL0 error");
		return ret;
	}

	ret = aw9310x_i2c_write_bits(REG_SCANCTRL0, ~(0x3f << AW_BIT8), (data_en & 0x3f) << AW_BIT8);
	if (ret != AW_OK) {
		AW9310X_ERR("write_bits REG_SCANCTRL0 error");
		return ret;
	}
	AW9310X_INF("aw9310x_auto_cali finish");

	return AW_OK;
}

void aw9310x_operation_mode_set(AW_U8 mode)
{
	if (mode == AW9310X_ACTIVE_MODE &&
				g_aw9310x.old_mode != AW9310X_ACTIVE_MODE) {
		if (g_aw9310x.old_mode == AW9310X_DEEPSLEEP_MODE) {
			aw9310x_i2c_write(REG_HOSTCTRL1, 1);
		}
		aw9310x_i2c_write(REG_CMD, AW9310X_ACTIVE_MODE);
	} else if (mode == AW9310X_SLEEP_MODE &&
				g_aw9310x.old_mode != AW9310X_SLEEP_MODE) {
		if (g_aw9310x.old_mode == AW9310X_DEEPSLEEP_MODE) {
			aw9310x_i2c_write(REG_HOSTCTRL1, 1);
		}
		aw9310x_i2c_write(REG_CMD, AW9310X_SLEEP_MODE);
	} else if ((mode == AW9310X_DEEPSLEEP_MODE) && g_aw9310x.old_mode != AW9310X_DEEPSLEEP_MODE) {
		aw9310x_i2c_write(REG_CMD, AW9310X_DEEPSLEEP_MODE);
	} else {
		AW9310X_ERR("failed to operation mode!");
	}
	g_aw9310x.mode = mode;
	g_aw9310x.old_mode = g_aw9310x.mode;
}

static AW_U16 diff_regs[] = {REG_DIFF_CH0, REG_DIFF_CH1, REG_DIFF_CH2, REG_DIFF_CH3, REG_DIFF_CH4, REG_DIFF_CH5};
AW_U8 aw9310x_read_diff(AW_S32 *diff)
{
	AW_U32 ret = -1;

	for (AW_U32 i = 0; i < AW9310X_CHANNEL_MAX; i ++) {
		ret = aw9310x_i2c_read(diff_regs[i], (AW_U32 *)&diff[i]);// + (AW_U16)(i*4)

		if (ret < 0) {
			AW9310X_ERR("read reg diff failed: %d", ret);
			return AW_FALSE;
		}else{
			AW9310X_DBG( "aw9310x diff data: ch%d = %x", i, diff[i] >> 10);
		}
	}

	return AW_OK;
}

/**
 * @brief this func used to parse data reviced via spp.
 *
 * @param in data recived
 * @param in_len revived data len.
 * @param out data stored in type 'aw_prf_t'.
 * @return AW_S32 0 if unpack successed. others if unpack error.
 */
static AW_S32 aw9310x_data_unpack(AW_U8 *in, AW_U8 in_len, aw_prf_t *out)
{
	AW_U32 valid_dat_len = 0;
	AW_U64 sum_r = 0;
	AW_U64 sum_v = 0;

	if (in[0] != APK_HEADER) {
		AW9310X_ERR("frame header error, 0x%x.", in[0]);
		return AW_ERROR;
	}

	if (in[in_len - 2] != APK_END0 || in[in_len - 1] != APK_END1) {
		AW9310X_ERR("frame end error, 0x%x 0x%x", in[in_len - 2], in[in_len - 1]);
		return AW_ERROR;
	}

	valid_dat_len = in[2];

	for (AW_U8 i = 0; i < valid_dat_len; i++) {
		sum_v += (long)in[i + 3];
	}
	sum_r = (AW_U64)(((AW_U64)in[APK_VALID_DATA_HEADER + valid_dat_len]) |
			((AW_U64)in[APK_VALID_DATA_HEADER + valid_dat_len + 1]) << AW_BIT8 |
			((AW_U64)in[APK_VALID_DATA_HEADER + valid_dat_len + 2]) << AW_BIT16 |
			((AW_U64)in[APK_VALID_DATA_HEADER + valid_dat_len + 3]) << AW_BIT24 |
			((AW_U64)in[APK_VALID_DATA_HEADER + valid_dat_len + 4]) << AW_BIT32 );
	if (sum_r != sum_v) {
		AW9310X_ERR("sum_r = 0x%lld  sum_v = 0x%lld.", sum_r, sum_v);
		return AW_ERROR;
	}
	out->cmd = in[1];
	out->len = valid_dat_len;
	memcpy((void *)out->dat, (void *)&in[APK_VALID_DATA_HEADER], valid_dat_len);

	return AW_OK;
}

/**
 * @brief this func pack data.
 *
 * @param cmd the purpose this package of data.
 * @param in the data to be sent.
 * @param in_len length of buf.
 * @param out data after package.
 */
static void aw9310x_data_pack(AW_U8 cmd, AW_U8 *in, AW_U8 in_len, AW_U8 *out, AW_U16 *out_len)
{
	AW_U64 sum = 0;

	out[0] = APK_HEADER;
	out[1] = cmd;
	out[2] = in_len;

	for (AW_U32 i = 0; i < in_len; i++) {
		sum += in[i];
		out[APK_VALID_DATA_HEADER + i] = in[i];
	}

	out[APK_VALID_DATA_HEADER + in_len] = (AW_U8)(sum & AW_ONE_BYTE_1);
	out[APK_VALID_DATA_HEADER + in_len + 1] = (AW_U8)((sum >> AW_BIT8) & AW_ONE_BYTE_1);
	out[APK_VALID_DATA_HEADER + in_len + 2] = (AW_U8)((sum >> AW_BIT16) & AW_ONE_BYTE_1);
	out[APK_VALID_DATA_HEADER + in_len + 3] = (AW_U8)((sum >> AW_BIT24) & AW_ONE_BYTE_1);
	out[APK_VALID_DATA_HEADER + in_len + 4] = (AW_U8)((sum >> AW_BIT32) & AW_ONE_BYTE_1);
	out[APK_VALID_DATA_HEADER + in_len + 5] = APK_END0;
	out[APK_VALID_DATA_HEADER + in_len + 6] = APK_END1;

	*out_len = in_len + AW9310X_REDUNDANCY_DATA_LEN;
}

/* func about factory cali */
static void aw9310x_flash_read(AW_U32 offset, AW_U8 *out, AW_U32 len)
{
	g_aw9310x_func->flash_func.flash_read(offset, out, len);
}

static void aw9310x_flash_write(AW_U32 offset, AW_U8 *in, AW_U32 len)
{
	g_aw9310x_func->flash_func.flash_write(offset, in, len);
}

/**
 * @brief this func pack and wirte approach data to flash.
 *
 * @param in data need to write.
 * @param len data len
 */
static void aw9310x_factory_cali_diff_write(AW_U8 *in, AW_U32 len)
{
	AW_U8 out[DATA_MAX_LEN] = { 0 };
	AW_U16 out_len = 0;

	aw9310x_data_pack(AW_APP_CMD_DIFF_APPROACH, in, len, out, &out_len);
	aw9310x_flash_write(0, out, out_len);
}

/**
 * @brief read the result of factory cali(diff) and
 * judge if meets the expection, if meets, set the result as threshold.
 *
 * @param mask channel used.
 * @return AW_U8 set or not
 */
static AW_U16 th_regs[] = {REG_PROXTH0_CH0, REG_PROXTH0_CH1, REG_PROXTH0_CH2, REG_PROXTH0_CH3, REG_PROXTH0_CH4, REG_PROXTH0_CH5};
static AW_U8 aw9310x_factory_cali_diff_read_set(AW_U8 mask)
{
	AW_U8 data[AW9310X_CHANNEL_MAX * sizeof(AW_S32) + AW9310X_REDUNDANCY_DATA_LEN] = {0};
	AW_S32 diff_th[AW9310X_CHANNEL_MAX];
	aw_prf_t data_valid;

	aw9310x_flash_read(0, data, sizeof(data));
	if (aw9310x_data_unpack(data, sizeof(data), &data_valid) != AW_OK) {
		AW9310X_ERR("aw9310x unpack err");
		return -AW_ERROR;
	}

	for (AW_U8 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		if ((mask >> i) & 0x01) {
			diff_th[i] = AW9310X_BYTE_TO_INT(&data_valid.dat[i * sizeof(AW_S32) + 3]);
			// set threshold only when diff kept in flash within seted range.
			if (diff_th[i] < AW9310X_DIFF_PROX_MIN || diff_th[i] > AW9310X_DIFF_PROX_MAX) {
				AW9310X_ERR("aw9310x diff_th = %d, out of rangne %d ~ %d", diff_th[i], AW9310X_DIFF_PROX_MIN, AW9310X_DIFF_PROX_MAX);
				return -AW_ERROR;
			} else {
				AW9310X_INF("aw9310x channel %d threshold is %d", i, diff_th[i]);
				aw9310x_i2c_write(th_regs[i], diff_th[i]);
			}
		}
	}

	return AW_OK;
}
/**
 * @brief user can define their own behaiver in this func when event happend.
 *
 */
static void aw9310x_event_report(enum aw9310x_event_list event)
{
	// make sure array out of bound wont happen.
	if (event > AW_APP_EVENT_LEFT_SLIDE) {
		return;
	}

	switch (event)
	{
	case AW_APP_EVENT_SINGLE_CLICK:
		AW9310X_INF("******************************** single click ********************************");
		break;
	case AW_APP_EVENT_DOUBLE_CLICK:
		AW9310X_INF("******************************** double click ********************************");
		break;
	case AW_APP_EVENT_TRIPLE_CLICK:
		AW9310X_INF("******************************* triple click ********************************");
		break;
	case AW_APP_EVENT_LONG_PRESS:
		AW9310X_INF("******************************** long press ********************************");
		break;
	case AW_APP_EVENT_SUPER_LONG_PRESS:
		AW9310X_INF("******************************** long press plus ********************************");
		break;
	case AW_APP_EVENT_WEAR:
		if (g_aw9310x.events[AW_APP_EVENT_WEAR]) {
			AW9310X_INF("******************************** wear ********************************");
		} else {
			AW9310X_INF("******************************** fall off ********************************");
		}
		break;
	case AW_APP_EVENT_RIGHT_SLIDE:
		AW9310X_INF("******************************** right slide ********************************");
		break;
	case AW_APP_EVENT_LEFT_SLIDE:
		AW9310X_INF("******************************** left slide ********************************");
		break;
	default:
		break;
	}

// send data only when bluetooth is used but sw algorithm is not.
#if defined(AW_SPP_USED) && (!defined(AW_ALGO_USED))
	aw9310x_work_set_signal(AW9310X_SPP_GET_INTN_EVENT_RUN);
#endif
}

#if defined(AW_OS_USED) && (defined(AW_ALGO_USED) || defined(AW_SPP_USED))
static void aw9310x_work_thread_create(void (*thread_cb)(void))
{
	g_aw9310x_func->work_thread_func.create_thread(thread_cb);
}

static void aw9310x_work_set_signal(AW_U32 sig)
{
	g_aw9310x_func->work_thread_func.set_signal(sig);
}

static AW_U32 aw9310x_wait_work_signal_forever(void)
{
	return  g_aw9310x_func->work_thread_func.wait_signal_forever();
}

static void aw9310x_work_thread_cb(void)
{
	AW_U32 ret = 0;

	while(1) {
		ret = 0;
		ret = aw9310x_wait_work_signal_forever();
#ifdef AW_ALGO_USED
		if (ret == AW9310X_ALGO_RUN && g_aw9310x.is_getting_curve_data == 0) {
			aw9310x_detect();
		}
#ifdef AW_SPP_USED
		if (ret == AW9310X_SPP_CURVE_RUN && g_aw9310x.is_getting_curve_data == 1) {
			aw9310x_app_send_curve_dat();
		}
#endif
#else
		if (ret == AW9310X_SPP_GET_INTN_DATA_APPROACH_RUN) {
			AW_U32 tmp = g_aw9310x.irq_status;
			aw9310x_send_intn_data(AW_APPROACH, tmp);
		} else if (ret == AW9310X_SPP_GET_INTN_DATA_FAR_AWAY_RUN) {
			AW_U32 tmp = g_aw9310x.irq_status;
			aw9310x_send_intn_data(AW_FAR_AWAY, tmp);
		}

		if (ret == AW9310X_SPP_GET_INTN_EVENT_RUN) {
			aw9310x_send_intn_event();
		}
#endif
	}
}
#endif

#ifdef AW_ALGO_USED
static AW_U8 aw9310x_read_stat(AW_U8 *th_stat)
{
	AW_U32 ret = -1;
	AW_U32 stat;

	ret = aw9310x_i2c_read(REG_STAT0, &stat);
	if (ret < 0) {
		AW9310X_ERR("read reg stat failed: %d", ret);
		return AW_FALSE;
	}else{
		AW9310X_DBG("read reg stat success! %x", stat);
	}
	th_stat[0] = stat & AW_ONE_BYTE_1;
	th_stat[1] = (stat >> AW_BIT8) & AW_ONE_BYTE_1;
	th_stat[2] = (stat >> AW_BIT16) & AW_ONE_BYTE_1;
	th_stat[3] = (stat >> AW_BIT24) & AW_ONE_BYTE_1;

	return AW_OK;
}

static void aw9310x_algo_timer_created(void (*timer_cb)(void *param))
{
	 g_aw9310x_func->algo_timer.timer_create(timer_cb);
}

static void aw9310x_algo_timer_start(AW_U32 ms)
{
	 g_aw9310x_func->algo_timer.timer_start(ms);
}

static void aw9310x_algo_timer_stop(void)
{
	 g_aw9310x_func->algo_timer.timer_stop();
}
/**
 * @brief this timer is used to start sw algo.
 *        when interrupt(approach/far away) happend, this timer will start, when event is got or timeout(10s), this timer will stop.
 *
 * @param arg
 */
static void aw9310x_algo_timer_cb(void *arg)
{
#ifdef AW_OS_USED
	aw9310x_work_set_signal(AW9310X_ALGO_RUN);
#else
	aw9310x_detect();
#endif
}

static AW_BOOL aw_init_btn_wear_algo_param(void)
{
	memset(&g_aw9310x.algo_param, 0x00, sizeof(AW_ALGO_PATAM_T));

	g_aw9310x.algo_param.ssen = 0x3f;

	g_aw9310x.algo_param.slide_en = SLIDE_EN; //Slider Enable
	g_aw9310x.algo_param.slide_shape = LINGER_SLIDER; //Use Linger Slider
	g_aw9310x.algo_param.slide_btn_en = SLIDER_REUSED; //Slider reused key
	g_aw9310x.algo_param.slide_mode = NORMAL_SLIDE; //Normal Slide event
	g_aw9310x.algo_param.slide_num = SLIDE_NUM; //The number of channals used for slider
	g_aw9310x.algo_param.slide_btn_fun = 0x1f; //Support single/double/treble click and (super)long press
	g_aw9310x.algo_param.slide_resolution = 100;
	g_aw9310x.algo_param.slide_touch_sel = 0;
	g_aw9310x.algo_param.slide_touch_deb = 0;
	g_aw9310x.algo_param.slide_touch_th = 100000;
	g_aw9310x.algo_param.norm_slide_judge_sel = 2;
	g_aw9310x.algo_param.norm_slide_th = 100;
	g_aw9310x.algo_param.norm_slide_speed_th = 20;

	g_aw9310x.algo_param.wear_fun_en = 0x18;
	g_aw9310x.algo_param.soft_wear_th_sel[3] = 1;
	g_aw9310x.algo_param.soft_wear_th_sel[4] = 1;
	g_aw9310x.algo_param.click_time_th = CLICK_TIME;
	g_aw9310x.algo_param.click_interval_th = CLICK_INTERVAL;
	g_aw9310x.algo_param.long_press_th = LONG_PRESS_TH;
	g_aw9310x.algo_param.super_long_press_th = SUPER_LONG_PRESS_TH;

	aw_set_algo_param(&g_aw9310x.algo_param);

	return AW_OK;
}
/**
 * @brief algo running in this func. this func is called by timer.
 *
 */
static void aw9310x_detect(void)
{
	AW_U8 th_stat[4];
	AW_S32 diff[8];
	AW_U32 reg;
	AW_U32 chl;
	struct aw_data_send p_data;
	struct aw_data_store p_store;

	if (g_aw9310x.g_det_cnt > AW9310X_POLLING_MAX_TIME) {
		AW9310X_INF("time out");
		goto exit_timer;
	}
	memset(diff, 0, sizeof(diff));
	memset(th_stat, 0, sizeof(th_stat));
	memset(&p_data, 0, sizeof(struct aw_data_send));
	memset(&p_store, 0, sizeof(struct aw_data_store));
	aw9310x_read_diff(diff);
	aw9310x_read_stat(th_stat);

	aw_algo_fun(diff, th_stat, &p_data, &p_store);
	if (p_data.irq_state & AW_ALGO_CLICK_PRESS_FLAG) {
		if (p_data.SingleClickST) {
			aw9310x_event_report(AW_APP_EVENT_SINGLE_CLICK);
		} else if (p_data.DoubleClickST) {
			aw9310x_event_report(AW_APP_EVENT_DOUBLE_CLICK);
		} else if (p_data.TrebleClickST) {
			aw9310x_event_report(AW_APP_EVENT_TRIPLE_CLICK);
		} else if (p_data.LongPressST) {
			aw9310x_event_report(AW_APP_EVENT_LONG_PRESS);
		} else if (p_data.SuperLongPressST) {
			aw9310x_event_report(AW_APP_EVENT_SUPER_LONG_PRESS);
		}
		if (!p_data.LongPressST) {
			goto exit_timer;
		}
	}
	if (p_data.irq_state & AW_ALGO_SLIDE_FLAG) {
		if (p_data.NormSlideST) {
			if (p_data.NormDirST) {
				aw9310x_event_report(AW_APP_EVENT_LEFT_SLIDE);
			} else {
				aw9310x_event_report(AW_APP_EVENT_RIGHT_SLIDE);
			}
		}
		goto exit_timer;
	}
	// wear judge
	aw9310x_i2c_read(REG_STAT0, &chl);
	aw9310x_wear_judge(chl);

	g_aw9310x.g_det_cnt++;
	aw9310x_algo_timer_start(AW9310X_ALGO_RUN_INTERVAL);
	return;
exit_timer:
	AW9310X_INF("exit_timer");
	g_aw9310x.g_det_cnt = 0;
	aw_algo_fun(diff, th_stat, &p_data, &p_store); // call this func to make sure algo will clear all params.
	aw9310x_i2c_read(REG_HOSTIRQSRC, &reg); //read clear to avoid interrupt after event report.
	aw9310x_i2c_write(REG_HOSTIRQEN, AW9310X_FAR_AWAY_APPORACH_ENABLE);
	aw9310x_algo_timer_stop();
}
#else


/*
static void aw9310x_click_timer_created(void (*timer_cb)(void *param))
{
	g_aw9310x_func->click_timer.timer_create(timer_cb);
}
static void aw9310x_click_timer_start(AW_U32 ms)
{
	g_aw9310x_func->click_timer.timer_start(ms);
}
static void aw9310x_click_timer_stop(void)
{
	g_aw9310x_func->click_timer.timer_stop();
}
*/
 uint8_t  key_state_last 		= 0;
 uint8_t  key_press_cont  		= 0;
 uint16_t key_release_time  	= 0;
 uint16_t key_long_press_time  	= 0;
 volatile uint8_t  out_key_num  		= 0;
 volatile uint8_t  out_key_ifint  		= 0;

/**
 * @brief this func is callback of click timer. if this func is touch off, click event will report.
 *        notice that, if click count is more than 3, event reported in this func is always triple click.
 *
 * @param param
 */
void aw9310x_click_timer_cb()
{
//	if(aw_state_change)
//	{
//		aw_state_change = 0;
//		printf("set aw_state_change 1\r\n");
//	}
	#if 1
	if(aw_state_change)
	{
		aw_state_change = 0;
		AW_U32 reg = 0;

		aw9310x_i2c_read(REG_HOSTIRQSRC, &reg);
		AW9310X_INF("aw9310x REG_HOSTIRQSRC = 0x%x.", reg);	

		//static AW_U8 
		if (reg == 0x4)
		{
			key_state_curr = 1;
		}else if ((reg == 0x2) || (reg == 0xa))
		{
			key_state_curr = 0;
		}
	}
	//key_state_curr 是已消抖的按键状态，0按下，1松开，这里不提供消抖部分逻辑
	 #endif
	
	#if 1
	int task_tim = 50;
	if(key_state_last != key_state_curr)
	{
		//当前是按下，下降沿
		if(key_state_curr == 0)
		{
			//处理按下相关
			
			key_long_press_time=0;
			key_release_time=0;			
			key_press_cont++;
		}
		//当前是松开，上升沿
		else
		{
			//处理松开相关
		}
		
		key_state_last = key_state_curr;
	} 
	
	
	//长按下检测
	if(key_state_curr == 0)
	{
		if(key_press_cont)
		{
			key_long_press_time++;
			
			//长按2秒
			if(key_long_press_time *task_tim >= 19*100)
			{
				//处理长按相关
					//printf("AT+INT,touch,longK\r\n");				
									out_key_num = 3;
						out_key_ifint = 1;
				key_long_press_time=0;
				key_press_cont=0;
					//timer_disable(TIMER6);
			}				
		}
	}
	//长松开检测
	else
	{
		if(key_press_cont)
		{
			key_release_time++;
			
			//松开500ms，处理按键值
			if(key_release_time*task_tim >= 3*100)  //当前周期是100ms  0723: 5->3
			{
				//当前是单击
				if(key_press_cont == 1)
				{
					//处理单击相关
					//printf("AT+INT,touch,singleK\r\n");
					out_key_num = 1;
						out_key_ifint = 1;
					//timer_disable(TIMER6);
				}
				//当前是双击
				else if(key_press_cont >= 2)
				{
					//处理双击相关
					//printf("AT+INT,touch,doubleK\r\n");
										out_key_num = 2;
						out_key_ifint = 1;

					//timer_disable(TIMER6);
				}					
				key_press_cont = 0;
				key_release_time = 0;
			}
		}
	}
	#endif
}
void aw9310x_click_cb(void *param)
{	
	if(aw_state_change)
	{
		aw_state_change = 0;
		AW_U32 reg = 0;

		aw9310x_i2c_read(REG_HOSTIRQSRC, &reg);
		AW9310X_INF("aw9310x REG_HOSTIRQSRC = 0x%x.", reg);	

		//static AW_U8 
		if (reg == 0x4)
		{
			key_state_curr = 1;
		}else if ((reg == 0x2) || (reg == 0xa))
		{
			key_state_curr = 0;
		}
	}
	//key_state_curr 是已消抖的按键状态，0按下，1松开，这里不提供消抖部分逻辑
	 
	int task_tim = 50;
	if(key_state_last != key_state_curr)
	{
		//当前是按下，下降沿
		if(key_state_curr == 0)
		{
			//处理按下相关
			
			key_long_press_time=0;
			key_release_time=0;			
			key_press_cont++;
		}
		//当前是松开，上升沿
		else
		{
			//处理松开相关
		}
		
		key_state_last = key_state_curr;
	} 
	
	
	//长按下检测
	if(key_state_curr == 0)
	{
		if(key_press_cont)
		{
			key_long_press_time++;
			
			//长按2秒
			if(key_long_press_time *task_tim >= 19*100)
			{
				//处理长按相关
					//printf("AT+INT,touch,longK\r\n");				
									out_key_num = 3;
						out_key_ifint = 1;
				key_long_press_time=0;
				key_press_cont=0;
					//timer_disable(TIMER6);
			}				
		}
	}
	//长松开检测
	else
	{
		if(key_press_cont)
		{
			key_release_time++;
			
			//松开500ms，处理按键值
			if(key_release_time*task_tim >= 3*100)  //当前周期是100ms  0723: 5->3
			{
				//当前是单击
				if(key_press_cont == 1)
				{
					//处理单击相关
					//printf("AT+INT,touch,singleK\r\n");
					out_key_num = 1;
						out_key_ifint = 1;
					//timer_disable(TIMER6);
				}
				//当前是双击
				else if(key_press_cont >= 2)
				{
					//处理双击相关
					//printf("AT+INT,touch,doubleK\r\n");
										out_key_num = 2;
						out_key_ifint = 1;

					//timer_disable(TIMER6);
				}					
				key_press_cont = 0;
				key_release_time = 0;
			}
		}
	}
}

/*
static void aw9310x_press_timer_created(void (*timer_cb)(void *param))
{
	 g_aw9310x_func->press_timer.timer_create(timer_cb);
}
static void aw9310x_press_timer_start(AW_U32 ms)
{
	 g_aw9310x_func->press_timer.timer_start(ms);
}
static void aw9310x_press_timer_stop(void)
{
	 g_aw9310x_func->press_timer.timer_stop();
}
*/
/**
 * @brief callback of press timer. if a pad is continuous pressed, this func will be called per second.
 *        long press event will report in 2s, super long press event will report in 5s
 *
 * @param param
 */
void aw9310x_press_timer_cb()
{
	static AW_U8 press_cnt = 0;
	if(g_aw9310x.press_event.approach_cnt == 1 && g_aw9310x.press_event.far_away_cnt == 0) {
		press_cnt++;
		if(press_cnt <= AW_SUPER_LONG_PRESS_CNT) {
			memset(&g_aw9310x.click_event, 0, sizeof(g_aw9310x.click_event));
			if (press_cnt == AW_LONG_PRESS_CNT) {
				aw9310x_event_report(AW_APP_EVENT_LONG_PRESS);
			} else if (press_cnt == AW_SUPER_LONG_PRESS_CNT) {
				aw9310x_event_report(AW_APP_EVENT_SUPER_LONG_PRESS);
			}
		} else {
			press_cnt = 0;
			memset(&g_aw9310x.click_event, 0, sizeof(g_aw9310x.click_event));
			memset(&g_aw9310x.press_event, 0, sizeof(g_aw9310x.press_event));
			return;
		}
#if USE_MULIT_TIMER
		aw9310x_press_timer_start(AW9310X_PRESS_INTERVAL);
#endif
	}
}
#endif
/**
 * @brief callback of wear timer. this func will be called if wear channel is pressed.
 *        notice that, this func will report wear event only when both wear channel is pressed.
 *        fall off event will be reported only when both channel is released.
 *
 * @param state wear or not
 */
static void aw9310x_wear_judge(AW_U32 chl)
{
	static AW_U8 wear_flag = 0;

	if (((chl >> (AW_BIT24 + AW9310X_WEAR_CHANNEL_0) & AW_BIT1) == 1) && (g_aw9310x.wear_event[0].approach_state == 0)) {
		g_aw9310x.wear_event[0].approach_state = 1;
	}
	if (((chl >> (AW_BIT24 + AW9310X_WEAR_CHANNEL_1) & AW_BIT1) == 1) && (g_aw9310x.wear_event[1].approach_state == 0)) {
		g_aw9310x.wear_event[1].approach_state = 1;
	}

	if (((chl >> (AW_BIT24 + AW9310X_WEAR_CHANNEL_0) & AW_BIT1) == 0) && (g_aw9310x.wear_event[0].approach_state == 1)) {
		g_aw9310x.wear_event[0].approach_state = 0;
	}
	if (((chl >> (AW_BIT24 + AW9310X_WEAR_CHANNEL_1) & AW_BIT1) == 0) && (g_aw9310x.wear_event[1].approach_state == 1)) {
		g_aw9310x.wear_event[1].approach_state = 0;
	}

	if (g_aw9310x.wear_event[0].approach_state == 1 && g_aw9310x.wear_event[1].approach_state == 1) {
		if(wear_flag != 1) {
			wear_flag = 1;
			g_aw9310x.events[AW_APP_EVENT_WEAR] = 1;
			aw9310x_event_report(AW_APP_EVENT_WEAR);
		}
	}

	if (g_aw9310x.wear_event[0].approach_state == 0 && g_aw9310x.wear_event[1].approach_state == 0) {
		if(wear_flag == 1) {
			wear_flag = 0;
			g_aw9310x.events[AW_APP_EVENT_WEAR] = 0;
			memset(&g_aw9310x.wear_event, 0, sizeof(g_aw9310x.wear_event));
			aw9310x_event_report(AW_APP_EVENT_WEAR);
		}
	}
}
//#define AW_SPP_USED
#ifdef AW_SPP_USED
static AW_U16 app_data_regs[] = {REG_APP_DATA_CH0, REG_APP_DATA_CH1, REG_APP_DATA_CH2, REG_APP_DATA_CH3, REG_APP_DATA_CH4, REG_APP_DATA_CH5};
static AW_U8 aw9310x_read_app_data(AW_S32 *app_data)
{
	AW_U32 ret = -1;

	for (AW_U32 i = 0; i < 6; i ++) {
		ret = aw9310x_i2c_read(app_data_regs[i], (AW_U32 *)&app_data[i]);

		if (ret < 0) {
			AW9310X_ERR("read reg app_data failed: %d", ret);
			return AW_FALSE;
		}else{
			AW9310X_DBG("aw9310x app_data data: ch%d = 0x%08x", i, app_data[i]);
		}
	}

	return AW_OK;
}
static void aw9310x_spp_init(void (*cb)(AW_U8 *dat, AW_U16 len))
{
	g_aw9310x_func->spp_func.spp_init(cb);
}

/**
 * @brief this func pack valid data and send. frame head/end, cmd, data len, sum check is added in this func.
 *
 * @param cmd the purpose this package of data.
 * @param buf the data to be sent.
 * @param length length of buf.
 */
static void aw9310x_app_data_send(AW_U8 cmd, AW_U8 *buf, AW_U32 length)
{
	AW_U8 send_total_buf[DATA_MAX_LEN] = { 0 };
	AW_U16 send_len = 0;

	aw9310x_data_pack(cmd, buf, length, send_total_buf, &send_len);
	g_aw9310x_func->spp_func.spp_write(send_total_buf, (AW_U16)(send_len));
}
/**
 * @brief this func is used to send dev(aw ic) info,
 * inlcude sensoor type(byte 0), reg data width(byte 1), reg addr
 * width(byte 2), interval between communication(byte 3) how many
 * curve packs contained in one communication(byte 4), curve data
 * width(byte 5, some ic store one data in multiple regs.)
 *
 */
static void aw9310x_app_send_dev_info(void)
{
	AW_U8 buf[6];

	buf[0] = AW_CHIP_SAR;
	buf[1] = AW_SAR_REG_LEN;
	buf[2] = AW_SAR_DATA_LEN;
	buf[3] = AW_COMM_CYCLE;
	buf[4] = PACKS_IN_ONE_COMM;
	buf[5] = AW_CURVE_DATA_LEN;
	aw9310x_app_data_send(AW_APP_CMD_GET_DEVICE_INFO, buf, sizeof(buf));
}
/**
 * @brief get register value.
 *
 * @param app_data data recived via spp.
 */
static void aw9310x_app_get_reg(aw_prf_t app_data)
{
	AW_U8 sbuf[AW_SPP_REG_MAX_NUM * SAR_REG_DATA_LEN] = { 0 };
	AW_U16 addr = 0;
	AW_U32 buf;

	memcpy(&addr, app_data.dat, sizeof(AW_U16));
	if (app_data.dat[2] > AW_SPP_REG_MAX_NUM) {
		app_data.dat[2] = AW_SPP_REG_MAX_NUM;
	}

	for (AW_U32 i = 0; i < app_data.dat[2]; i++) {
		aw9310x_i2c_read(addr + i * 4, &buf);
		AW9310X_INF(" addr = 0x%x, data = 0x%x", addr + i * 4, buf);
		sbuf[i * SAR_REG_DATA_LEN + 0] = (AW_U8)(buf);
		sbuf[i * SAR_REG_DATA_LEN + 1] = (AW_U8)(buf >> AW_BIT8);
		sbuf[i * SAR_REG_DATA_LEN + 2] = (AW_U8)(buf >> AW_BIT16);
		sbuf[i * SAR_REG_DATA_LEN + 3] = (AW_U8)(buf >> AW_BIT24);
	}
	aw9310x_app_data_send(AW_APP_CMD_READ_REG, sbuf, app_data.dat[2] * SAR_REG_DATA_LEN);
}

/**
 * @brief set register value.
 *
 * @param app_data data recived via spp.
 */
static void aw9310x_app_set_reg(aw_prf_t app_data)
{
	AW_U8 sbuf = 0;
	AW_U16 addr = 0;
	AW_U32 data = 0;

	memcpy(&addr, app_data.dat, sizeof(AW_U16));
	memcpy(&data, &app_data.dat[2], sizeof(AW_U32));
	AW9310X_INF(" addr = 0x%x, data = 0x%x", addr, data);
	aw9310x_i2c_write(addr, data);

	aw9310x_app_data_send(AW_APP_CMD_WRITE_REG, &sbuf, sizeof(AW_U8));
}

#ifdef AW_ALGO_USED
/**
 * @brief this func is used to change sw algo params which are setted by app.
 *
 * @param app_data data recived via spp.
 */
static void aw9310x_app_set_params(aw_prf_t app_data)
{
	AW_U8 sbuf = 0;

	g_aw9310x.algo_param.slide_resolution = AW9310X_BYTE_TO_INT(&app_data.dat[3]);
	g_aw9310x.algo_param.click_time_th = AW9310X_BYTE_TO_INT(&app_data.dat[7]);
	g_aw9310x.algo_param.click_interval_th = AW9310X_BYTE_TO_INT(&app_data.dat[11]);
	g_aw9310x.algo_param.long_press_th = AW9310X_BYTE_TO_INT(&app_data.dat[15]);
	g_aw9310x.algo_param.super_long_press_th = AW9310X_BYTE_TO_INT(&app_data.dat[19]);
	g_aw9310x.algo_param.slide_touch_th = AW9310X_BYTE_TO_INT(&app_data.dat[23]);
	aw_set_algo_param(&g_aw9310x.algo_param);
	aw9310x_app_data_send(AW_APP_CMD_SET_ALGO_PARA, &sbuf, sizeof(sbuf));
}
/**
 * @brief this func is used to send params of sw algo.
 *
 */
static void aw9310x_app_send_params(void)
{
	AW_U8 sbuf[24];

	// aw_get_alg_param(&g_aw9310x.algo_param);
	sbuf[0] = (AW_U8)(g_aw9310x.algo_param.slide_resolution);
	sbuf[1] = (AW_U8)(g_aw9310x.algo_param.slide_resolution >> AW_BIT8);
	sbuf[2] = (AW_U8)(g_aw9310x.algo_param.slide_resolution >> AW_BIT16);
	sbuf[3] = (AW_U8)(g_aw9310x.algo_param.slide_resolution >> AW_BIT24);

	sbuf[4] = (AW_U8)(g_aw9310x.algo_param.click_time_th);
	sbuf[5] = (AW_U8)(g_aw9310x.algo_param.click_time_th >> AW_BIT8);
	sbuf[6] = (AW_U8)(g_aw9310x.algo_param.click_time_th >> AW_BIT16);
	sbuf[7] = (AW_U8)(g_aw9310x.algo_param.click_time_th >> AW_BIT24);

	sbuf[8] = (AW_U8)(g_aw9310x.algo_param.click_interval_th);
	sbuf[9] = (AW_U8)(g_aw9310x.algo_param.click_interval_th >> AW_BIT8);
	sbuf[10] = (AW_U8)(g_aw9310x.algo_param.click_interval_th >> AW_BIT16);
	sbuf[11] = (AW_U8)(g_aw9310x.algo_param.click_interval_th >> AW_BIT24);

	sbuf[12] = (AW_U8)(g_aw9310x.algo_param.long_press_th);
	sbuf[13] = (AW_U8)(g_aw9310x.algo_param.long_press_th >> AW_BIT8);
	sbuf[14] = (AW_U8)(g_aw9310x.algo_param.long_press_th >> AW_BIT16);
	sbuf[15] = (AW_U8)(g_aw9310x.algo_param.long_press_th >> AW_BIT24);

	sbuf[16] = (AW_U8)(g_aw9310x.algo_param.super_long_press_th);
	sbuf[17] = (AW_U8)(g_aw9310x.algo_param.super_long_press_th >> AW_BIT8);
	sbuf[18] = (AW_U8)(g_aw9310x.algo_param.super_long_press_th >> AW_BIT16);
	sbuf[19] = (AW_U8)(g_aw9310x.algo_param.super_long_press_th >> AW_BIT24);

	sbuf[20] = (AW_U8)(g_aw9310x.algo_param.slide_touch_th);
	sbuf[21] = (AW_U8)(g_aw9310x.algo_param.slide_touch_th >> AW_BIT8);
	sbuf[22] = (AW_U8)(g_aw9310x.algo_param.slide_touch_th >> AW_BIT16);
	sbuf[23] = (AW_U8)(g_aw9310x.algo_param.slide_touch_th >> AW_BIT24);
	aw9310x_app_data_send(AW_APP_CMD_GET_ALGO_PARA, sbuf, sizeof(sbuf));
}
/**
 * @brief this func send data to app to draw curve.
 *
 */
static void aw9310x_app_send_curve_dat()
{
	AW_U8 th_stat[4] = {0};
	AW_U8 buf[AW_SAR_SPP_ONE_PACK_LEN * PACKS_IN_ONE_COMM];
	AW_U32 cnt = 0;
	AW_S32 spp_offset = 0;
	AW_S32 wear_addr = 0;
	AW_S32 slide_addr = 0;
	AW_S32 click_offset = 0;
	AW_S32 diff[8];
	AW_S32 app_data[8];
	struct aw_data_send p_data;
	struct aw_data_store p_store;

	memset(buf, 0, AW_SAR_SPP_ONE_PACK_LEN * PACKS_IN_ONE_COMM);
	while (AW_TRUE) {
		aw9310x_read_app_data(app_data);
		aw9310x_read_diff(diff);
		aw9310x_read_stat(th_stat);

		aw_algo_fun((AW_S32*)diff, th_stat, &p_data, &p_store);
		for(AW_U32 i = 0; i < 8; i++) {
			app_data[i] >>= 10;
			spp_offset = i * 4 + cnt * AW_SAR_SPP_ONE_PACK_LEN;
			buf[spp_offset] = (AW_U8)(app_data[i] & AW_ONE_BYTE_1);
			buf[spp_offset + 1] = (AW_U8)(app_data[i] >> AW_BIT8 & AW_ONE_BYTE_1);
			buf[spp_offset + 2] = (AW_U8)(app_data[i] >> AW_BIT16 & AW_ONE_BYTE_1);
			buf[spp_offset + 3] = (AW_U8)(app_data[i] >> AW_BIT24 & AW_ONE_BYTE_1);
		}

		for(AW_U32 i = 0; i < 8; i++) {
			spp_offset = AW_CURVE_DATA_LEN + i * 4 + cnt * AW_SAR_SPP_ONE_PACK_LEN;
			diff[i] >>= 10;
			buf[spp_offset] = (AW_U8)(diff[i] & AW_ONE_BYTE_1);
			buf[spp_offset + 1] = (AW_U8)(diff[i] >> AW_BIT8 & AW_ONE_BYTE_1);
			buf[spp_offset + 2] = (AW_U8)(diff[i] >> AW_BIT16 & AW_ONE_BYTE_1);
			buf[spp_offset + 3] = (AW_U8)(diff[i] >> AW_BIT24 & AW_ONE_BYTE_1);
		}
		click_offset = cnt * AW_SAR_SPP_ONE_PACK_LEN + SAR_APP_CURVE_CNT * AW_CURVE_DATA_LEN;
		if (p_data.irq_state & AW_BIT_12_EN) {
			buf[click_offset + AW_APP_EVENT_SINGLE_CLICK] = (p_data.SingleClickST != 0) ? 1 : 0;
			buf[click_offset + AW_APP_EVENT_DOUBLE_CLICK] = (p_data.DoubleClickST != 0) ? 1 : 0;
			buf[click_offset + AW_APP_EVENT_TRIPLE_CLICK] = (p_data.TrebleClickST != 0) ? 1 : 0;
			buf[click_offset + AW_APP_EVENT_LONG_PRESS] = (p_data.LongPressST != 0) ? 1 : 0;
			buf[click_offset + AW_APP_EVENT_SUPER_LONG_PRESS] = (p_data.SuperLongPressST != 0) ? 1 : 0;
		} else {
			buf[click_offset + AW_APP_EVENT_SINGLE_CLICK] = 0;
			buf[click_offset + AW_APP_EVENT_DOUBLE_CLICK] = 0;
			buf[click_offset + AW_APP_EVENT_TRIPLE_CLICK] = 0;
			buf[click_offset + AW_APP_EVENT_LONG_PRESS] = 0;
			buf[click_offset + AW_APP_EVENT_SUPER_LONG_PRESS] = 0;
		}
		slide_addr = cnt * AW_SAR_SPP_ONE_PACK_LEN + SAR_APP_CURVE_CNT * AW_CURVE_DATA_LEN;
		if (p_data.irq_state & AW_BIT_11_EN) {
			if (p_data.NormSlideST) {
				if (p_data.NormDirST) { //left slide
					if (p_data.NormSpeedST) { //fast
						buf[slide_addr + AW_APP_EVENT_LEFT_SLIDE] = 2;
					} else { // normal
						buf[slide_addr + AW_APP_EVENT_LEFT_SLIDE] = 1;
					}
				} else { //right slide
					if (p_data.NormSpeedST) { //fast
						buf[slide_addr + AW_APP_EVENT_RIGHT_SLIDE] = 2;
					} else { // normal
						buf[slide_addr + AW_APP_EVENT_RIGHT_SLIDE] = 1;
					}
				}
			}
		} else {
			buf[slide_addr + AW_APP_EVENT_LEFT_SLIDE] = 0;
			buf[slide_addr + AW_APP_EVENT_RIGHT_SLIDE] = 0;
		}
		wear_addr = cnt * AW_SAR_SPP_ONE_PACK_LEN + SAR_APP_CURVE_CNT * AW_CURVE_DATA_LEN;
		if (p_data.irq_state & AW_BIT_10_EN) {
			buf[wear_addr + AW_APP_EVENT_WEAR] = p_data.WearST;
		} else {
			buf[wear_addr + AW_APP_EVENT_WEAR] = 0;
		}

		cnt++;
		if(cnt >= PACKS_IN_ONE_COMM) {
			aw9310x_app_data_send(AW_APP_CMD_GET_CURVE_DATA, buf, AW_SAR_SPP_ONE_PACK_LEN * PACKS_IN_ONE_COMM);
			cnt = 0;
			break;
		}
		aw9310x_delay(AW_COMM_CYCLE / PACKS_IN_ONE_COMM);
	}
}
#else
/*********************** func about intn **********************/
/**
 * @brief this func is used for send data when interrupt(apporach/far away) happend.
 *
 * @param state apporach or far away.
 * @param irq_reg irq state reg.
 */
void aw9310x_send_intn_data(enum aw9310x_irq_stat state, AW_U32 irq_reg)
{
	AW_U8 sbuf[AW_SEND_INTN_DATA_LEN];
	AW_U8 tmp = 0;
	AW_S32 diff[8];
	AW_S32 app_data[8];

	memset((void *)sbuf, 0 , sizeof(sbuf));
	for (AW_U32 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		if ((irq_reg >> (AW_BIT24 + i)) & 0x01) {
			tmp |= (1 << i);
		}
	}
	sbuf[state - 1] = tmp;
	sbuf[2] = (AW_U8)(irq_reg & AW_ONE_BYTE_1);
	sbuf[3] = (AW_U8)(irq_reg >> AW_BIT8 & AW_ONE_BYTE_1);
	sbuf[4] = (AW_U8)(irq_reg >> AW_BIT16 & AW_ONE_BYTE_1);
	sbuf[5] = (AW_U8)(irq_reg >> AW_BIT24 & AW_ONE_BYTE_1);

	aw9310x_read_app_data(app_data);
	aw9310x_read_diff(diff);
	for(AW_U32 i = 0; i < 8; i++) {
		app_data[i] >>= 10;
		sbuf[i * 4 + 6] = (AW_U8)(app_data[i] & AW_ONE_BYTE_1);
		sbuf[i * 4 + 6 + 1] = (AW_U8)(app_data[i] >> AW_BIT8 & AW_ONE_BYTE_1);
		sbuf[i * 4 + 6 + 2] = (AW_U8)(app_data[i] >> AW_BIT16 & AW_ONE_BYTE_1);
		sbuf[i * 4 + 6 + 3] = (AW_U8)(app_data[i] >> AW_BIT24 & AW_ONE_BYTE_1);
	}

	for (AW_U32 i = 0; i < 8; i++) {
		diff[i] >>= 10;
		sbuf[i * 4 + 32 + 6] = (AW_U8)(diff[i] & AW_ONE_BYTE_1);
		sbuf[i * 4 + 32 + 6 + 1] = (AW_U8)(diff[i] >> AW_BIT8 & AW_ONE_BYTE_1);
		sbuf[i * 4 + 32 + 6 + 2] = (AW_U8)(diff[i] >> AW_BIT16 & AW_ONE_BYTE_1);
		sbuf[i * 4 + 32 + 6 + 3] = (AW_U8)(diff[i] >> AW_BIT24 & AW_ONE_BYTE_1);
	}

	aw9310x_app_data_send(AW_APP_CMD_GET_INTN_DATA, sbuf, sizeof(sbuf));
}
/**
 * @brief this func is used for event sent.
 * events(click, press and wear) send to app via this func.
 *
 */
void aw9310x_send_intn_event(void)
{
	aw9310x_app_data_send(AW_APP_CMD_GET_INTN_EVENT, g_aw9310x.events, sizeof(g_aw9310x.events));
	for (AW_U8 i = 0; i < sizeof(g_aw9310x.events); i++) {
		if (i == AW_APP_EVENT_WEAR) {
			continue; // wear state will not clear until state is changed.
		}
		g_aw9310x.events[i] = 0;
	}
}
#endif

/**
 * @brief this fun is used for app cmd handler. base on protocol v0.4
 *
 * @param dat data recived via spp.
 * @param len recived data len.
 */
void aw9310x_spp_cmd_handler(AW_U8 *dat, AW_U16 len)
{
	aw_prf_t app_data;

	if (aw9310x_data_unpack(dat, len, &app_data)) {
		AW9310X_ERR(" unpack app data error.");
		return;
	}

	switch (app_data.cmd) {
	case AW_APP_CMD_GET_DEVICE_INFO:
		AW9310X_INF(" get dev info");
		aw9310x_app_send_dev_info();
		break;
#ifdef AW_ALGO_USED
	case AW_APP_CMD_GET_CURVE_DATA:
		AW9310X_INF(" get curve info");
		if (g_aw9310x.is_getting_curve_data == 1) {
			aw9310x_work_set_signal(AW9310X_SPP_CURVE_RUN);
		}
		break;
	case AW_APP_CMD_STOP_GET_CURVE_DATA:
		AW9310X_INF(" stop get curve data");
		g_aw9310x.is_getting_curve_data = 0;
		aw9310x_i2c_write(REG_HOSTIRQEN, AW9310X_FAR_AWAY_APPORACH_ENABLE);
		break;
	case AW_APP_CMD_START_GET_CURVE_DATA:
		AW9310X_INF(" start get curve data");
		aw9310x_i2c_write(REG_HOSTIRQEN, AW9310X_FAR_AWAY_APPORACH_DISABLE);
		g_aw9310x.is_getting_curve_data = 1;
		break;
	case AW_APP_CMD_GET_ALGO_PARA:
		AW9310X_INF(" get param info");
		aw9310x_app_send_params();
		break;
	case AW_APP_CMD_SET_ALGO_PARA:
		AW9310X_INF(" set param info");
		aw9310x_app_set_params(app_data);
		break;
#endif
	case AW_APP_CMD_READ_REG:
		AW9310X_INF(" get reg");
		aw9310x_app_get_reg(app_data);
		break;
	case AW_APP_CMD_WRITE_REG:
		AW9310X_INF("set reg");
		aw9310x_app_set_reg(app_data);
		break;
	case AW_APP_CMD_OFFSET_CALI:
		AW9310X_INF("offset cali");
		aw9310x_offset_cali();
		break;
	case AW_APP_CMD_DIFF_TO_AIR:
		AW9310X_INF("diff to air");
		aw9310x_diff_to_air();
		break;
	case AW_APP_CMD_DIFF_APPROACH:
		AW9310X_INF("diff approach");
		aw9310x_diff_approach();
		break;
	case AW_APP_CMD_SHORT_CIRCUIT:
		AW9310X_INF("short circuit");
		aw9310x_short_circuit_detect();
		break;
	case AW_APP_CMD_SET_DIFF_TH:
		AW9310X_INF("set th");
		aw9310x_factory_cali_diff_write(app_data.dat, app_data.len);
		break;
	default:
		AW9310X_ERR("no such commnd(%d).", app_data.cmd);
		break;
	}
}
#endif

/**
 * @brief send diff-prox data.
 *
 * @param diff_avg diff avg in prox state
 * @param avg_state diff avg in prox state
 * @param snr_state
 */
static void aw9310x_diff_approach_send(AW_S32 *diff_avg, AW_U8 avg_state, AW_U8 snr_state)
{
#ifdef AW_SPP_USED
	AW_U8 sbuf[sizeof(AW_S32) * AW9310X_CHANNEL_MAX + 2] = { 0 };

	for (AW_U8 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		sbuf[i * sizeof(AW_S32) + 0] = (uint8_t)diff_avg[i];
		sbuf[i * sizeof(AW_S32) + 1] = (uint8_t)(diff_avg[i] >> AW_BIT8);
		sbuf[i * sizeof(AW_S32) + 2] = (uint8_t)(diff_avg[i] >> AW_BIT16);
		sbuf[i * sizeof(AW_S32) + 3] = (uint8_t)(diff_avg[i] >> AW_BIT24);
	}
	sbuf[sizeof(AW_S32) * AW9310X_CHANNEL_MAX] = avg_state;
	sbuf[sizeof(AW_S32) * AW9310X_CHANNEL_MAX + 1] = snr_state;
	aw9310x_app_data_send(AW_APP_CMD_DIFF_APPROACH, sbuf, sizeof(sbuf));
#endif
}
/**
 * @brief this func sample diff data for 1s with press and calculate average value.
 *
 */
void aw9310x_diff_approach(void)
{
	AW_U8 diff_avg_save[AW9310X_CHANNEL_MAX * sizeof(AW_S32)];
	AW_U32 sample_cnt = 100;
	AW_S32 diff[AW9310X_CHANNEL_MAX];
	AW_S32 diff_avg[AW9310X_CHANNEL_MAX];
	AW_S64 diff_sum[AW9310X_CHANNEL_MAX];

	memset(diff_avg_save, 0, sizeof(diff_avg_save));
	memset(diff_avg, 0, sizeof(diff_avg));
	memset(diff_sum, 0, sizeof(diff_sum));
	memset(diff, 0, sizeof(diff));

	while (1) {
		sample_cnt--;
		if (sample_cnt <= 0) {
			sample_cnt = 100;
			break;
		}
		aw9310x_read_diff(diff);
		for (int i = 0; i < AW9310X_CHANNEL_MAX; i++) {
			diff[i] >>= 10;
			diff_sum[i] += diff[i];
		}
		aw9310x_delay(10);
	}

	for (AW_U8 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		diff_avg[i] = diff_sum[i] / sample_cnt;
		AW9310X_INF("aw9310x diff_avg[%d] = 0x%x", i, diff_avg[i]);
		if (diff_avg[i] < AW9310X_DIFF_PROX_MIN || diff_avg[i] > AW9310X_DIFF_PROX_MAX) {
			AW9310X_ERR("aw9310x diff_avg[%d] = 0x%x", i, diff_avg[i]);
		}
		diff_avg_save[i * 4] = (AW_U8)(diff_avg[i] & AW_ONE_BYTE_1);
		diff_avg_save[i * 4 + 1] = (AW_U8)(diff_avg[i] >> AW_BIT8 & AW_ONE_BYTE_1);
		diff_avg_save[i * 4 + 2] = (AW_U8)(diff_avg[i] >> AW_BIT16 & AW_ONE_BYTE_1);
		diff_avg_save[i * 4 + 3] = (AW_U8)(diff_avg[i] >> AW_BIT24 & AW_ONE_BYTE_1);
	}
//	aw9310x_factory_cali_diff_write(diff_avg_save, sizeof(diff_avg_save));
	aw9310x_diff_approach_send(diff_avg, 0, 0);
}

static void aw9310x_diff_to_air_send(AW_S32 *diff_max, AW_S32 *diff_min)
{
#ifdef AW_SPP_USED
	AW_U8 sbuf[2 * sizeof(AW_S32) * AW9310X_CHANNEL_MAX] = { 0 };

	for (AW_U8 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		sbuf[i * sizeof(AW_S32) + 0] = (uint8_t)diff_max[i];
		sbuf[i * sizeof(AW_S32) + 1] = (uint8_t)(diff_max[i] >> AW_BIT8);
		sbuf[i * sizeof(AW_S32) + 2] = (uint8_t)(diff_max[i] >> AW_BIT16);
		sbuf[i * sizeof(AW_S32) + 3] = (uint8_t)(diff_max[i] >> AW_BIT24);
	}
	for (AW_U8 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		sbuf[sizeof(AW_S32) * AW9310X_CHANNEL_MAX + i * sizeof(AW_S32) + 0] = (uint8_t)diff_min[i];
		sbuf[sizeof(AW_S32) * AW9310X_CHANNEL_MAX + i * sizeof(AW_S32) + 1] = (uint8_t)(diff_min[i] >> AW_BIT8);
		sbuf[sizeof(AW_S32) * AW9310X_CHANNEL_MAX + i * sizeof(AW_S32) + 2] = (uint8_t)(diff_min[i] >> AW_BIT16);
		sbuf[sizeof(AW_S32) * AW9310X_CHANNEL_MAX + i * sizeof(AW_S32) + 3] = (uint8_t)(diff_min[i] >> AW_BIT24);
	}
	aw9310x_app_data_send(AW_APP_CMD_DIFF_TO_AIR, sbuf, sizeof(sbuf));
#endif
}
/**
 * @brief this func sample diff data for 5s without press and peak to peak value.
 *
 */
void aw9310x_diff_to_air(void)
{
	AW_U32 sample_cnt = 500;
	AW_S32 diff[AW9310X_CHANNEL_MAX];
	AW_S32 diff_max[AW9310X_CHANNEL_MAX];
	AW_S32 diff_min[AW9310X_CHANNEL_MAX];
	AW_S32 diff_to_air_vpp[AW9310X_CHANNEL_MAX];
	AW_S64 diff_sum[AW9310X_CHANNEL_MAX];

	memset(diff, 0, sizeof(diff));
	memset(diff_max, 0, sizeof(diff_max));
	memset(diff_min, 0, sizeof(diff_min));
	memset(diff_sum, 0, sizeof(diff_sum));
	memset(diff_to_air_vpp, 0, sizeof(diff_to_air_vpp));

	aw9310x_read_diff(diff);
	for (int i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		diff[i] >>= 10;
		diff_max[i] = diff[i];
		diff_min[i] = diff[i];
	}

	while (1) {
		sample_cnt--;
		if (sample_cnt <= 0) {
			break;
		}
		aw9310x_read_diff(diff);
		for (int i = 0; i < AW9310X_CHANNEL_MAX; i++) {
			diff[i] >>= 10;
			diff_sum[i] += diff[i];
			if (diff_max[i] < diff[i]) {
				diff_max[i] = diff[i];
			}
			if (diff_min[i] > diff[i]) {
				diff_min[i] = diff[i];
			}
		}
		aw9310x_delay(10);
	}
	for (int i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		diff_to_air_vpp[i] = diff_max[i] - diff_min[i];
		AW9310X_INF("aw9310x diff_to_air_vpp[%d] = 0x%x", i, diff_to_air_vpp[i]);
		AW9310X_INF("aw9310x diff_max[%d] = 0x%x", i, diff_max[i]);
		AW9310X_INF("aw9310x diff_min[%d] = 0x%x", i, diff_min[i]);
	}
	aw9310x_diff_to_air_send(diff_max, diff_min);
	AW9310X_INF("leave");
}

void aw9310x_offset_cali_send(AW_U32 *offset, AW_U32 *init, AW_U8 offset_state, AW_U8 init_state)
{
#ifdef AW_SPP_USED
	AW_U8 sbuf[2 * sizeof(AW_S32) * AW9310X_CHANNEL_MAX + 2] = { 0 };

	for (AW_U8 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		sbuf[i * sizeof(AW_S32) + 0] = (uint8_t)offset[i];
		sbuf[i * sizeof(AW_S32) + 1] = (uint8_t)(offset[i] >> AW_BIT8);
		sbuf[i * sizeof(AW_S32) + 2] = (uint8_t)(offset[i] >> AW_BIT16);
		sbuf[i * sizeof(AW_S32) + 3] = (uint8_t)(offset[i] >> AW_BIT24);
	}
	for (AW_U8 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		sbuf[sizeof(AW_S32) * AW9310X_CHANNEL_MAX + i * sizeof(AW_S32) + 0] = (uint8_t)init[i];
		sbuf[sizeof(AW_S32) * AW9310X_CHANNEL_MAX + i * sizeof(AW_S32) + 1] = (uint8_t)(init[i] >> AW_BIT8);
		sbuf[sizeof(AW_S32) * AW9310X_CHANNEL_MAX + i * sizeof(AW_S32) + 2] = (uint8_t)(init[i] >> AW_BIT16);
		sbuf[sizeof(AW_S32) * AW9310X_CHANNEL_MAX + i * sizeof(AW_S32) + 3] = (uint8_t)(init[i] >> AW_BIT24);
	}
	sbuf[2 * sizeof(AW_S32) * AW9310X_CHANNEL_MAX] = offset_state;
	sbuf[2 * sizeof(AW_S32) * AW9310X_CHANNEL_MAX + 1] = init_state;
	aw9310x_app_data_send(AW_APP_CMD_OFFSET_CALI, sbuf, sizeof(sbuf));
#endif
}
/**
 * @brief this func is used for parasitic capacitance calibration.
 *
 */
void aw9310x_offset_cali(void)
{
	AW_U8 offset_state = 0;
	AW_U8 init_state = 0;
	AW_U32 cnt = 0;
	AW_U32 reg = 0;
	AW_U32 crange[AW9310X_CHANNEL_MAX];
	AW_U32 offset[AW9310X_CHANNEL_MAX];
	AW_U32 initial_data_after_cali[AW9310X_CHANNEL_MAX];
	AW_U32 reg_afe_cfg1[] = {REG_AFECFG1_CH0, REG_AFECFG1_CH1, REG_AFECFG1_CH2, REG_AFECFG1_CH3, REG_AFECFG1_CH4, REG_AFECFG1_CH5};
	AW_U32 reg_afe_cfg0[] = {REG_AFECFG0_CH0, REG_AFECFG0_CH1, REG_AFECFG0_CH2, REG_AFECFG0_CH3, REG_AFECFG0_CH4, REG_AFECFG0_CH5};
	float cap[AW9310X_CHANNEL_MAX];
	float aw9310x_cap_map[16][2] = { {0, 1.1}, {1, 2.2}, {2, 3.3}, {3, 4.4},
									{4, 6.6}, {5, 7.7}, {6, 8.8}, {7, 9.9},
									{8, 11}, {9, 12.1}, {10, 13.2}, {11, 14.3},
									{12, 16.5}, {13, 17.6}, {14, 18.7}, {15, 19.8} };

	//step 1: Parasitic capacitance calibration
	aw9310x_i2c_write(REG_SCANCTRL0, 0x00001f1f);

	//step 2: Waiting 1s for calibration complete flag
	while(1) {
		if (cnt >= 100) {
			AW9310X_ERR("aw9310x get calibration complete flag error");
			break;
		}
		aw9310x_i2c_read(REG_HOSTIRQSRC, &reg);
		if(((reg >> AW_BIT3) & 1) == 1) {
			break;
		}
		cnt++;
		aw9310x_delay(10);
	}

	//step 3: Read offset data and restore
	for (AW_U32 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		aw9310x_i2c_read(reg_afe_cfg1[i], &offset[i]);
		offset[i] >>= AW_BIT16;
		offset[i] &= 0x0000ffff;
		AW9310X_INF("aw9310x reg = 0x%x  data = 0x%x", reg_afe_cfg1[i], offset[i]);
	}

	//step 4: Read the initial data after calibration and restore
	for (AW_U32 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		aw9310x_i2c_read(REG_PSCBD_CH0 + i * 4, &initial_data_after_cali[i]);
		AW9310X_INF("aw9310x reg = 0x%x, initial_data_after_cali[%d] = 0x%x", reg_afe_cfg0[i], i, initial_data_after_cali[i]);
	}

	//step 5: Judge whether the offset meets the expectation
	for (AW_U32 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		if ((offset[i] > AW9310X_OFFSET_MAX) || (offset[i] < AW9310X_OFFSET_MIN)) {
			offset_state &= ~(1 << i);
			AW9310X_ERR("offset doesn't meets the expectation, offset[%d] = 0x%x", i, offset[i]);
		}else{
			offset_state |= 1 << i;
			AW9310X_INF("aw9310x offset[%d] = 0x%x", i, offset[i]);
		}
	}

	//step 6: Judge whether the initial data meets the expectation
	for (AW_U32 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		aw9310x_i2c_read(reg_afe_cfg0[i], &crange[i]);
		crange[i] >>= AW_BIT12;
		crange[i] &= 0x0000000f;
		AW9310X_INF("aw9310x crange[%d] = 0x%2x", i, crange[i]);
		for (AW_U32 j = 0; j < 16; j++) {
			if (crange[i] == aw9310x_cap_map[j][0]) {
				cap[i] = aw9310x_cap_map[j][1];
				AW9310X_INF("aw9310x cap[%d] = %d / 1000", i, (AW_U32)(cap[i] * 1000));
				break;
			}
		}
	}
	for (AW_U32 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
		AW9310X_INF("aw9310x initial_data_after_cali[%d] = 0x%x", i, (AW_U32)(initial_data_after_cali[i]));
		float result = (float)(initial_data_after_cali[i] >> 21) * cap[i];
		if ((result < AW9310X_CAP_MIN) || (result > AW9310X_CAP_MAX)) {
			init_state &= ~(1 << i);
			AW9310X_ERR("aw9310x initial data doesn't meet the expection, result[%d] = (%d / 1000)", i, (AW_U32)(result * 1000));
		}else{
			init_state |= 1 << i;
			AW9310X_INF("aw9310x result[%d] = (%d / 1000)", i, (AW_U32)(result * 1000));
		}
	}
	aw9310x_offset_cali_send(offset, initial_data_after_cali, offset_state, init_state);
}

static void aw9310x_short_circuit_state_send(AW_U8 gnd, AW_U8 vcc)
{
#ifdef AW_SPP_USED
	AW_U8 sbuf[2] = {0};

	sbuf[0] = gnd;
	sbuf[1] = vcc;
	aw9310x_app_data_send(AW_APP_CMD_SHORT_CIRCUIT, sbuf, sizeof(sbuf));
#endif
}
/**
 * @brief this func detect if any channel connected to vdd or gnd.
 *
 */
void aw9310x_short_circuit_detect(void)
{
	AW_U8 gnd_stat = 0;
	AW_U8 vcc_stat = 0;
	AW_U32 stat = 0;

	AW9310X_INF("ennter");
	aw9310x_i2c_write(REG_CMD, AW9310X_SLEEP);
	aw9310x_i2c_write(REG_PSR0, 0x0055);
	aw9310x_i2c_write(REG_PSR1, 0x0001);
	aw9310x_i2c_write(REG_DICR, 0);
	aw9310x_i2c_write(REG_IEB, AW9310X_GPIO_DIRIN_EN);

	aw9310x_i2c_write(REG_PUUC, AW9310X_GPIO_PUU_EN);
	aw9310x_i2c_write(REG_PUDN, AW9310X_GPIO_PUD_DIS);
	aw9310x_i2c_read(REG_GPIODATA, &stat);
	gnd_stat = stat;
	if (stat != 0x001f) {
		for (AW_U32 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
			if (((stat >> i) & 0x01) == 0) {
				AW9310X_ERR("aw9310x channel %d connect the ground.", i);
			}
		}
	}

	aw9310x_i2c_write(REG_PUUC, AW9310X_GPIO_PUU_DIS);
	aw9310x_i2c_write(REG_PUDN, AW9310X_GPIO_PUD_EN);
	aw9310x_i2c_read(REG_GPIODATA, &stat);
	vcc_stat = stat;
	if (stat != 0) {
		for (AW_U32 i = 0; i < AW9310X_CHANNEL_MAX; i++) {
			if (((stat >> i) & 0x01) == 1) {
				AW9310X_ERR("aw9310x channel %d connect the power supply.", i);
			}
		}
	}

	// reset reg to pre-state.
	aw9310x_i2c_write(REG_PUUC, 0x0);
	aw9310x_i2c_write(REG_PUDN, 0x0);
	aw9310x_i2c_write(REG_PSR0, 0x0);
	aw9310x_i2c_write(REG_PSR1, 0x0);
	// aw9310x_i2c_write(REG_IEB,  0x40);
	aw9310x_i2c_write(REG_CMD,  0x1);

	aw9310x_short_circuit_state_send(gnd_stat, vcc_stat);
}

static AW_U32 aw9310x_init_irq_handle(void)
{
	AW_U8 cnt = 20;
	AW_U32 reg_data;

	AW9310X_INF("enter");

	while (cnt--) {
		aw9310x_i2c_read(REG_HOSTIRQSRC, &reg_data);
		if ((reg_data & 0x01) == 1) {
			AW9310X_INF("cnt = %d", cnt);
			return AW_OK;
		}
	}
	AW9310X_ERR("trouble encounter!");

	return -AW_ERROR;
}


struct aw9310x_func aw9310x_demo_func = {
	.i2c_func = {
		.i2c_r = aw9310x_i2c_read,
		.i2c_w = aw9310x_i2c_write,
	},
#ifdef AW_OS_USED
	.irq_thread_func = {
		.create_thread       = aw9310x_create_irq_thread,
		.set_signal          = aw9310x_irq_set_signal,
		.wait_signal_forever = aw9310x_wait_irq_signal_forever
	},
#endif
	.irq_init = aw9310x_irq_init,
	.delay    = aw9310x_delay,
#if defined(AW_OS_USED) && (defined(AW_ALGO_USED) || defined(AW_SPP_USED))
	.work_thread_func = {
		.create_thread       = aw9310x_create_work_thread,
		.set_signal          = aw9310x_work_set_signal,
		.wait_signal_forever = aw9310x_wait_work_signal_forever,
	},
#endif
#ifdef AW_ALGO_USED
	.algo_timer = {
		.timer_create = aw9310x_algo_timer_create,
		.timer_start  = aw9310x_algo_timer_start,
		.timer_stop   = aw9310x_algo_timer_stop,
	},
#else
#if USE_MULIT_TIMER
	.click_timer = {
		.timer_create = aw9310x_click_timer_create,
		.timer_start  = aw9310x_click_timer_start,
		.timer_stop   = aw9310x_click_timer_stop,
	},
	.press_timer = {
		.timer_create = aw9310x_press_timer_create,
		.timer_start  = aw9310x_press_timer_start,
		.timer_stop   = aw9310x_press_timer_stop,
	},
#endif	
#endif
#ifdef AW_SPP_USED
	.spp_func = {
		.spp_init  = aw9310x_spp_create,
		.spp_write = aw9310x_spp_write,
	},
#endif
	.flash_func = {
		.flash_read = aw9310x_flash_read,
		.flash_write = aw9310x_flash_write,
	},
};

/**
 * @brief driver init.
 *
 * @param hw_func pointer of func related to platform, include i2c, timer, thread etc.
 * @return AW_U32 driver init success if return 0.
 */
AW_U32 aw9310x_init(struct aw9310x_func *hw_func)
{
	AW_U32 ret = -AW_ERROR;

	if (hw_func->i2c_func.i2c_r == NULL || hw_func->i2c_func.i2c_w == NULL ||
		hw_func->irq_init == NULL || hw_func->delay == NULL ||
		hw_func->flash_func.flash_read == NULL || hw_func->flash_func.flash_write == NULL) {
		AW9310X_ERR("some basic func pointer are NULL %p %p %p %p %p %p", hw_func->i2c_func.i2c_r, hw_func->i2c_func.i2c_w, hw_func->irq_init, hw_func->delay,
		hw_func->flash_func.flash_read, hw_func->flash_func.flash_write);
		return -AW_ERROR;
	}

#ifdef AW_OS_USED
	if (hw_func->irq_thread_func.create_thread == NULL || hw_func->irq_thread_func.set_signal == NULL ||
		hw_func->irq_thread_func.wait_signal_forever == NULL) {
		AW9310X_ERR("some irq_thread_func pointer are NULL");
		return -AW_ERROR;
	}
#endif

#ifdef AW_ALGO_USED
#ifdef AW_OS_USED
	if (hw_func->work_thread_func.create_thread == NULL || hw_func->work_thread_func.set_signal == NULL ||
		hw_func->work_thread_func.wait_signal_forever == NULL) {
		AW9310X_ERR("some work thread func pointer are NULL");
		return -AW_ERROR;
	}
#endif
	if (hw_func->algo_timer.timer_create == NULL || hw_func->algo_timer.timer_start == NULL ||
		hw_func->algo_timer.timer_stop== NULL) {
		AW9310X_ERR("some algorithm timer func pointer are NULL");
		return -AW_ERROR;
	}
#else
#if USE_MULIT_TIMER
	if (hw_func->click_timer.timer_create == NULL || hw_func->click_timer.timer_start == NULL ||
		hw_func->click_timer.timer_stop == NULL || hw_func->press_timer.timer_create == NULL ||
		hw_func->press_timer.timer_start == NULL || hw_func->press_timer.timer_stop== NULL) {
		return -AW_ERROR;
	}
#endif		
#endif

#ifdef AW_SPP_USED
	if (hw_func->spp_func.spp_init == NULL || hw_func->spp_func.spp_write == NULL) {
		AW9310X_ERR("spp func pointer are NULL");
		return -AW_ERROR;
	}
#endif

	memset(&g_aw9310x, 0, sizeof(g_aw9310x));

	g_aw9310x_func = hw_func;

	ret = aw9310x_read_chipid();
	if (ret != AW_OK) {
		AW9310X_ERR("aw9310x read_chipid failed :%d", ret);
		return -AW_ERROR;
	}

	ret = aw9310x_sw_reset();
	if (ret != AW_OK) {
		AW9310X_ERR("aw9310x sw reset failed :%d", ret);
		return -AW_ERROR;
	}

	ret = aw9310x_init_irq_handle();
	if (ret != AW_OK) {
		AW9310X_ERR("aw9310x_init_irq_handle failed :%d", ret);
		return -AW_ERROR;
	}

	if (ret != AW_OK) {
		AW9310X_ERR("version_init failed :%d", ret);
		return -AW_ERROR;
	}

	aw9310x_irq_init(aw9310x_irq_cb);

#ifdef AW_OS_USED
	ret = aw_get_algo_ver();
	AW9310X_INF("algo ver = 0x%x", ret);
	// as some platform do not support operate i2c in intn or timer,
	// so this thread is necessary. timer's or intn's callback will send
	// signal to this thread.  opreations about i2c will be done in this thread.
	aw9310x_irq_thread_create(aw9310x_irq_thread_cb);
#endif

	ret = aw9310x_param_load();
	if (ret < 0) {
		AW9310X_ERR("aw9310x failed to load params.");
		return -AW_ERROR;
	}

	g_aw9310x.old_mode = AW9310X_SLEEP_MODE;

#ifdef AW_ALGO_USED
	ret = aw_init_btn_wear_algo_param();
	if (ret) {
		AW9310X_ERR("aw9310x failed to initalize algorithm params.");
		return -AW_ERROR;
	}
#endif

#if defined(AW_OS_USED) && (defined(AW_ALGO_USED) || defined(AW_SPP_USED))
	/* timer will set signal to this thread to operate i2c. */
	aw9310x_work_thread_create(aw9310x_work_thread_cb);
#endif

#ifdef AW_ALGO_USED // get event base on polling(algorithm)
	aw9310x_algo_timer_created(aw9310x_algo_timer_cb);
#else
	/*
	 * this timer report click event.
	 * it also controls interval time of continous click, namely the max time between continous click.
	*/
//	aw9310x_click_timer_create(aw9310x_click_timer_cb);    //0713  rjq --
//	aw_timer_nvic_config();
#if USE_MULIT_TIMER
	aw9310x_click_timer_create(aw9310x_click_timer_cb);
	/* this timer report press event. it will report event per second. */
	aw9310x_press_timer_create(aw9310x_press_timer_cb);
	aw_timer_nvic_config();
#endif	
#endif

		AW9310X_INF("diff approach");
		//aw9310x_diff_approach();

#ifdef AW_SPP_USED
	aw9310x_spp_init(aw9310x_spp_cmd_handler);
#endif

	//if (aw9310x_factory_cali_diff_read_set(0x1f)) {
	//	AW9310X_INF("aw9310x didn't set data in flash as threshold.");
	//}

	aw9310x_operation_mode_set(AW9310X_ACTIVE_MODE);
	AW9310X_INF("aw9310x init finish.");
	

	return AW_OK;
}

uint8_t is_check = 0;
uint8_t l81_AT_AW_START_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	is_check = 1;
	
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,end\r\n");
	return 1U;
}
uint8_t l81_AT_AW_STOP_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	is_check = 0;
	
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,end\r\n");
	return 1U;
}

#ifdef AW_factory_USED
AW_S32 diff[AW9310X_CHANNEL_MAX];
#endif

/**
 * @brief we always need interrupt in both mode(with and without algorithm).
 *        when driver run with algorithm, it sends signal to thread to start algorithm.
 *        when driver run without algorithm, we get event base on interrupt.
 *        Notice!!!  Wear off Event is obtained base on interrupt in both mode.
 */
static void aw9310x_get_irq_stat(void)
{
//  printf("aw9310x_get_irq_stat \r\n");  //0714 rjq+
//	if (out_key_ifint)
//	{
//		printf(" \r\n");  //0714 rjq+
////		printf("aw9310x_get_irq_stat return because out_key_ifint!=0\r\n");  //0714 rjq+
////		exti_interrupt_flag_clear(EXTI_12);
//		return;
//	}else{
//		aw_state_change = 1;
//	}  //0809 rjq-
		aw_state_change = 1;  //0809 rjq+

//#ifdef AW_factory_USED  //0809 rjq-
//	memset(diff, 0, sizeof(diff));
//	aw9310x_read_diff(diff);
//#endif
//	AW_U32 reg = 0;  //0809 rjq-
//	AW_U32 chl = 0;
//	aw9310x_i2c_read(REG_HOSTIRQSRC, &reg);
//	AW9310X_INF("aw9310x REG_HOSTIRQSRC = 0x%x.", reg);	
//  //printf("aw9310x REG_HOSTIRQSRC = 0x%x.\r\n", reg);  //0714 rjq+
//	//static AW_U8 
//	if (reg == 0x4)
//	{
//		key_state_curr = 1;
//	}else if ((reg == 0x2) || (reg == 0xa))
//	{
//		key_state_curr = 0;
//	}

//	if (((reg >> AW_BIT1) & AW_BIT_0_1_EN) == AW_APPROACH) {  //0809 rjq-
//#ifdef AW_ALGO_USED // get event base on polling(algorithm)
//		if (g_aw9310x.g_det_cnt == 0 && g_aw9310x.is_getting_curve_data == 0) {
//			aw9310x_i2c_write(REG_HOSTIRQEN, AW9310X_FAR_AWAY_APPORACH_DISABLE);
//			aw9310x_algo_timer_stop();
//			aw9310x_algo_timer_start(AW9310X_ALGO_RUN_INTERVAL);
//		}
//#else // get event base on interrupt
//		aw9310x_i2c_read(REG_STAT0, &chl);
//		g_aw9310x.irq_status = chl;
//		aw9310x_wear_judge(chl);
//		AW9310X_INF("aw9310x approach 0x0090 = 0x%x.", chl);
///*		if (chl & 0x4000000)
//			printf("AT+INT,touch,K1\r\n");
//		if (chl & 0x2000000)
//			printf("AT+INT,touch,K2\r\n");
//		if (chl & 0x1000000)
//			printf("AT+INT,touch,K3\r\n");*/
//		if(((chl >> (AW_BIT24 + AW9310X_CLICK_CHANNEL) & AW_BIT1) == 1) && g_aw9310x.click_event.approach_state == 0) {
//			g_aw9310x.click_event.approach_state = 1;
//			g_aw9310x.click_event.approach_cnt++;
////			g_aw9310x.press_event.approach_state = 1;  //0714 rjq-
////			g_aw9310x.press_event.approach_cnt++;  //0714 rjq-
////						timer_enable(TIMER6);

//#if USE_MULIT_TIMER			  //0809 rjq-
//			aw9310x_press_timer_stop();
//			aw9310x_press_timer_start(AW9310X_PRESS_INTERVAL);
//#endif
//		}
//#ifdef AW_SPP_USED
//		aw9310x_work_set_signal(AW9310X_SPP_GET_INTN_DATA_APPROACH_RUN);
//#endif
//#endif
//	}
//	if (((reg >> AW_BIT1) & AW_BIT_0_1_EN) == AW_FAR_AWAY) {  //0809 rjq-
//		aw9310x_i2c_read(REG_STAT0, &chl);
//		g_aw9310x.irq_status = chl;
//		aw9310x_wear_judge(chl);
//#ifndef AW_ALGO_USED // we also need pay attention to far away to get event when algorithm don't run.
//		AW9310X_INF("aw9310x far away 0x0090 = 0x%x.", chl);
//		if((chl >> (AW_BIT24 + AW9310X_CLICK_CHANNEL) & AW_BIT1) == 0 && g_aw9310x.click_event.approach_state == 1) {
//			g_aw9310x.click_event.approach_state = 0;
//			g_aw9310x.click_event.far_away_cnt++;
////			g_aw9310x.press_event.approach_state = 0;  //0714 rjq-
////			g_aw9310x.press_event.far_away_cnt++;  //0714 rjq-
//#if USE_MULIT_TIMER			
//			aw9310x_click_timer_stop();
//			aw9310x_click_timer_start(AW9310X_CLICK_INTERVAL);
//#endif
//		}
//#ifdef AW_SPP_USED
//		aw9310x_work_set_signal(AW9310X_SPP_GET_INTN_DATA_FAR_AWAY_RUN);
//#endif
//#endif
//	}
}

/**
 * @brief callback of irq, don't do any thing here apart from set signal if OS used.
 *
 */
void aw9310x_irq_cb()
{
#ifdef AW_OS_USED
	aw9310x_irq_set_signal(AW9310X_READ_IRQ_STAT);
#else//  
	aw9310x_get_irq_stat();
	
#endif
}
void KeyNumoutToSOC(void *param){

	//AT+INT,touch
	if (out_key_num){
		printf("AT+INT,touch,%d\r\n",out_key_num);
		out_key_num = 0;
		out_key_ifint = 0;
	}
	// output AW num to factorytest
	if(is_check)
	{
		//AW9310X_INF("is_check,%d\r\n",is_check);
		AW_read_task();
	}
	
	//return 0;
}
uint8_t l81_AT_AW_R_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

	ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 0U){
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,This command doesn't hava params\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}

	AW_U8 cnt = 0;
	AW_S32 ret = -1;
	AW_U32 data = 0;

	
	AW_S32 diff_num[AW9310X_CHANNEL_MAX] = {};
	
	for (AW_U32 i = 0; i < AW9310X_CHANNEL_MAX; i ++) {
		ret = aw9310x_i2c_read(diff_regs[i], (AW_U32 *)&diff_num[i]);// + (AW_U16)(i*4)

		if (ret < 0) {
			printf("read reg diff failed: %d\r\n", ret);
			return AW_FALSE;
		}else{
			AW9310X_INF( "aw9310x diff data: ch%d = %d\r\n", i, diff_num[i] >> 10);
		}
	}
	
	AW_U32 reg = 0;

	aw9310x_i2c_read(REG_HOSTIRQSRC, &reg);   //from line 1812
	printf("AT+RES,ACK\r\n");
	printf("AT+RES,0x%x\r\n", reg);
	printf("AT+RES,end\r\n");
	return 1U;		
}
//int num_i = 0;
AW_S32 diff_max[AW9310X_CHANNEL_MAX] = {0};
void AW_read_task()
{
#ifdef AW_factory_USED  //0815 rjq+
	memset(diff, 0, sizeof(diff));
	aw9310x_read_diff(diff);
#endif
	
	for (AW_U32 i = 0; i < 3; i ++) 
		if( diff[i] > diff_max[i])
			diff_max[i] = diff[i];	
//	if(num_i++ < 20000) return;
//	num_i = 0;
	AW_U32 print_num = 80000;
	if(diff_max[0] >> 10 < print_num || diff_max[1] >> 10  < print_num || diff_max[2] >> 10  < print_num )
		return;
	
	printf( "AT+INT,AW,%d,%d,%d\r\n",diff_max[0] >> 10,diff_max[1] >> 10,diff_max[2] >> 10);
	for (AW_U32 i = 0; i < 3; i ++) 
		diff_max[i] = 0;
}

void AW_check_task(void *param)
{
	aw9310x_click_timer_cb();
	KeyNumoutToSOC((void *)1);
}