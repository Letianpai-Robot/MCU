/*!
    \file    L81_Motors.c
    \brief   Motor1/2/3/4/5/6 drivers, supply APIs for applications
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32l23x.h"
#include "systick.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "L81_Motors.h"
#include "L81_inf.h"
#include "L81_AT.h"

volatile static float input_value1 = 0.0;
volatile static float input_value2 = 0.0;
volatile static float input_value3 = 0.0;
volatile static float input_value4 = 0.0;
volatile static float input_value5 = 0.0;
volatile static float input_value6 = 0.0;

volatile static float output_value1 = 0.0;
volatile static float output_value2 = 0.0;
volatile static float output_value3 = 0.0;
volatile static float output_value4 = 0.0;
volatile static float output_value5 = 0.0;
volatile static float output_value6 = 0.0;

volatile static float target_angle1 = 0.0;
volatile static float target_angle2 = 0.0;
volatile static float target_angle3 = 0.0;
volatile static float target_angle4 = 0.0;
volatile static float target_angle5 = 0.0;
volatile static float target_angle6 = 0.0;

volatile static float last_target_angle1 = 0.0;
volatile static float last_target_angle2 = 0.0;
volatile static float last_target_angle3 = 0.0;
volatile static float last_target_angle4 = 0.0;
volatile static float last_target_angle5 = 0.0;
volatile static float last_target_angle6 = 0.0;

volatile static float P = 12.0;
volatile static float I = 0.0;
volatile static float D = 0.0;

volatile static float last_err1 = 0.0;
volatile static float last_err2 = 0.0;
volatile static float last_err3 = 0.0;
volatile static float last_err4 = 0.0;
volatile static float last_err5 = 0.0;
volatile static float last_err6 = 0.0;

volatile static float integral1 = 0.0;
volatile static float integral2 = 0.0;
volatile static float integral3 = 0.0;
volatile static float integral4 = 0.0;
volatile static float integral5 = 0.0;
volatile static float integral6 = 0.0;


/************************************motor operation brief*******************************************/
/* initialize sequence
    1. l81_motor_gpio_config
    2. l81_ldo_on enable                                                                              
    3. l81_motor_poweron                                                                              
    4. l81_motor_timer_config(TIMER11 is for motor0/1 and TIMER2 is for motor/2/3/4/5)
	  5. l81_adc_config   will feedback the adc of motor rotation angle
	 operation functions
	  1. motor_set_angle directly set angle
		2. motor_set_pulse directly set pulse, you should be familar with the relatioinship of angle and ADC value
*/
/****************************************************************************************************/

/**
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
  */
void l81_motor_gpio_config(void)
{
		/* enable the GPIO clock */
	rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	
	/////////////////////////////////////////Motor EN pin//////////////////////////////////////////
	
 /* configure Motor control GPIO pin */
  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
	
	/////////////////////////////////////////TIMERx////////////////////////////////////////////////
	
  /* enable TIMER2/11 GPIO clock
  GPIOB, TIMER11_CH0/1 which used for motor1/2
  GPIOC, TIMER2_CH0/1/2/3 which used for motor3/4/5/6
	*/
  /*configure PB14/15 as alternate function*/
  gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_14 | GPIO_PIN_15);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14 | GPIO_PIN_15);
  gpio_af_set(GPIOB, GPIO_AF_2, GPIO_PIN_14 | GPIO_PIN_15);
	
  /*Configure PC6/7/8/9(TIMER2_CH0/1/2/3) as alternate function*/
  gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
  gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
  gpio_af_set(GPIOC, GPIO_AF_1, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);	
		
  /*ear pwr ctrl pin init --0425 wxf add*/  
  gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_2 | GPIO_PIN_3);
  gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2 | GPIO_PIN_3);

  
  /* set Motor EN pin */
  gpio_bit_set(GPIOA, GPIO_PIN_8);  //body motor pwr 
  gpio_bit_set(GPIOC, GPIO_PIN_2);  //ear left motor pwr --0425 wxf add
  gpio_bit_set(GPIOC, GPIO_PIN_3);  //ear right motor pwr --0425 wxf add
}
/**
    \brief      set Motor EN pin, 
                Note. before enable motor enable pin, must to enable ldo by call function l81_ldo_on();
                if didn't call l81_ldo_on, even call motor EN pin, the motor wouldn't work.
                Note. this should be called after l81_motor_gpio_config.
    \param[in]  none
    \param[out] none
    \retval     none
  */
void l81_motor_poweron(void)
{
	/* set Motor EN pin */
  gpio_bit_set(GPIOA, GPIO_PIN_8);	
}
/**
    \brief      disable Motor EN pin
    \param[in]  none
    \param[out] none
    \retval     none
  */
void l81_motor_poweroff(void)
{
	/* set Motor EN pin */
  gpio_bit_reset(GPIOA, GPIO_PIN_8);	
}
/**
    \brief      configure the TIMER peripheral
                motor1/2 use TIIMER11, motor3/4/5/6 use TIMER2
    \param[in]  none
    \param[out] none
    \retval     none
  */
void l81_motor_timer_config(uint32_t timer_periph, rcu_periph_enum rcu_periph)
{
    /* TIMER2/11 configuration: generate PWM signals with different duty cycles:
       TIMER2/11CLK = SystemCoreClock / 64 = 1MHz */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

  if(timer_periph == TIMER11)
  {
		/*enalbe TIIMER2/11 clock*/
    rcu_periph_clock_enable(rcu_periph);
    timer_deinit(timer_periph);

    /* TIMER2/11 configuration */
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 63;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 20000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(timer_periph, &timer_initpara);

    /* CHx configuration in PWM mode */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_LOW;
    timer_channel_output_config(timer_periph, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_config(timer_periph, TIMER_CH_1, &timer_ocintpara);
//    timer_channel_output_config(timer_periph, TIMER_CH_2, &timer_ocintpara);
//    timer_channel_output_config(timer_periph, TIMER_CH_3, &timer_ocintpara);

    timer_channel_output_pulse_value_config(timer_periph, TIMER_CH_0, 1500);
    timer_channel_output_mode_config(timer_periph, TIMER_CH_0, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(timer_periph, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    timer_channel_output_pulse_value_config(timer_periph, TIMER_CH_1, 1500);
    timer_channel_output_mode_config(timer_periph, TIMER_CH_1, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(timer_periph, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

//    timer_channel_output_pulse_value_config(timer_periph, TIMER_CH_2, 1500);
//    timer_channel_output_mode_config(timer_periph, TIMER_CH_2, TIMER_OC_MODE_PWM1);
//    timer_channel_output_shadow_config(timer_periph, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

//    timer_channel_output_pulse_value_config(timer_periph, TIMER_CH_3, 1500);
//    timer_channel_output_mode_config(timer_periph, TIMER_CH_3, TIMER_OC_MODE_PWM1);
//    timer_channel_output_shadow_config(timer_periph, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);
		
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(timer_periph);

    /* TIMER enable */
    timer_enable(timer_periph);
  }
  else if(timer_periph == TIMER2)
  {
    /*enalbe TIIMER2/11 clock*/
    rcu_periph_clock_enable(rcu_periph);
    timer_deinit(timer_periph);

    /* TIMER2/11 configuration */
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 63;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 20000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(timer_periph, &timer_initpara);

    /* CHx configuration in PWM mode */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_LOW;
    timer_channel_output_config(timer_periph, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_config(timer_periph, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(timer_periph, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_config(timer_periph, TIMER_CH_3, &timer_ocintpara);

    timer_channel_output_pulse_value_config(timer_periph, TIMER_CH_0, 1500);
    timer_channel_output_mode_config(timer_periph, TIMER_CH_0, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(timer_periph, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    timer_channel_output_pulse_value_config(timer_periph, TIMER_CH_1, 1500);
    timer_channel_output_mode_config(timer_periph, TIMER_CH_1, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(timer_periph, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    timer_channel_output_pulse_value_config(timer_periph, TIMER_CH_2, 1500);
    timer_channel_output_mode_config(timer_periph, TIMER_CH_2, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(timer_periph, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    timer_channel_output_pulse_value_config(timer_periph, TIMER_CH_3, 1500);
    timer_channel_output_mode_config(timer_periph, TIMER_CH_3, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(timer_periph, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);
		
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(timer_periph);

    /* TIMER enable */
    timer_enable(timer_periph);
  }
}

uint32_t PID_core(float target, float output, volatile float *integral, volatile float *last_err)
{
	/* change from ADC value to angle value*/
	float output_angel = 0.0817 * output - 46.302;
	float err = target - output_angel;
	float input = 0.0;
	
	*integral += err;
	
	input = P * err + I * (*integral) + D * (err - (*last_err));
	
	*last_err = err;
	
	if (input > 2500) //motor pulse width is 500us ~ 2500us
		return  2500U;
	if (input < 500)
		return  500U;
	
	return (uint32_t)input;
}

/*!
    \brief      Motor_Ops
    \param[in]  Motor_num
								MOTOR_NUM1
								MOTOR_NUM2
								MOTOR_NUM3
								MOTOR_NUM4
								MOTOR_NUM5
								MOTOR_NUM6
    \param[out] none
    \retval     none
*/
void motor_PID_set_angle(MOTOR_NUM_TYPE Motor)
{	
	uint32_t pulse = 0;
	switch (Motor){
		case MOTOR_NUM1:
			pulse = PID_core(target_angle1, output_value1, &integral1, &last_err1);
			timer_channel_output_pulse_value_config(TIMER11, TIMER_CH_0, pulse);
		  output_value1 = (float)l81_adc_channel_get_sample(ADC_CHANNEL_0);
			break;		
		case MOTOR_NUM2:
			pulse = PID_core(target_angle2, output_value2, &integral2, &last_err2);
			timer_channel_output_pulse_value_config(TIMER11, TIMER_CH_1, pulse);
		  output_value2 = (float)l81_adc_channel_get_sample(ADC_CHANNEL_1);
			break;		
		case MOTOR_NUM3:
			pulse = PID_core(target_angle3, output_value3, &integral3, &last_err3);
			timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, pulse);
		  output_value3 = (float)l81_adc_channel_get_sample(ADC_CHANNEL_2);
			break;
		case MOTOR_NUM4:
			pulse = PID_core(target_angle4, output_value4, &integral4, &last_err4);
			timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, pulse);
			output_value4 = (float)l81_adc_channel_get_sample(ADC_CHANNEL_3);
			break;
		case MOTOR_NUM5:
			pulse = PID_core(target_angle5, output_value5, &integral5, &last_err5);
			timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_2, pulse);
			output_value5 = (float)l81_adc_channel_get_sample(ADC_CHANNEL_4);
			break;
		case MOTOR_NUM6:
			pulse = PID_core(target_angle6, output_value6, &integral6, &last_err6);
			timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, pulse);
			output_value6 = (float)l81_adc_channel_get_sample(ADC_CHANNEL_5);
			break;
		default:
			break;
	};
}

void motor_set_angle(MOTOR_NUM_TYPE motor, uint32_t angle)
{
	switch (motor){
		case MOTOR_NUM1:
			if (angle != last_target_angle1)
			{
				target_angle1 = (float)angle;
				last_target_angle1 = target_angle1;
		    break;
			}
			break;
		case MOTOR_NUM2:
			if (angle != last_target_angle2)
			{
				target_angle2 = (float)angle;
				last_target_angle2 = target_angle2;
		    break;
			}
		  break;
		case MOTOR_NUM3:
			if (angle != last_target_angle3)
			{
				target_angle3 = (float)angle;
				last_target_angle3 = target_angle3;
		    break;
			}
		  break;
		case MOTOR_NUM4:
			if (angle != last_target_angle4)
			{
				target_angle4 = (float)angle;
				last_target_angle4 = target_angle4;
		    break;
			}
		  break;
		case MOTOR_NUM5:
			if (angle != last_target_angle5)
			{
				target_angle5 = (float)angle;
				last_target_angle5 = target_angle5;
		    break;
			}
		  break;
		case MOTOR_NUM6:
			if (angle != last_target_angle6)
			{
				target_angle6 = (float)angle;
				last_target_angle6 = target_angle6;
		    break;
			}
		  break;
    default:
      break;			
	}
	
	motor_PID_set_angle(motor);
}

/*!
    \brief      directly set pulse
    \param[in]  motor
       \arg     which motor
	               MOTOR_NUM1
	               MOTOR_NUM2
	               MOTOR_NUM3
	               MOTOR_NUM4
	               MOTOR_NUM5
	               MOTOR_NUM6
		\parm[in]   pulse
       \arg      the pulse width 500us~2500us
    \param[out] uint16_t
       \arg       the ADC value of angle 
    \retval     none
*/
uint16_t motor_set_pulse(MOTOR_NUM_TYPE motor, uint32_t pulse)
{
	switch (motor){
		case MOTOR_NUM1:
			timer_channel_output_pulse_value_config(TIMER11, TIMER_CH_0, pulse);
		  return l81_adc_channel_get_sample(ADC_CHANNEL_0);
		case MOTOR_NUM2:
			timer_channel_output_pulse_value_config(TIMER11, TIMER_CH_1, pulse);
		  return l81_adc_channel_get_sample(ADC_CHANNEL_1);
		case MOTOR_NUM3:
			timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, pulse);
		  return l81_adc_channel_get_sample(ADC_CHANNEL_2);
		case MOTOR_NUM4:
			timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, pulse);
			return l81_adc_channel_get_sample(ADC_CHANNEL_3);
		case MOTOR_NUM5:
			timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_2, pulse);
			return l81_adc_channel_get_sample(ADC_CHANNEL_4);
		case MOTOR_NUM6:
			timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, pulse);
			return l81_adc_channel_get_sample(ADC_CHANNEL_5);
		default:
			break;
	};
	
	return 0U;
}

uint8_t l81_AT_MOTOR_W_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;

  ATcmd_split_params(params, param, &param_num);
	
	if (param_num != 3U){  //motor must has three paramters, (motor number, value type(angle or pulse), value)
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
	uint32_t motor_num = String2Int(param[0]);
	uint32_t value_kind = String2Int(param[1]);
	uint32_t value = String2Int(param[2]);
		
	if (motor_num < 1U || motor_num > 6U) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong motor number %d, motor number is[1,6]\r\n", motor_num);
		printf("AT+RES,end\r\n");
		return 0U;
	}
	if (value_kind > 1U) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong value_kind %d, value_kind is[0,1]\r\n", value_kind);
		printf("AT+RES,end\r\n");
		return 0U;
	}	
	if (value < 500U || value > 2500U) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong value %d, value is[500,2500]\r\n", value);
		printf("AT+RES,end\r\n");
		return 0U;
	}	
	
		if(MOTOR_NUM1 == motor_num){
			if (0U == value_kind)   // 0 means set angle, and 1  means set pulse
				motor_set_angle(MOTOR_NUM1, value);
			else
				motor_set_pulse(MOTOR_NUM1, value);
		}	else if (MOTOR_NUM2 == motor_num){
			if (0U == value_kind)   // 0 means set angle, and 1  means set pulse
				motor_set_angle(MOTOR_NUM2, value);
			else
				motor_set_pulse(MOTOR_NUM2, value);
		} else if (MOTOR_NUM3 == motor_num){
			if (0U == value_kind)   // 0 means set angle, and 1  means set pulse
				motor_set_angle(MOTOR_NUM3, value);
			else
				motor_set_pulse(MOTOR_NUM3, value);
		} else if (MOTOR_NUM4 == motor_num){
			if (0U == value_kind)   // 0 means set angle, and 1  means set pulse
				motor_set_angle(MOTOR_NUM4, value);
			else
				motor_set_pulse(MOTOR_NUM4, value);
		} else if (MOTOR_NUM5 == motor_num){
			if (0U == value_kind)   // 0 means set angle, and 1  means set pulse
				motor_set_angle(MOTOR_NUM5, value);
			else
				motor_set_pulse(MOTOR_NUM5, value);
		} else if (MOTOR_NUM6 == motor_num){
			if (0U == value_kind)   // 0 means set angle, and 1  means set pulse
				motor_set_angle(MOTOR_NUM6, value);
			else
				motor_set_pulse(MOTOR_NUM6, String2Int(param[2]));
		} 
		
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,end\r\n");
  return 1U;
}

uint8_t l81_AT_MOTOR_R_func(char params[])
{
  char param[PARAM_MAX_NUM][PER_PARAM_MAX_LEN] = {'\0'};
  uint8_t param_num = 0;
	uint16_t ADC_value = 0;

  ATcmd_split_params(params, param, &param_num);

	if (param_num != 1U){  //motor must has one paramters, (motor number)
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong param num\r\n");
		printf("AT+RES,end\r\n");
		return 0U; 
	}
	
	uint32_t motor_num = String2Int(param[0]);
		
	if (motor_num < 1U || motor_num > 6U) {
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,Err,wrong motor number %d, motor number is[1,6]\r\n", motor_num);
		printf("AT+RES,end\r\n");
		return 0U;
	}
	
		if(MOTOR_NUM1 == motor_num){
			ADC_value = l81_adc_channel_get_sample(ADC_CHANNEL_0);
		}	else if (MOTOR_NUM2 == motor_num){
			ADC_value = l81_adc_channel_get_sample(ADC_CHANNEL_1);
		} else if (MOTOR_NUM3 == motor_num){
			ADC_value = l81_adc_channel_get_sample(ADC_CHANNEL_2);
		} else if (MOTOR_NUM4 == motor_num){
			ADC_value = l81_adc_channel_get_sample(ADC_CHANNEL_3);
		} else if (MOTOR_NUM5 == motor_num){
			ADC_value = l81_adc_channel_get_sample(ADC_CHANNEL_4);
		} else if (MOTOR_NUM6 == motor_num){
			ADC_value = l81_adc_channel_get_sample(ADC_CHANNEL_5);
		} 
		
		printf("AT+RES,ACK\r\n");
		printf("AT+RES,motor,%d,%d\r\n", motor_num, ADC_value);
		printf("AT+RES,end\r\n");
		
    return 1U;
}
/*!
    \brief      Initialization for Motor0/1/2/3/4/5. after this, customer could set motor angle or pulse
                Beacase of motor needs ADC feedback, before motor init, you should call l81_adc_init()
    \param[in]  none
    \param[out] none
    \retval     none
*/
void l81_motor_init(void)
{
	l81_motor_gpio_config();
	l81_ldo_gpio_config();
	
	l81_motor_timer_config(TIMER11, RCU_TIMER11);
	l81_motor_timer_config(TIMER2, RCU_TIMER2);
	
	l81_ldo_on();
	l81_motor_poweron();
	
}

