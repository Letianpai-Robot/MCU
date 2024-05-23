/*!
    \file    i2c_inf.c
    \brief   the read and write function file

    \version 2021-08-04, V1.0.0, firmware for GD32L23x
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
#include "i2c_inf.h"
#include "L81_inf.h"
#include <stdio.h>

#define BUFFER_SIZE              1
#define MAX_RELOAD_SIZE          255

/*!
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable GPIOC clock */
    rcu_periph_clock_enable(RCU_GPIOC);
	
	//I2C1
    /* connect PB10 to I2C_SCL */
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_10);
    /* connect PB11 to I2C_SDA */
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_11);
    /* configure GPIO pins of I2C */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_11);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
	
	//I2C2
		/* connect PC0 to I2C_SCL */
    gpio_af_set(GPIOC, GPIO_AF_4, GPIO_PIN_0);
    /* connect PC1 to I2C_SDA */
    gpio_af_set(GPIOC, GPIO_AF_4, GPIO_PIN_1);
    /* configure GPIO pins of I2C */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_0);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_1);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
}

/*!
    \brief      configure the I2C interface only support I2C1 and I2C2
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_config(uint32_t i2c_periph, rcu_periph_enum periph)
{
		if (i2c_periph == I2C0)
			return;
		
	  gpio_config();
	    /* enable I2C clock */
    rcu_periph_clock_enable(periph);
	
    /* configure I2C timing */
    i2c_timing_config(i2c_periph, 0, 0x3, 0);
	  //the sencond param is used for high level, the third param is used for period.
    i2c_master_clock_config(i2c_periph, 0x33, 0x36);  // high level 1.3us, low level 1.5us, freg is 340khz, rising time 410ns
    /* enable I2C */
    i2c_enable(i2c_periph);
}

/*!
    \brief      reset I2C gpio configure
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_gpio_reset(void)
{
    /* reset PB10 and PB11 */
    gpio_mode_set(I2C_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, I2C_SCL_PIN);
    gpio_output_options_set(I2C_SCL_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, I2C_SCL_PIN);
    gpio_mode_set(I2C_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, I2C_SDA_PIN);
    gpio_output_options_set(I2C_SDA_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, I2C_SDA_PIN);
}

/*!
    \brief      reset i2c bus
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_bus_reset()
{
    /* configure SDA/SCL for GPIO */
    GPIO_BC(I2C_SCL_PORT) |= I2C_SCL_PIN;
    GPIO_BC(I2C_SDA_PORT) |= I2C_SDA_PIN;
    /* reset PB10 and PB11 */
    i2c_gpio_reset();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    /* stop signal */
    GPIO_BOP(I2C_SCL_PORT) |= I2C_SCL_PIN;
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    GPIO_BOP(I2C_SDA_PORT) |= I2C_SDA_PIN;
    /* connect PB10 to I2C_SCL */
    /* connect PB11 to I2C_SDA */
    gpio_mode_set(I2C_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SCL_PIN);
    gpio_output_options_set(I2C_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SCL_PIN);
    gpio_mode_set(I2C_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SDA_PIN);
    gpio_output_options_set(I2C_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SDA_PIN);
}

/*!
    \brief      write buffer of data to the I2C EEPROM
		\param[in]  slave_address: i2c slave addr
    \param[in]  p_buffer: pointer to the buffer  containing the data to be written to the EEPROM
    \param[in]  write_address: EEPROM's internal address to write to
    \param[in]  number_of_byte: number of bytes to write to the EEPROM
    \param[out] none
    \retval     none
*/
void i2c_buffer_write(uint32_t i2c_periph, uint16_t slave_address, uint8_t write_address, uint8_t *p_buffer, uint16_t number_of_byte)
{
    uint8_t number_of_page = 0, number_of_single = 0, address = 0, count = 0;

    address = write_address % I2C_PAGE_SIZE;
    count = I2C_PAGE_SIZE - address;
    number_of_page =  number_of_byte / I2C_PAGE_SIZE;
    number_of_single = number_of_byte % I2C_PAGE_SIZE;

    /* if write_address is I2C_PAGE_SIZE aligned */
    if(0 == address) {
        while(number_of_page--) {
            i2c_page_write(i2c_periph, slave_address, write_address, p_buffer, I2C_PAGE_SIZE);
            delay_1ms(1);//delay_1ms(5);
            write_address +=  I2C_PAGE_SIZE;
            p_buffer += I2C_PAGE_SIZE;
        }
        if(0 != number_of_single) {
            i2c_page_write(i2c_periph, slave_address, write_address, p_buffer, number_of_single);
            delay_1ms(1);//delay_1ms(5);
        }
    } else {
        /* if write_address is not I2C_PAGE_SIZE aligned */
        if(number_of_byte < count) {
            i2c_page_write(i2c_periph, slave_address, write_address, p_buffer, number_of_byte);
            delay_1ms(1);//delay_1ms(5);
        } else {
            number_of_byte -= count;
            number_of_page =  number_of_byte / I2C_PAGE_SIZE;
            number_of_single = number_of_byte % I2C_PAGE_SIZE;
            if(0 != count) {
                i2c_page_write(i2c_periph, slave_address, write_address, p_buffer, count);
                delay_1ms(1);//delay_1ms(5);
                write_address += count;
                p_buffer += count;
            }
            /* write page */
            while(number_of_page--) {
                i2c_page_write(i2c_periph, slave_address, write_address, p_buffer, I2C_PAGE_SIZE);
                delay_1ms(1);//delay_1ms(5);
                write_address +=  I2C_PAGE_SIZE;
                p_buffer += I2C_PAGE_SIZE;
            }
            /* write single */
            if(0 != number_of_single) {
                i2c_page_write(i2c_periph, slave_address, write_address, p_buffer, number_of_single);
                delay_1ms(1);//delay_1ms(5);
            }
        }
    }
}

/*!
    \brief      write more than one byte to the EEPROM with a single write cycle
		\param[in]  slave_address: i2c slave addr
    \param[in]  p_buffer: pointer to the buffer containing the data to be written to the EEPROM
    \param[in]  write_address: EEPROM's internal address to write to
    \param[in]  number_of_byte: number of bytes to write to the EEPROM
    \param[out] none
    \retval     none
*/
void i2c_page_write(uint32_t i2c_periph, uint16_t slave_address, uint8_t write_address, uint8_t *p_buffer, uint16_t number_of_byte)
{
    i2c_process_enum state = I2C_START;
    uint16_t timeout = 0;
    uint8_t end_flag = 0;

		uint8_t retry = 0u;
	
    while(!end_flag) {
			if (retry++ >= 10)  //try 10 times will exit forbiddent loop all the time
				break;			
			
        switch(state) {
        case I2C_START:
            /* configure slave address */
            i2c_master_addressing(i2c_periph, slave_address, I2C_MASTER_TRANSMIT);
            /* configure number of bytes to be transferred */
            i2c_transfer_byte_number_config(i2c_periph, number_of_byte + 1);
            /* clear I2C_TDATA register */
            I2C_STAT(i2c_periph) |= I2C_STAT_TBE;
            /* enable I2C automatic end mode in master mode */
            i2c_automatic_end_enable(i2c_periph);
            /* i2c master sends start signal only when the bus is idle */
            while(i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) {
                i2c_start_on_bus(i2c_periph);
                timeout = 0;
                state = I2C_SEND_ADDRESS;
            } else {
                /* timeout, bus reset */
                i2c_bus_reset();
                timeout = 0;
                state = I2C_START;
                printf("i2c bus is busy in page write!\n");
            }
            break;
        case I2C_SEND_ADDRESS:
            /* wait until the transmit data buffer is empty */
            while((!i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) {
                /* send the EEPROM's internal address to write to : only one byte address */
                i2c_data_transmit(i2c_periph, write_address);
                timeout = 0;
                state = I2C_TRANSMIT_DATA;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("i2c master sends address timeout in page write!\n");
            }
            break;
        case I2C_TRANSMIT_DATA:
            while(number_of_byte--) {
                /* wait until TI bit is set */
                while((!i2c_flag_get(i2c_periph, I2C_FLAG_TI)) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }
                if(timeout < I2C_TIME_OUT) {
                    /* while there is data to be written */
                    i2c_data_transmit(i2c_periph, *p_buffer);
                    /* point to the next byte to be written */
                    p_buffer++;
                    timeout = 0;
                    state = I2C_STOP;
                } else {
                    /* wait TI timeout */
                    timeout = 0;
                    state = I2C_START;
                    printf("i2c master sends data timeout in page write!\n");
                    return ;
                }
            }
            break;
        case I2C_STOP:
            /* wait until the stop condition is finished */
            while((!i2c_flag_get(i2c_periph, I2C_FLAG_STPDET)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) {
                /* clear STPDET flag */
                i2c_flag_clear(i2c_periph, I2C_FLAG_STPDET);
                timeout = 0;
                state = I2C_END;
                end_flag = 1;
            } else {
                /* stop detect timeout */
                timeout = 0;
                state = I2C_START;
                printf("i2c master sends stop signal timeout in page write!\n");
            }
            break;
        default:
            /* default status */
            state = I2C_START;
            end_flag = 1;
            timeout = 0;
            printf("i2c master sends start signal in page write!\n");
            break;
        }
    }
}

/*!
    \brief      read data from the EEPROM
		\param[in]  slave_address: i2c slave addr
    \param[in]  p_buffer: pointer to the buffer that receives the data read from the EEPROM
    \param[in]  read_address: EEPROM's internal address to start reading from
    \param[in]  number_of_byte: number of bytes to reads from the EEPROM
    \param[out] none
    \retval     none
*/
void i2c_buffer_read(uint32_t i2c_periph, uint16_t slave_address, uint8_t read_address, uint8_t *p_buffer, uint16_t number_of_byte)
{
    uint32_t nbytes_reload = 0;
    i2c_process_enum state = I2C_START;
    uint32_t timeout = 0;
    uint8_t end_flag = 0;
    uint8_t restart_flag = 0;
    uint8_t first_reload_flag = 1;
	
		uint8_t retry = 0u;

    while(!end_flag) {
			if (retry++ >= 10)  //try 10 times will exit forbiddent loop all the time
				break;
			
        switch(state) {
        case I2C_START:
            if(0 == restart_flag) {
                /* clear I2C_TDATA register */
                I2C_STAT(i2c_periph) |= I2C_STAT_TBE;
                /* configure slave address */
							 i2c_address_config(I2C0, slave_address, I2C_ADDFORMAT_7BITS);
                i2c_master_addressing(i2c_periph, slave_address, I2C_MASTER_TRANSMIT);
                /* configure number of bytes to be transferred */
								i2c_transfer_byte_number_config(i2c_periph, 1);
                /* disable I2C automatic end mode in master mode */
                i2c_automatic_end_disable(i2c_periph);
                /* i2c master sends start signal only when the bus is idle */
                while(i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }
                if(timeout < I2C_TIME_OUT) {
                    i2c_start_on_bus(i2c_periph);
                    timeout = 0;
                    state = I2C_SEND_ADDRESS;
                } else {
                    /* timeout, bus reset */
                    i2c_bus_reset();
                    timeout = 0;
                    state = I2C_START;
                    printf("i2c bus is busy in read!\n");
                }
            } else {
                /* restart */
                i2c_start_on_bus(i2c_periph);
                restart_flag = 0;
                state = I2C_TRANSMIT_DATA;
            }
            break;
        case I2C_SEND_ADDRESS:
            /* wait until the transmit data buffer is empty */
            while((!i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) {
                /* send the EEPROM's internal address to write to : only one byte address */
								i2c_data_transmit(i2c_periph, read_address);
                timeout = 0;
                state = I2C_RESTART;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("i2c master sends data timeout in read!\n");
            }
            break;
        case I2C_RESTART:
            /* wait until the transmit data buffer is empty */
            while((!i2c_flag_get(i2c_periph, I2C_FLAG_TC)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) {
                /* configure the EEPROM's internal address to write to : only one byte address */
							 i2c_address_config(I2C0, slave_address, I2C_ADDFORMAT_7BITS);
                i2c_master_addressing(i2c_periph, slave_address, I2C_MASTER_RECEIVE);
                /* enable I2C reload mode */
                i2c_reload_enable(i2c_periph);
                /* configure number of bytes to be transferred */
                timeout = 0;
                state = I2C_RELOAD;
                restart_flag = 1;
            } else {
                timeout = 0;
                state = I2C_START;
             //   printf("i2c master sends EEPROM's internal address timeout in read!\n");
            }
            break;
        case I2C_RELOAD:
            if(number_of_byte > MAX_RELOAD_SIZE) {
                number_of_byte = number_of_byte - MAX_RELOAD_SIZE;
                nbytes_reload = MAX_RELOAD_SIZE;
            } else {
                nbytes_reload = number_of_byte;
            }
            if(1 == first_reload_flag) {
                /* configure number of bytes to be transferred */
                i2c_transfer_byte_number_config(i2c_periph, nbytes_reload);
                first_reload_flag = 0;
                state = I2C_START;
            } else {
                /* wait for TCR flag */
                while((!i2c_flag_get(i2c_periph, I2C_FLAG_TCR)) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }
                if(timeout < I2C_TIME_OUT) {
                    /* configure number of bytes to be transferred */
                    i2c_transfer_byte_number_config(i2c_periph, nbytes_reload);
                    /* disable I2C reload mode */
                    if(number_of_byte <= MAX_RELOAD_SIZE) {
                        i2c_reload_disable(i2c_periph);
                        /* enable I2C automatic end mode in master mode */
                        i2c_automatic_end_enable(i2c_periph);
                    }
                    timeout = 0;
                    state = I2C_TRANSMIT_DATA;
                } else {
                    timeout = 0;
                    state = I2C_START;
                    printf("i2c master reload data timeout in read!\n");
                }
            }
            break;
        case I2C_TRANSMIT_DATA:
            /* wait until TI bit is set */
            while((!i2c_flag_get(i2c_periph, I2C_FLAG_TBE)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) {
                while(nbytes_reload) {
                    /* wait until the RBNE bit is set and clear it */
                    if(i2c_flag_get(i2c_periph, I2C_FLAG_RBNE)) {
                        /* read a byte from the EEPROM */
                        *p_buffer = i2c_data_receive(i2c_periph);
                        /* point to the next location where the byte read will be saved */
                        p_buffer++;
                        /* decrement the read bytes counter */
                        nbytes_reload--;
                    }
                }
                timeout = 0;
                /* check if the reload mode is enabled or not */
                if(I2C_CTL1(i2c_periph) & I2C_CTL1_RELOAD) {
                    timeout = 0;
                    state = I2C_RELOAD;
                } else {
                    timeout = 0;
                    state = I2C_STOP;
                }
            } else {
                /* wait TI timeout */
                timeout = 0;
                state = I2C_START;
                printf("i2c master read data timeout in read!\n");
            }
            break;
        case I2C_STOP:
            /* wait until the stop condition is finished */
            while((!i2c_flag_get(i2c_periph, I2C_FLAG_STPDET)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }
            if(timeout < I2C_TIME_OUT) {
                /* clear STPDET flag */
                i2c_flag_clear(i2c_periph, I2C_FLAG_STPDET);
                timeout = 0;
                state = I2C_END;
                end_flag = 1;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("i2c master sends stop signal timeout in read!\n");
            }
            break;
        default:
            /* default status */
            state = I2C_START;
            end_flag = 1;
            timeout = 0;
            printf("i2c master sends start signal in read!\n");
            break;
        }
    }
}

/*-------------------------------------------------gpio simulate i2c-----------------------------------------------------------*/

void sw_i2c_gpio_config(void)
{
	rcu_periph_clock_enable(RCU_GPIOB);
	gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_8 | GPIO_PIN_9);
  gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
	
	gpio_bit_set(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
}

#define sw_i2c_clk_1()  (GPIO_BOP(GPIOB) = GPIO_PIN_8)
#define sw_i2c_clk_0()  (GPIO_BC(GPIOB)  = GPIO_PIN_8)
#define sw_i2c_sda_1()  (GPIO_BOP(GPIOB) = GPIO_PIN_9)
#define sw_i2c_sda_0()  (GPIO_BC(GPIOB)  = GPIO_PIN_9)

#define sw_i2c_clk_read() (GPIO_ISTAT(GPIOB) & (GPIO_PIN_8))
#define sw_i2c_sda_read() (GPIO_ISTAT(GPIOB) & (GPIO_PIN_9))

void sw_i2c_set_sda_output(void)
{
	gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_9);
}

void sw_i2c_set_sda_input(void)
{
	gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_9);
}

#define sw_i2c_sda_input()    sw_i2c_set_sda_input()
#define sw_i2c_sda_output()   sw_i2c_set_sda_output()
#define IsTopTouch
#ifdef IsTopTouch 
	void  delay_us_new()
    {
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();//72 __NOP(); 1us
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();//72 __NOP(); 1us
    }
void sw_i2c_delay(void)
{
//	uint8_t count = 4;  //110khz
//	for(;count > 0;count--){}
	//delay_4us(1);   //125khz
	delay_us_new();
}
#else
void sw_i2c_delay(void)
{
//	uint8_t count = 4;  //110khz
//	for(;count > 0;count--){}
	delay_4us(1);   //125khz
}
#endif
void sw_i2c_start(void)
{
	sw_i2c_sda_1();
	sw_i2c_clk_1();
	sw_i2c_delay();
	sw_i2c_sda_0();
	sw_i2c_delay();
	sw_i2c_clk_0();
	sw_i2c_delay();
}

void sw_i2c_stop(void)
{
	sw_i2c_sda_0();
	sw_i2c_clk_1();
	sw_i2c_delay();
	sw_i2c_sda_1();	
	sw_i2c_delay();
}

void sw_i2c_ack(void)
{
	sw_i2c_sda_0();
	sw_i2c_delay();
	sw_i2c_clk_1();
	sw_i2c_delay();
	sw_i2c_clk_0();
	sw_i2c_delay();
	sw_i2c_clk_1();
}

void sw_i2c_nack(void)
{
	sw_i2c_sda_1();
	sw_i2c_delay();
	sw_i2c_clk_1();
	sw_i2c_delay();
	sw_i2c_clk_0();
	sw_i2c_delay();
}

uint8_t sw_i2c_wait_ack(void)
{
	uint8_t sda_status;
	uint8_t wait_time = 0u;
	uint8_t ack_nack = 1;
	
	sw_i2c_sda_input();
	
	while(sw_i2c_sda_read())
	{
		wait_time++;
		
		if (wait_time >= 200){
			ack_nack = 0;
			break;
		}
	}
	
	sw_i2c_delay();
	sw_i2c_clk_1();
	sw_i2c_delay();
	
	sw_i2c_clk_0();
	sw_i2c_delay();
	sw_i2c_sda_output();
	sw_i2c_delay();
	
	return ack_nack;
}

void sw_i2c_writeByte(uint8_t data)
{
	uint8_t i;
	
	for(i = 0u; i < 8u; i++){
		sw_i2c_clk_0();
		sw_i2c_delay();
		if(data & 0x80u)
			sw_i2c_sda_1();
		else
			sw_i2c_sda_0();
		
		sw_i2c_delay();
		sw_i2c_clk_1();
		sw_i2c_delay();
		data <<= 1;
	}
	
	sw_i2c_clk_0();
	sw_i2c_delay();
}

uint8_t sw_i2c_readByte(void)
{
	uint8_t i = 0u, data = 0u;
	
	sw_i2c_sda_input();
	for(i = 0u; i < 8; i++){
		data <<= 1;
		sw_i2c_delay();
		
		sw_i2c_clk_1();
		sw_i2c_delay();
		
		if (sw_i2c_sda_read())
			data |= 0x01;
		
		sw_i2c_clk_0();
	}
	
	sw_i2c_sda_output();
	return data;
}


void sw_i2c_write_nBytes(uint8_t slave_addr, uint8_t *data, uint8_t len)
{
	uint8_t j;
	
	//7bit address should left shifft 1 bit
	slave_addr <<= 1;
	
	sw_i2c_start();
	
	sw_i2c_writeByte(slave_addr);
	
	if (!sw_i2c_wait_ack())
		goto err;
	
	for (j = 0u; j < len; j++){
		sw_i2c_writeByte(*(data + j));
		if (!sw_i2c_wait_ack())
			goto err;		
	}
	
	err:
	sw_i2c_stop();
}

void sw_i2c_read_nBytes(uint8_t slave_addr, uint8_t *buf, uint8_t len)
{
	uint8_t j;
	
	slave_addr <<= 1;
	
	sw_i2c_start();
	sw_i2c_writeByte(slave_addr | 0x01);
	
	if(!sw_i2c_wait_ack())
		goto err;
	
	for(j = 0; j < len; j++){
		buf[j] = sw_i2c_readByte();
		sw_i2c_ack();
	}
	
	err:
	sw_i2c_stop();
}

void sw_i2c_send2read_8bit(uint8_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t j;
	
	slave_addr <<= 1;
	
	sw_i2c_start();
	
	sw_i2c_writeByte(slave_addr);
	if (!sw_i2c_wait_ack())
		goto err;
	
	sw_i2c_writeByte(reg);
	if (!sw_i2c_wait_ack())
		goto err;
	
	sw_i2c_start();
	
	sw_i2c_writeByte(slave_addr | 0x01);
	if (!sw_i2c_wait_ack())
		goto err;	
	
	for (j = 0; j < len; j++){
		buf[j] = sw_i2c_readByte();
		sw_i2c_ack();
	}
	
	err:
	sw_i2c_stop();
}

void sw_i2c_send2read_16bit(uint8_t slave_addr, uint16_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t j;
	
	slave_addr <<= 1;
	
	sw_i2c_start();
	
	sw_i2c_writeByte(slave_addr);
	if (!sw_i2c_wait_ack())
		goto err;
	
	sw_i2c_writeByte((reg >> 8u) & 0xFFu);
	if (!sw_i2c_wait_ack())
		goto err;

	sw_i2c_writeByte(reg & 0xFFu);
	if (!sw_i2c_wait_ack())
		goto err;	
	
	sw_i2c_start();
	
	sw_i2c_writeByte(slave_addr | 0x01);
	if (!sw_i2c_wait_ack())
		goto err;	
	
	for (j = 0; j < len; j++){
		buf[j] = sw_i2c_readByte();
		sw_i2c_ack();
	}
	
	err:
	sw_i2c_stop();
}