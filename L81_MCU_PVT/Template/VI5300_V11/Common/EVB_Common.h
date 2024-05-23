/**
  ******************************************************************************
  * @file    EVB_Common.h
  * @author  L
  * @version V1.0.0
  * @date    22-November-2021
  * @brief   This file contains all the functions prototypes for the Key module.
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef EVB_COMMON_H
#define EVB_COMMON_H

//#ifdef __cplusplus
// extern "C" {
//#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32l23x.h"

/** @addtogroup 
  * @{
  */

/** @addtogroup 
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/**
  * @}
  */

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/

//*LED***********************************************************************/
//PB3
#define EVB_LED0_CLK			RCU_GPIOB
#define EVB_LED0_PORT			GPIOB
#define EVB_LED0_PIN			GPIO_PIN_3
#define EVB_LED0_LOW()		{GPIO_BC(EVB_LED0_PORT) = EVB_LED0_PIN;}
#define EVB_LED0_HIGH()		{GPIO_BOP(EVB_LED0_PORT) = EVB_LED0_PIN;}
//PB4
#define EVB_LED1_CLK			RCU_GPIOB
#define EVB_LED1_PORT			GPIOB
#define EVB_LED1_PIN			GPIO_PIN_4
#define EVB_LED1_LOW()		{GPIO_BC(EVB_LED1_PORT) = EVB_LED1_PIN;}
#define EVB_LED1_HIGH()		{GPIO_BOP(EVB_LED1_PORT) = EVB_LED1_PIN;}



/* definition for COM, connected to USART0 */
#define EVAL_COM                         USART0
#define EVAL_COM_CLK                     RCU_USART0

#define EVAL_COM_TX_PIN                  GPIO_PIN_9
#define EVAL_COM_RX_PIN                  GPIO_PIN_10

#define EVAL_COM_GPIO_PORT               GPIOA
#define EVAL_COM_GPIO_CLK                RCU_GPIOA
#define EVAL_COM_AF                      GPIO_AF_1			

#define USART0_RDATA_ADDRESS      ((uint32_t)&USART_RDATA(EVAL_COM)) 
#define USART0_TDATA_ADDRESS      ((uint32_t)&USART_TDATA(EVAL_COM)) 
//串口UART0接收buff长度	
#define UART0_RX_LEN	100
extern uint8_t l_rx_complete;		
extern uint8_t rxcomm[10];	
//串口接收计数
extern unsigned char uart0_RX_cnt;								
//串口UART0接收buff
extern unsigned char uart0_RX_buff[UART0_RX_LEN];
																			


/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
void EVB_LED_Init(void);
void EVB_UART_Init(uint32_t bound_val);
void EVB_UART_Command(void);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H */

/**
  * @}
  */

/**
  * @}
  */


