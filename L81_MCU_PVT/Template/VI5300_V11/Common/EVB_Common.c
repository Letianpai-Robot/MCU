/***********************************************************************************************************************
Includes
***********************************************************************************************************************/


/* Start user code for include. Do not edit comment generated here */
#include "VI530x_Common.h"
#include "EVB_Common.h"
//#include "stdio.h"
//#include <string.h>
/* End user code. Do not edit comment generated here */
extern VI530x_MEASURE_TypeDef result;

uint8_t rxbuffer[10];
uint8_t rxcomm[10];
uint8_t l_rx_complete = 0;

//串口接收计数
unsigned char uart0_RX_cnt = 0;								
//串口UART0接收buff
unsigned char uart0_RX_buff[UART0_RX_LEN];
//串口UART0发送buff
//unsigned char uart0_TX_buff[UART0_TX_LEN];

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* 以下代码支持printf函数，不需要选择use MicroLIB */
#if PRINT_A
//#pragme import(__use_no_semihosting)
#pragma import(__use_no_semihosting)  
/* 标准库需要的支持函数 */
struct __FILE
{
  int handle;
};

FILE __stdout;

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
  x = x;
}

/* 重定义fputc函数 */
int fputc(int ch, FILE *f)
{
  while((USART2->SR & 0x40) == 0); //USART_FLAG_TC = 0x40:   Transmission Complete flag
	USART2->DR = (uint8_t)ch;
	return ch;
//	UART_HandleTypeDef huart2;
//	HAL_UART_Transmit(&huart2 , (uint8_t *)&ch, 1, 0xFFFF);    
//while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX)
//	{};   
//return ch;
}
#endif
/* End user code. Do not edit comment generated here */


//**LED-ON**//
/***********************************************************************************************************************
* Function Name: EVB_LED_Init
* Description  : LED控制使能脚初始化
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void EVB_LED_Init(void)
{
	/* enable the GPIO clock */
	rcu_periph_clock_enable(EVB_LED0_CLK);
	/* configure GPIO port */ 
	gpio_mode_set(EVB_LED0_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EVB_LED0_PIN);
	gpio_output_options_set(EVB_LED0_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, EVB_LED0_PIN);
	/* configure GPIO port */ 
	gpio_mode_set(EVB_LED1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EVB_LED1_PIN);
	gpio_output_options_set(EVB_LED1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, EVB_LED1_PIN);

	EVB_LED1_HIGH();
	EVB_LED0_HIGH();
}

//**UART**//
/***********************************************************************************************************************
* Function Name: EVB_UART_Init
* Description  : EVB板UART的初始化
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void EVB_UART_Init(uint32_t bound_val)
{
	uint32_t COM_ID;
	
	COM_ID = 0U;

	/* enable COM GPIO clock */
	rcu_periph_clock_enable(EVAL_COM_GPIO_CLK);

	/* enable USART clock */
	rcu_periph_clock_enable(EVAL_COM_CLK);

	/* connect port to USARTx_Tx */
	gpio_af_set(EVAL_COM_GPIO_PORT, EVAL_COM_AF, EVAL_COM_TX_PIN);

	/* connect port to USARTx_Rx */
	gpio_af_set(EVAL_COM_GPIO_PORT, EVAL_COM_AF, EVAL_COM_RX_PIN);

	/* configure USART Tx as alternate function push-pull */
	gpio_mode_set(EVAL_COM_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, EVAL_COM_TX_PIN);
	gpio_output_options_set(EVAL_COM_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, EVAL_COM_TX_PIN);

	/* configure USART Rx as alternate function push-pull */
	gpio_mode_set(EVAL_COM_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, EVAL_COM_RX_PIN);
	gpio_output_options_set(EVAL_COM_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, EVAL_COM_RX_PIN);

	/* USART configure */
	usart_deinit(EVAL_COM);
	usart_baudrate_set(EVAL_COM, bound_val);
	usart_receive_config(EVAL_COM, USART_RECEIVE_ENABLE);
	usart_transmit_config(EVAL_COM, USART_TRANSMIT_ENABLE);
	
	/* enable USART TBE interrupt */
  //usart_interrupt_enable(EVAL_COM, USART_INT_IDLE);
	
	//nvic_irq_enable(USART0_IRQn,2);

	usart_enable(EVAL_COM);
	
	dma_parameter_struct dma_init_struct;	
	
	/* enable DMA clock */
  rcu_periph_clock_enable(RCU_DMA);
	
//	dma_deinit(DMA_CH1);
//	dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
//	dma_init_struct.memory_addr = (uint32_t)uart0_TX_buff;
//	dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
//	dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
//	dma_init_struct.number = sizeof(uart0_TX_buff);
//	dma_init_struct.periph_addr = USART0_TDATA_ADDRESS;
//	dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
//	dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
//	dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
//	dma_init(DMA_CH1,&dma_init_struct);

	dma_deinit(DMA_CH2);
	dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
	dma_init_struct.memory_addr = (uint32_t)uart0_RX_buff;
	dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
	dma_init_struct.number = UART0_RX_LEN;
	dma_init_struct.periph_addr = USART0_RDATA_ADDRESS;
	dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA_CH2, &dma_init_struct);
	
	dma_circulation_disable(DMA_CH2);
	dma_memory_to_memory_disable(DMA_CH2);

	dma_channel_enable(DMA_CH2);
	usart_dma_receive_config(EVAL_COM, USART_DENR_ENABLE);

	nvic_irq_enable(USART0_IRQn,2);
	usart_interrupt_flag_clear(EVAL_COM, USART_INT_FLAG_IDLE);
  usart_interrupt_enable(EVAL_COM, USART_INT_IDLE);	 //串口空闲中断
	/* enable DMA0 channel2 transfer complete interrupt */
	//dma_interrupt_enable(DMA_CH2, DMA_INT_FTF);  //DMA接收中断不开
}

/***********************************************************************************************************************
* Function Name: EVB_UART_Init
* Description  : EVB板UART的初始化
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void EVB_UART_Command(void)
{
	uint8_t ret = 0;
	static uint8_t ranging_status = 0;
	uint8_t ratio=0;
	uint8_t Recv_Data_Buff[UART0_RX_LEN];
	uint8_t Recv_Comm1[] = "1_Xtalk";
	uint8_t Recv_Comm2[] = "2_Offset";
	uint8_t Recv_Comm3[] = "3_CGRatio";
	uint8_t Recv_Comm4[] = "4_Ranging";
	uint8_t Recv_Comm8[] = "8_Reset";
	uint8_t Recv_Comm9[] = "9_Debug";
	
	memcpy(Recv_Data_Buff, uart0_RX_buff, sizeof(Recv_Data_Buff));
	if(memcmp(Recv_Data_Buff, Recv_Comm1, sizeof(Recv_Comm1)-1) == 0)
	{
		VI530x_Calibration_Xtalk();
	}
	
	if(memcmp(Recv_Data_Buff, Recv_Comm2, sizeof(Recv_Comm2)-1) == 0)
	{
		VI530x_Calibration_Offset();
	}
	
	if(memcmp(Recv_Data_Buff, Recv_Comm3, sizeof(Recv_Comm3)-1) == 0)
	{
		ratio = VI530x_Calculate_Xtalk_Ratio(result.correction_tof, result.xtalk_count, result.noise, result.confidence, result.peak);
		ret =	VI530x_Calibration_Xtalk_Ratio(ratio);
		if(!ret)
		{
			printf("VI530x Xtalk Ratio is %d!\r\n",ratio);
		}
		else
		{
			printf("VI530x Xtalk Ratio Configer Error!\r\n");
		}
	}
	
	if(memcmp(Recv_Data_Buff, Recv_Comm4, sizeof(Recv_Comm4)-1) == 0)
	{

		if(ranging_status)
		{
			ret = VI530x_Start_Continue_Ranging_Cmd();
			ranging_status = 0;
		}
		else
		{
			ret = VI530x_Stop_Continue_Ranging_Cmd();
			ranging_status = 1;
		}
	}
	
	if(memcmp(Recv_Data_Buff, Recv_Comm8, sizeof(Recv_Comm8)-1) == 0)
	{
		printf("MCU Reset!");
		while(1);
	}
	if(memcmp(Recv_Data_Buff, Recv_Comm9, sizeof(Recv_Comm9)-1) == 0)
	{
		Debug_Output_State = !Debug_Output_State;
	}
	uart0_RX_cnt = 0;
}

/***********************************************************************************************************************
* Function Name: EVB_UART_Interrupt_Callback
* Description  : EVB板UART中断Callback函数
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void EVB_UART_Interrupt_Callback(void)
{  
		uint16_t tmp = 0;
	
		dma_channel_disable(DMA_CH2);
		tmp = dma_transfer_number_get(DMA_CH2);
		uart0_RX_cnt = UART0_RX_LEN - tmp;

		dma_transfer_number_config(DMA_CH2, UART0_RX_LEN);
//		printf("%d\r\n",uart0_RX_cnt);
		dma_channel_enable(DMA_CH2); 
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM, USART_FLAG_TBE));

    return ch;
}








