#include "usart2.h"

void TIM3_IRQHandler(void);
void USART2_IRQHandler(void);
void TIM3_Set(u8 sta);
void TIM3_Init(u16 arr,u16 psc);

uint8_t USART2_RX_BUF[USART2_MAX_RECV_LEN]; 				
uint16_t USART2_RX_STA=0;  

//setup usart2
//baudrate: assigned baudrate
void Usart2_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	//Rx
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_3;	// USART2 Rx (PA.3)								
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//Tx
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_2;	// USART2 Tx (PA.2)
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//Parameters for USART2
	USART_InitStructure.USART_BaudRate 				= baudrate;
	USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 				= USART_StopBits_1;
	USART_InitStructure.USART_Parity 				= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode 					= USART_Mode_Rx | USART_Mode_Tx;   
	// Configure USART2
	USART_Init(USART2, &USART_InitStructure);		
	USART_Cmd(USART2, ENABLE);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)	;

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	    	
	/* Enable the USART2 Interrupt */																																												
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;																																												
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;																																												
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;																																												
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;																																												
	NVIC_Init(&NVIC_InitStructure);	

	TIM3_Init(99,7199);		//set 10ms interrupt
	USART2_RX_STA=0;		//reset counter
	TIM3_Set(0);			//disable timer	
}

//usart2 interrupt function
void USART2_IRQHandler(void)
{
	uint8_t res;	    
	if(USART2->SR&(1<<5))//if rx data
	{	 
		res=USART2->DR; 			 
		if(USART2_RX_STA<USART2_MAX_RECV_LEN)	//can get more data	
		{
			TIM3->CNT=0;	//reset timer counter       					
			if(USART2_RX_STA==0)TIM3_Set(1);	//start timer counter	 
			USART2_RX_BUF[USART2_RX_STA++]=res;	//put data into buffer
		}else 
		{
			USART2_RX_STA|=1<<15;	//set data receive complete flag					
		} 		 
	}  											 
}  

//­timer3 interrupt function	    
void TIM3_IRQHandler(void)
{ 	
	if(TIM3->SR&0X01)
	{	 			   
		USART2_RX_STA|=1<<15;	//set usart2 rx complete flag
		TIM3->SR&=~(1<<0);				   
		TIM3_Set(0);			//disable timer 
	}	    
}

//Enable/Disable timer3
void TIM3_Set(u8 sta)
{
	if(sta)
	{
    	TIM3->CNT=0;         
		TIM3->CR1|=1<<0;     
	}else TIM3->CR1&=~(1<<0);	   
}

//setup timer3	 
void TIM3_Init(u16 arr,u16 psc)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//Enable the Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	//Timer Setting 
    TIM_TimeBaseStructure.TIM_Period =arr;			
  	TIM_TimeBaseStructure.TIM_Prescaler = psc;		
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
 	TIM_ARRPreloadConfig(TIM3, DISABLE);
 	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	/* Enable the TIM3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);									 
}