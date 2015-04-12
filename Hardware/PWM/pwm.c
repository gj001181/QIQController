#include "pwm.h"
#include "myprintf.h"
//Timer4, Channel 1 for Tachometer, capture rising  for total cycle
u8 TIM4CH1_CAPTURE_STA = 0;	//通道1輸入捕獲標誌，高兩位元做捕獲標誌，低6位元做溢出標誌		
u16 TIM4CH1_CAPTURE_1st_UPVAL;
u16 TIM4CH1_CAPTURE_2st_UPVAL;
u32 tempup1 = 0;	//捕獲總高電平的時間, RPM input channel
u32 tim4_T1;
//Timer1, Channel 2 for PWM_Input from master flight control kernel
u8 TIM1CH2_CAPTURE_STA = 0;	//通道1輸入捕獲標誌，高兩位元做捕獲標誌，低6位元做溢出標誌		
u16 TIM1CH2_CAPTURE_1st_UPVAL;
u16 TIM1CH2_CAPTURE_2st_UPVAL;
u32 tempup2 = 0;	//捕獲總高電平的時間, RPM input channel
u32 tim1_T1;



int  rc_pwm ;//input duty cycle from RC controller
int32_t engine_period = 0;
int32_t engine_rpm = 0;

//Function protoype
void TIM2_Init(u16 arr,u16 psc);
void TIM2_Set(u8 sta);
void TIM2_IRQHandler(void);
void PWM_Init(u16 arr, u16 psc);
void TIM4_IRQHandler(void);
void TIM1_CC_IRQHandler(void);

//Timer2 Initialization
//For monitor Tachometer timeout	 
void TIM2_Init(u16 arr,u16 psc)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  //Enable the Clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  //Timer Setting 
  TIM_TimeBaseStructure.TIM_Period =arr;			
  TIM_TimeBaseStructure.TIM_Prescaler = psc;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ARRPreloadConfig(TIM2, DISABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
	/* Enable the TIM2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);									 
}

//Enable or Disable Timer 2
//sta: 0=Off, 1=On
void TIM2_Set(u8 sta)
{
	if(sta)
	{
    	TIM2->CNT=0;         //Clear counter
			TIM2->CR1|=1<<0;     //Enable Timer 2
	}else TIM2->CR1&=~(1<<0);//Disable Timer 2  
}

//Timer 2 ISR	    
void TIM2_IRQHandler(void)
{ 	
	if(TIM2->SR&0X01) //Time up interrupt 
	{	 			   
		TIM2->SR&=~(1<<0);	//Clear interrupt pending bit		   
		TIM2_Set(0);		//Disable Timer 2
		TIM4CH1_CAPTURE_STA = 0; //clear Tachometer variables
		TIM4CH1_CAPTURE_1st_UPVAL = 0;
		TIM4CH1_CAPTURE_2st_UPVAL = 0;
		engine_rpm = 0;
		tempup1 = 0;
	}	    
}


TIM_OCInitTypeDef TIM1_OCInitStructure;

//計時器4配置
void PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ICInitTypeDef TIM4_ICInitStructure;
    TIM_ICInitTypeDef TIM1_ICInitStructure;	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能TIM4時鐘
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_AFIO, ENABLE);	//使能TIM1時鐘
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);  //使能GPIO時鐘
	//RPM Input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  //PB6, TIM4_CH1 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//Throttle Input(Master_PWM_IN)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  //PA9, TIM1_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//Throttle output to servo
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;  //PA8, TIM1_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//初始化計時器4 TIM4	 
	TIM_TimeBaseStructure.TIM_Period = arr; //設定計數器自動重裝值 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	//預分頻器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上計數模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根據TIM_TimeBaseInitStruct中指定的參數初始化TIMx的時間基數單位
	//RPM Channel
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕獲
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置輸入分頻,不分頻 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置輸入濾波器 不濾波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//初始化計時器1 TIM1	 
	TIM_TimeBaseStructure.TIM_Period = 20500-1; //設定計數器自動重裝值 
	TIM_TimeBaseStructure.TIM_Prescaler = 71; 	//預分頻器 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上計數模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根據TIM_TimeBaseInitStruct中指定的參數初始化TIMx的時間基數單位
    //Throttle Output Channel
    TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM1_OCInitStructure.TIM_Pulse = 1100;
    TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(TIM1, &TIM1_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_ARRPreloadConfig(TIM1, ENABLE);	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	//Throttle Input Channel (form master)
	TIM_ICStructInit(&TIM1_ICInitStructure);
    TIM1_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //由于先检测低电平的同步头，所以这里下降沿触发
    TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM1_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInit(TIM1, &TIM1_ICInitStructure);

	//中斷分組初始化
	//RPM Input Channel
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中斷
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占優先順序1級
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //從優先順序0級
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);   //根據NVIC_InitStruct中指定的參數初始化外設NVIC寄存器 
	TIM_ITConfig(TIM4, TIM_IT_CC1,ENABLE);   //不允許更新中斷，允許CC1IE interrupt capture	
	//PWM Input Channel
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;  //TIM1中斷
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占優先順序1級
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //從優先順序0級
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);   //根據NVIC_InitStruct中指定的參數初始化外設NVIC寄存器 
    TIM_ClearFlag(TIM1,TIM_FLAG_CC2 );
    TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE); 
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);	


	TIM_Cmd(TIM4, ENABLE); 		//使能計時器4
	TIM_Cmd(TIM1, ENABLE); 		//使能計時器1

	TIM2_Init(60000-1,71);			//rpm under 1000 will be ignored due to this monitor timer


}

//計時器4中斷服務程式
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET) 		//捕獲1發生捕獲事件
	{
		if (TIM4CH1_CAPTURE_STA & 0X40)	//Capture second rising edge
		{
			TIM4CH1_CAPTURE_2st_UPVAL = TIM_GetCapture1(TIM4);//記錄下此時的計時器計數值
			if (TIM4CH1_CAPTURE_2st_UPVAL < TIM4CH1_CAPTURE_1st_UPVAL)
			{
				tim4_T1 = 65535;
			}
			else	tim4_T1 = 0;
			
			tempup1 = TIM4CH1_CAPTURE_2st_UPVAL - TIM4CH1_CAPTURE_1st_UPVAL + tim4_T1;	

			engine_period = tempup1;
			tempup1 = 0;
	    	if(engine_period!=0)
	    	{
	    		tempup1 = (60000000/engine_period);
	    		if((tempup1<=10000)&&(tempup1>=1000))
	    		{
	    			engine_rpm = tempup1;
	    		}
	    	}
	    	else 
	    	engine_rpm = 0;

			TIM4CH1_CAPTURE_STA = 0;	//捕獲標誌位元清零	 
			tempup1 = 0;
			TIM2_Set(0);	//Disable Timer 2 			
			TIM2_Set(1);	//Enable Timer 2 
		}
		else //第一次捕獲到上升沿，記錄此時的計時器計數值
		{
			TIM4CH1_CAPTURE_1st_UPVAL = TIM_GetCapture1(TIM4);		//獲取上升沿資料
			TIM4CH1_CAPTURE_STA |= 0X40;		//Get first rising edge				
			TIM2_Set(0);	//Disable Timer 2 			
			TIM2_Set(1);	//Enable Timer 2 
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1); 		//清除中斷標誌位元
	}
}

//計時器1中斷服務程式
void TIM1_CC_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET) 		//TIM1, Channel 2 interrupt
	{
		if (TIM1CH2_CAPTURE_STA & 0X40)	//Capture second rising edge
		{
			TIM1CH2_CAPTURE_2st_UPVAL = TIM_GetCapture2(TIM1);//記錄下此時的計時器計數值
			if (TIM1CH2_CAPTURE_2st_UPVAL < TIM1CH2_CAPTURE_1st_UPVAL)
			{
				tim1_T1 = 20500;
			}
			else	tim1_T1 = 0;
			
			tempup2 = TIM1CH2_CAPTURE_2st_UPVAL - TIM1CH2_CAPTURE_1st_UPVAL + tim1_T1;	
			rc_pwm = tempup2;
			TIM1CH2_CAPTURE_STA = 0;	//捕獲標誌位元清零	 
			tempup2 = 0; 
			TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Rising); //設置為上升沿捕獲
		}
		else //第一次捕獲到上升沿，記錄此時的計時器計數值
		{
			TIM1CH2_CAPTURE_1st_UPVAL = TIM_GetCapture2(TIM1);		//獲取上升沿資料
			TIM1CH2_CAPTURE_STA |= 0X40;		//Get first rising edge	
			TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Falling);//設置為下降沿捕獲			 
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2); 		//清除中斷標誌位元
	}
}

//Throttle for servo control
void Throttle_Output(uint16_t dutycyle)
{
    TIM1->CCR1 = dutycyle;
}