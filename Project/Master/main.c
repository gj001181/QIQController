#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "led.h" 
#include "myprintf.h"
#include "usart2.h"
#include "pwradc.h"
#include "pwm.h"
#include "max6675.h"
#include "systick.h"
#include "can.h"
#include "button.h"

void CANBuf_FillInThrottleCuteMsg(void);
void CANBuf_FillInThrottleCuteReleaseMsg(void);

int main(void)
{
	SysTick_Config(SYSTICK_PRESCALER);
	LED_Init();
	//Usart2_Init(115200);
	//Myprintf_Init(0x00,myputc);
	//PWRADC_Init();
	//PWM_Init(65535,71); //1us as unit
	//MAX6675_Init();
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_8tq,CAN_BS2_7tq,5,CAN_Mode_Normal);//CAN初始化环回模式,波特率450Kbps
	Button_Init();

	while (1) 
	{
		if(SYS_CNT>=SYS_LOOP)
		{
			//Reset loop status
			SYS_CNT = 0;
			//Check if button pressed
			if(IF_BTN_PRESSED()==0)
			{
				//Change canbuf data
				CANBuf_FillInThrottleCuteMsg();
				//Send out Throttle Cut command
				Can_Send_Msg(canbuf,8);
				//waiting for user release button
				while(IF_BTN_PRESSED()==0);
				//Change canbuf data
				CANBuf_FillInThrottleCuteReleaseMsg();
				//Send out Throttle Cut command
				Can_Send_Msg(canbuf,8);
				//waiting for user release button			
			}

			//LED Toggle
			if(GPIO_ReadOutputDataBit(LED_Port,LED_Pin))
				{LED_OFF();}
			else
				{LED_ON();}
		}

	}
}

void CANBuf_FillInThrottleCuteMsg(void)
{
	//CAN Bus Data fill in
	canbuf[0] = 0x21;	canbuf[1] = 0x22;	canbuf[2] = 0x23;	canbuf[3] = 0x24;	
	canbuf[4] = 0x31;	canbuf[5] = 0x41;	canbuf[6] = 0x51;	canbuf[7] = 0x61;	
}

void CANBuf_FillInThrottleCuteReleaseMsg(void)
{
	//CAN Bus Data fill in
	canbuf[0] = 0x51;	canbuf[1] = 0x52;	canbuf[2] = 0x53;	canbuf[3] = 0x54;	
	canbuf[4] = 0x51;	canbuf[5] = 0x61;	canbuf[6] = 0x71;	canbuf[7] = 0x81;		
}