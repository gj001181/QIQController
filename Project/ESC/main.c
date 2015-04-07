//firmware verison: v1.02
//useing PWM Input to control the engine under MODE_RPMCTRL  
//contorl loop frequenct = 50Hz
//PWM for throttle control  = 50Hz (20.5ms)
//Servo reversed version
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
#include "string.h"
#include "stdlib.h"
#include "i2c.h"
#include "24cxx.h"
#include "ChiAhsuN_F1.h"

void Delay(vu32 nCount);
void Delaynus(vu32 nus);

void Delay(vu32 nCount)
{
         for(; nCount != 0; nCount--);
}


void Delaynus(vu32 nus)
{
        u8 nCount ;
        while(nus--)
        {
                for(nCount = 6 ; nCount != 0; nCount--);
        }
}



//Command Type Definition
#define MODE_RC 11
#define MODE_MCU 1
#define MODE_RPMCTRL	17
#define MODE_KAI	23
#define MODE_THROCUT	25
#define CMD_SETTHROTTLE	2
#define CMD_SETENDH 3
#define CMD_SETENDL 4
#define CMD_SETIDLE 5
#define CMD_ENDHSET 6
#define CMD_ENDLSET 7
#define CMD_IDLESET 8
#define CMD_MODERC	9
#define CMD_MODEMCU 10
#define CMD_MODERPMCTRL	18
#define CMD_MODEKAI 24
#define CMD_ECHOCONGIF 12
#define CMD_MODETHROCUT 26
#define CMD_LOGENABLE 13
#define CMD_LOGDISABLE 14
#define CMD_TESTABORT 15
#define CMD_RPMTHROTTLETEST 16
#define CMD_SETKP 19
#define CMD_SETKI 20
#define CMD_SETKD 21
#define CMD_SETRPMTARGET 22
//EEPROM Address Definition
#define ADDR_SERVOH 0
#define ADDR_SERVOL 2
#define ADDR_SERVOIDLE 4
#define ADDR_KP	6
#define ADDR_KI 8
#define ADDR_KD 10
#define ADDR_RPM_H 12
#define ADDR_RPM_L 14
//Throttle PWM_CMD Limitation 
#define THROTTLE_CMD_H  1930
#define THROTTLE_CMD_L	1107
#define THROTTLE_FULL_RANGE	(THROTTLE_CMD_H-THROTTLE_CMD_L)

uint8_t Status = 1;
uint8_t LogEnable = 0; 			   //1=enable, 0=disable
uint8_t SYS_MODE = MODE_MCU; 	   //system mode
uint16_t ServoEndPointHigh = 1700; //sevo end point (high)
uint16_t ServoEndPointLow = 1100;   //servo end point (low)
uint16_t ServoIdlePoint = 1600;	   //servo idle point
uint16_t ThrottlePwm = 1600; 	   //current throttle output via MCU under MCU mode
uint16_t ThrottlePwmTempt = 1600;  //temporary throttle output command
int16_t Throttle_percent = 0;	   //throttle percentage, 1000 = 100%, 0 = 0%
uint16_t TargetRPM = 2000;		   //target RPM of RPM controller
uint16_t TargetRPM_H = 8000;	   //highest RPM system can reach
uint16_t TargetRPM_L = 3500;	   //lowest RPM system can reach
uint16_t Kp = 0;				   //propotional gain for RPM ocntrol loop  (int16_t)
uint16_t Kd = 0;				   //derivative gain for RPM ocntrol loop (int16_t)
uint16_t Ki = 0;				   //integral gain for RPM ocntrol loop (int16_t)
float	 Kp_f = 0.01;			   //propotional gain for RPM ocntrol loop 
float	 Ki_f = 0.01;			   //derivative gain for RPM ocntrol loop 
float	 Kd_f = 0.01;			   //integral gain for RPM ocntrol loop 
uint16_t VarPID_Tempt = 0;		   //temporary PID value
uint16_t VarRPM_Tempt = 0;		   //temporary target RPM
uint16_t RPM_Last = 0;			   //RPM at t-1
int Err_Last = 0;			   //RPM error at t-1
int Err_Now = 0;			   //RPM error at t
int Err_Det = 0;			   //RPM error difference at t
int Err_Sum = 0;			   //RPM error integral
uint16_t Timer = 0; 

uint8_t SerialDataAnalysis(void);
uint8_t CMD_Analysis(char* cmd_buf, uint16_t* cmdvalue);
uint8_t RpmThrottleTest(uint16_t StartThrottle, uint16_t EndThrottle, uint16_t Unitstep, uint16_t Interval, uint16_t delay_s, uint16_t retry); 
uint8_t GetTestParameters(char* cmd_buf, uint16_t* startthrottle,uint16_t* endtthrottle,uint16_t* unitstep,uint16_t* interval, uint16_t* delay_s, uint16_t* retry);
uint8_t GetPIDParameters(char* cmd_buf, uint16_t* var_pid);
uint8_t GetTargetRPM(char* cmd_buf, uint16_t* var_rpm);
uint16_t ServoLinear(int16_t throttle_percentage);

int main(void)
{   
	
	uint8_t CMD = 0; //data CMD type from serial port
	uint16_t i=0;
	uint16_t ValueTemp = 0; //command value from serial port
	uint16_t startthrottle, endtthrottle, unitstep, interval,delay_s,retry;

	SysTick_Config(SYSTICK_PRESCALER);
	LED_Init();
	Usart2_Init(115200);
	Myprintf_Init(0x00,myputc);
	PWRADC_Init();
	PWM_Init(65535,71); //1us as unit
	MAX6675_Init();
	AT24CXX_Init();
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS1_4tq,CAN_BS2_4tq,4,CAN_Mode_Normal);//CAN初始化环回模式,波特率450Kbps

	//check if EEPRON AT24C02 is valiable
	while(AT24CXX_Check())
	{
		LED_ON();
		while (USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET) ;
		USART2->DR=0xAA;	
	}
	//loading system configuration from EEPROM
	//End-Point (High)
	ValueTemp = (AT24CXX_ReadOneByte(ADDR_SERVOH)<<8)|(AT24CXX_ReadOneByte(ADDR_SERVOH+1));
	if ((ValueTemp>850)&&(ValueTemp<2000))	ServoEndPointHigh = ValueTemp;	
	//End-Point (Low)
	ValueTemp = (AT24CXX_ReadOneByte(ADDR_SERVOL)<<8)|(AT24CXX_ReadOneByte(ADDR_SERVOL+1));
	if ((ValueTemp>850)&&(ValueTemp<2000))	ServoEndPointLow = ValueTemp;
	//Idle Point
	ValueTemp = (AT24CXX_ReadOneByte(ADDR_SERVOIDLE)<<8)|(AT24CXX_ReadOneByte(ADDR_SERVOIDLE+1));
	if ((ValueTemp>850)&&(ValueTemp<2000))	ServoIdlePoint = ValueTemp;	
	//set current throttle at idle
	ThrottlePwm = ServoIdlePoint;
	//Kp
	Kp = (AT24CXX_ReadOneByte(ADDR_KP)<<8)|(AT24CXX_ReadOneByte(ADDR_KP+1));	
	//Ki
	Ki = (AT24CXX_ReadOneByte(ADDR_KI)<<8)|(AT24CXX_ReadOneByte(ADDR_KI+1));	
	//Kd
	Kd = (AT24CXX_ReadOneByte(ADDR_KD)<<8)|(AT24CXX_ReadOneByte(ADDR_KD+1));
	//Highest RPM	
	//TargetRPM_H = (AT24CXX_ReadOneByte(ADDR_RPM_H)<<8)|(AT24CXX_ReadOneByte(ADDR_RPM_H+1));	
	//Lowest RPM	
	//TargetRPM_H = (AT24CXX_ReadOneByte(ADDR_RPM_L)<<8)|(AT24CXX_ReadOneByte(ADDR_RPM_L+1));		

	//Print out system configuration
	my_printf("\r\n============ESC System Configuration============");
	my_printf("\r\nSystem Loop Frequency(Hz):%d",LOOP_FREQ);	
	my_printf("\r\nServo Reversed end-point(High):%d",ServoEndPointHigh);
	my_printf("\r\nServo Reversed end-point(Low):%d",ServoEndPointLow);
	my_printf("\r\nServo Reversed idle-point:%d",ServoIdlePoint);
	//my_printf("\r\nRPM Limitation (High):%d",TargetRPM_H);
	//my_printf("\r\nRPM Limitation (Low):%d",TargetRPM_L);
	if(SYS_MODE==MODE_RC)
		my_printf("\r\nCurrent Mode:RC Mode(Bypass)");
	else if(SYS_MODE==MODE_MCU)
		my_printf("\r\nCurrent Mode:MCU Mode");
	else if(SYS_MODE==MODE_KAI)
		my_printf("\r\nCurrent Mode:KAI Mode");
	else
		my_printf("\r\nCurrent Mode:RPM Control Mode");
	my_printf("\r\nCurrent RC Imput PWM DutyCycle:%d",rc_pwm);
	my_printf("\r\nCurrent Throttle Output DutyCycle:%d",ThrottlePwm);
	my_printf("\r\nKp:%d, ki:%d, kd:%d",Kp,Ki,Kd);
	my_printf("\r\n=============================================");	
	while (1) 
	{

		// if(SYS_CNT>=SYS_LOOP)
		// {

		// 	Status = engcommand(4);
		// 	if(Status==0)
		// 	{
		// 		SYS_MODE = MODE_THROCUT;
		// 	}
		// 	else if(Status==2)
		// 	{
  //               SYS_MODE = MODE_RPMCTRL;

		// 	}	
		// 	else if(Status==3)
		// 	{
  //               SYS_MODE = MODE_RC;

		// 	}
		// 	else
		// 	{
		// 		SYS_MODE = MODE_MCU;


		// 	}	
            
					//my_printf("\r\n22222");	
			//Reset loop1 status
			SYS_CNT = 0;
			//Check if MCU get CMD
			if((USART2_RX_STA>>15)==1) 
			{
				CMD = SerialDataAnalysis();
				switch(CMD)
				{
     //                case CMD_THROTTLERANGE:
     //                  if(SYS_MODE==MODE_RPMCTRL)
					// 		{	
					// 			//Get command value
					// 			if(CMD_Analysis((char*)USART2_RX_BUF,&ValueTemp))
					// 			{
					// 				if(ValueTemp==1000)
     //                                 Throttle_percent = 0;
					// 	            else if(ValueTemp==1500)
					// 	             Throttle_percent = 500;
					// 	            else if(ValueTemp==2000)  
					// 	             Throttle_percent = 1000;
     //                                else break;
					// 			}
					// 		}
					// break;
							

					case CMD_SETTHROTTLE:  
							if(SYS_MODE==MODE_MCU)
							{	
								//Get command value
								if(CMD_Analysis((char*)USART2_RX_BUF,&ValueTemp))
								{
									if(ValueTemp>ServoEndPointHigh)
										ThrottlePwm = ServoEndPointHigh;
									else if (ValueTemp<ServoEndPointLow)
										ThrottlePwm = ServoEndPointLow;
									else ThrottlePwm = ValueTemp;
									my_printf("\r\nThrottle set:%d",ThrottlePwm);
								}
							}
					break;

					case CMD_SETENDH:  
							//Get command value
							if(CMD_Analysis((char*)USART2_RX_BUF,&ValueTemp))
							{
								ServoEndPointHigh = ValueTemp;
								//record value into EEPROM
								AT24CXX_WriteOneByte(ADDR_SERVOH,(ServoEndPointHigh>>8)&0xFF);
								AT24CXX_WriteOneByte(ADDR_SERVOH+1,ServoEndPointHigh);
								//echo for confirm
								my_printf("\r\nServo Reversed end-point(High) set:%d",ServoEndPointHigh);
							}													
					break;

					case CMD_SETENDL:  
							//Get command value
							if(CMD_Analysis((char*)USART2_RX_BUF,&ValueTemp))
							{
								ServoEndPointLow = ValueTemp;
								//record value into EEPROM
								AT24CXX_WriteOneByte(ADDR_SERVOL,(ServoEndPointLow>>8)&0xFF);
								AT24CXX_WriteOneByte(ADDR_SERVOL+1,ServoEndPointLow);
								//echo for confirm
								my_printf("\r\nServo Reversed end-point(Low) set:%d",ServoEndPointLow);								
							}								
					break;

					case CMD_SETIDLE:  
							//Get command value
							if(CMD_Analysis((char*)USART2_RX_BUF,&ValueTemp))
							{
								ServoIdlePoint = ValueTemp;
								//record value into EEPROM
								AT24CXX_WriteOneByte(ADDR_SERVOIDLE,(ServoIdlePoint>>8)&0xFF);
								AT24CXX_WriteOneByte(ADDR_SERVOIDLE+1,ServoIdlePoint);
								//echo for confirm
								my_printf("\r\nServo Reversed idle-point(Low) set:%d",ServoIdlePoint);	
							}						
					break;
					case CMD_ENDHSET: 
							ServoEndPointHigh = rc_pwm;
							//record value into EEPROM
							AT24CXX_WriteOneByte(ADDR_SERVOH,(rc_pwm>>8)&0xFF);
							AT24CXX_WriteOneByte(ADDR_SERVOH+1,rc_pwm);	
							//echo for confirm
							my_printf("\r\nServo Reversed end-point(High) set:%d",ServoEndPointHigh);							 
					break;

					case CMD_ENDLSET:  
							ServoEndPointLow = rc_pwm;
							//record value into EEPROM
							AT24CXX_WriteOneByte(ADDR_SERVOL,(rc_pwm>>8)&0xFF);
							AT24CXX_WriteOneByte(ADDR_SERVOL+1,rc_pwm);
							//echo for confirm
							my_printf("\r\nServo Reversed end-point(Low) set:%d",ServoEndPointLow);													
					break;

					case CMD_IDLESET: 
							ServoIdlePoint = rc_pwm;
							//record value into EEPROM
							AT24CXX_WriteOneByte(ADDR_SERVOIDLE,(rc_pwm>>8)&0xFF);
							AT24CXX_WriteOneByte(ADDR_SERVOIDLE+1,rc_pwm); 
							//echo for confirm
							my_printf("\r\nServo Reversed idle-point(Low) set:%d",ServoIdlePoint);														
					break;

					case CMD_MODETHROCUT: 
							SYS_MODE = MODE_THROCUT;
							my_printf("\r\nSystem Mode: CUT Mode");
							ThrottlePwm = ServoEndPointHigh;

								
							Throttle_Output(ThrottlePwm);

					break;

					case CMD_MODERC:  
							//change system mode into RC Mode
							SYS_MODE = MODE_RC;
							//echo for confirm
							my_printf("\r\nSystem Mode: RC Mode");
							//set throttle into idle
							ThrottlePwm = ServoIdlePoint;
							Throttle_Output(ThrottlePwm);	
					break;

					case CMD_MODEKAI:  
							//change system mode into MCU Mode
							SYS_MODE = MODE_KAI;	
							//echo for confirm
							my_printf("\r\nSystem Mode: KAI Mode");
							ThrottlePwm = ServoIdlePoint;

								
							Throttle_Output(ThrottlePwm);
					        

							//set throttle into idle
							
																	
					break;

					case CMD_MODEMCU:  
							//change system mode into MCU Mode
							SYS_MODE = MODE_MCU;	
							//echo for confirm
							my_printf("\r\nSystem Mode: MCU Mode");	
							//set throttle into idle
							ThrottlePwm = ServoIdlePoint;
							Throttle_Output(ThrottlePwm);											
					break;

					case CMD_MODERPMCTRL:
							//change system mode into RPM Control Mode
							SYS_MODE = MODE_RPMCTRL;
							//echo for confirm
							my_printf("\r\nSystem Mode: RPM Control Mode");	
							//
							//set throttle into idle
							ThrottlePwm = ServoIdlePoint;
							Throttle_Output(ThrottlePwm);		
					break;

					case CMD_ECHOCONGIF:  
							//Echo current configuration from EEPROM data
							my_printf("\r\n============ESC System Configuration============");
							my_printf("\r\nSystem Loop Frequency(Hz):%d",LOOP_FREQ);	
							my_printf("\r\nServo Reversed end-point(High):%d",ServoEndPointHigh);
							my_printf("\r\nServo Reversed end-point(Low):%d",ServoEndPointLow);
							my_printf("\r\nServo Reversed idle-point:%d",ServoIdlePoint);
							if(SYS_MODE==MODE_RC)
								my_printf("\r\nCurrent Mode:RC Mode(Bypass)");
							else if(SYS_MODE==MODE_MCU)
								my_printf("\r\nCurrent Mode:MCU Mode");
							else if(SYS_MODE==MODE_KAI)
								my_printf("\r\nCurrent Mode:KAI Mode");
							else
								{
									my_printf("\r\nCurrent Mode:RPM Control Mode");
									my_printf("\r\nCurrent TargetRPM:%d",TargetRPM);
								}
							my_printf("\r\nCurrent RC Imput PWM DutyCycle:%d",rc_pwm);
							my_printf("\r\nCurrent Throttle Output DutyCycle:%d",ThrottlePwm);
							my_printf("\r\nKp:%d, ki:%d, kd:%d",Kp,Ki,Kd);
							my_printf("\r\n=============================================");				
					break;

					case CMD_LOGENABLE:
							//Enable data log
							LogEnable = 1;
					break;

					case CMD_LOGDISABLE:
							//Disable data log
							LogEnable = 0;
					break;

					case CMD_RPMTHROTTLETEST:
							//determine if system is under MCU mode
							if(SYS_MODE==MODE_MCU)
							{
								//get test parameters
								if(GetTestParameters((char*)USART2_RX_BUF,&startthrottle,&endtthrottle,&unitstep,&interval,&delay_s,&retry))	break; //data error
								else
								{
									//start test
									RpmThrottleTest(startthrottle,endtthrottle,unitstep,interval,delay_s,retry);
								}
							}
							else
							{
								//print out error message
								my_printf("\r\nCommand only valiable under MCU Mode.");
							}

					break;

					case CMD_SETKP:
						//get kp value
						if(GetPIDParameters((char*)USART2_RX_BUF,&VarPID_Tempt))	break; //command error
						Kp = VarPID_Tempt;
						//echo for confirm
						my_printf("\r\nKp set:%d",Kp);
						//Record into EEPROM
						AT24CXX_WriteOneByte(ADDR_KP,(Kp>>8)&0xFF);
						AT24CXX_WriteOneByte(ADDR_KP+1,Kp); 							
					break;

					case CMD_SETKI:
						//get ki value
						if(GetPIDParameters((char*)USART2_RX_BUF,&VarPID_Tempt))	break; //command error
						Ki = VarPID_Tempt;
						//echo for confirm
						my_printf("\r\nKi set:%d",Ki);
						//Record into EEPROM
						AT24CXX_WriteOneByte(ADDR_KI,(Ki>>8)&0xFF);
						AT24CXX_WriteOneByte(ADDR_KI+1,Ki); 	
					break;

					case CMD_SETKD:
						//get kd value
						if(GetPIDParameters((char*)USART2_RX_BUF,&VarPID_Tempt))	break; //command error
						Kd = VarPID_Tempt;
						//echo for confirm
						my_printf("\r\nKd set:%d",Kd);
						//Record into EEPROM
						AT24CXX_WriteOneByte(ADDR_KD,(Kd>>8)&0xFF);
						AT24CXX_WriteOneByte(ADDR_KD+1,Kd); 	
					break;

					case CMD_SETRPMTARGET:
						//get target value
						if(GetTargetRPM((char*)USART2_RX_BUF,&VarRPM_Tempt))	break; //command error
						//limit target RPM
						if(VarRPM_Tempt>8000)	VarRPM_Tempt = 8000; 
						if(VarRPM_Tempt<1500)	VarRPM_Tempt = 1500;
						TargetRPM = VarRPM_Tempt;
						//echo for confirm
						my_printf("\r\nTarget RPM set:%d",TargetRPM);						
					break;

					default:
					break;
				}
				//clear serial buffer
				for(i=0;i<USART2_MAX_RECV_LEN;i++)	USART2_RX_BUF[i] = 0x00; //clear buffer
				USART2_RX_STA = 0;	//clear usart2 rx flag and counter					
				//clear variables
				ValueTemp = 0;
				CMD = 0;
				startthrottle = 0;
				endtthrottle = 0;
				unitstep = 0;
				interval = 0;

			}
			else
			{
				//Determine which mode the system is now
				//RC Mode
				if(SYS_MODE==MODE_RC)
				{
					if(rc_pwm>ServoEndPointHigh)
						ThrottlePwm = ServoEndPointHigh;
					else if (rc_pwm<ServoEndPointLow)
						ThrottlePwm = ServoEndPointLow;
					else ThrottlePwm = rc_pwm;
					Throttle_Output(ThrottlePwm);
				}
				//MCU Mode
				else if(SYS_MODE==MODE_MCU)
				{
					if(ThrottlePwm>ServoEndPointHigh)
						ThrottlePwm = ServoEndPointHigh;
					else if (ThrottlePwm<ServoEndPointLow)
						ThrottlePwm = ServoEndPointLow;
					Throttle_Output(ThrottlePwm);
				}

				else if(SYS_MODE==MODE_THROCUT)
				{
					while(1)
				  {
					if(SYS_CNT>=SYS_LOOP)
					  {	
						SYS_CNT = 0;
                        Timer = Timer + 1;
					    if(ThrottlePwm>ServoEndPointHigh)
						ThrottlePwm = ServoEndPointHigh;
					    else if (ThrottlePwm<ServoEndPointLow)
						ThrottlePwm = ServoEndPointLow;

					    ThrottlePwm = ServoEndPointHigh;
					    Throttle_Output(ThrottlePwm);
					    my_printf("\r\n%d,%d,%d,%d,%d",engine_rpm,TargetRPM,ThrottlePwm,Engine_Temperature,Timer);
					    if(GPIO_ReadOutputDataBit(LED_Port,LED_Pin))
				        {LED_OFF();}
			            else
				        {LED_ON();}
				        if(Timer==250)
					    {
					    	Timer = 0;
					    	break;
					    }
				
				      }
				   }   
                    SYS_MODE = MODE_MCU;
                    my_printf("\r\nSystem Mode: MCU Mode");	
                    ThrottlePwm = ServoIdlePoint;
					Throttle_Output(ThrottlePwm);
				}

				else if(SYS_MODE==MODE_KAI)
				{
				  
				  ThrottlePwm--;


				   if(ThrottlePwm==ServoEndPointLow)
				   {
				  	while(1)

                    {
				  	  if(SYS_CNT>=SYS_LOOP)
				  	   {
				  		SYS_CNT = 0;
				  		ThrottlePwm++;
				  		if(ThrottlePwm==ServoIdlePoint)
				  		{
				  			break;
				  		}
				  		if(ThrottlePwm>ServoIdlePoint)
						ThrottlePwm = ServoIdlePoint;
					    else if (ThrottlePwm<ServoEndPointLow)
						ThrottlePwm = ServoEndPointLow;
			            Throttle_Output(ThrottlePwm);
			            my_printf("\r\n%d,%d,%d,%d",engine_rpm,TargetRPM,ThrottlePwm,Engine_Temperature);
			            if(GPIO_ReadOutputDataBit(LED_Port,LED_Pin))
				        {LED_OFF();}
			            else
				        {LED_ON();}

				  	   }
				    }
				   }
				   
				   if(ThrottlePwm>ServoIdlePoint)
				   ThrottlePwm = ServoIdlePoint;
				   else if (ThrottlePwm<ServoEndPointLow)
				   ThrottlePwm = ServoEndPointLow;
			       Throttle_Output(ThrottlePwm);	

				  
                
					
				 }

				
				//RPM Control Mode
				else
				{
					//Arrange calculation parameters
					Kp_f = 0.0001*Kp;
					Ki_f = 0.0001*Ki;
					Kd_f = 0.001*Kd;
					//Convert Target RPM form PWM_CMD Input
					if((rc_pwm - THROTTLE_CMD_L)<=0) rc_pwm = THROTTLE_CMD_L;
					if((THROTTLE_CMD_H - rc_pwm)<=0) rc_pwm = THROTTLE_CMD_H;
					TargetRPM = 4000+3500*(THROTTLE_CMD_H-rc_pwm)/THROTTLE_FULL_RANGE;
					//Error calculation
					Err_Now = TargetRPM - engine_rpm;
					Err_Det = Err_Now-Err_Last;
					Err_Sum = Err_Sum + Err_Now;
					//calculationate output throttle percentage
					Throttle_percent = Throttle_percent + Kp_f*Err_Now + Kd_f*Err_Det + Ki_f*Err_Sum;
					if(Throttle_percent>=1000)	Throttle_percent=1000;
					if(Throttle_percent<=0)		Throttle_percent=0;					
					ThrottlePwmTempt = ServoLinear(Throttle_percent);
					my_printf("\r\n%d,%d,%d,%d,%d,%d,%d",TargetRPM,engine_rpm,ThrottlePwmTempt,Throttle_percent,Kp,Kd);
					//Limiter
					if(ThrottlePwmTempt>ServoIdlePoint)
						ThrottlePwm = ServoIdlePoint;
					else if (ThrottlePwmTempt<ServoEndPointLow)
						ThrottlePwm = ServoEndPointLow;
					else
						ThrottlePwm = ThrottlePwmTempt;
					//Output throttle command
					Throttle_Output(ThrottlePwm);
					//finish control part
					Err_Last = Err_Now;
				}

			}


			//output RPM and Throttle status
			if(LogEnable)
			{
				//check if MAX6675 is ready to read out
				if(MAX6675_LoopCNT>=Max6675LoopFactor)
				{
					MAX6675_LoopCNT = 0;
					Read_MAX6675(&Engine_Temperature);
				}
				my_printf("\r\n%d,%d,%d,%d",engine_rpm,TargetRPM,ThrottlePwm,Engine_Temperature);
			}
			//LED Toggle
			if(GPIO_ReadOutputDataBit(LED_Port,LED_Pin))
				{LED_OFF();}
			else
				{LED_ON();}
		}

	}
}


//check data in serial port
//return: CMD type
uint8_t SerialDataAnalysis(void)
{
	uint8_t res = 0;

	if((USART2_RX_STA>>15)==0)	return 0; //no data 
	else
	{
		//find command
		if(strncmp("SETTHROTTLE",(const char *)USART2_RX_BUF,11)==0)	 res=CMD_SETTHROTTLE;
		else if(strncmp("MODE_KAI",(const char *)USART2_RX_BUF,8)==0) res=CMD_MODEKAI;
		else if(strncmp("MODE_THROCUT",(const char *)USART2_RX_BUF,7)==0) res=CMD_MODETHROCUT;
		else if(strncmp("SETENDH",(const char *)USART2_RX_BUF,7)==0) res=CMD_SETENDH;
		else if(strncmp("SETENDL",(const char *)USART2_RX_BUF,7)==0) res=CMD_SETENDL;
		else if(strncmp("SETIDLE",(const char *)USART2_RX_BUF,7)==0) res=CMD_SETIDLE;
		else if(strncmp("ENDHSET",(const char *)USART2_RX_BUF,7)==0) res=CMD_ENDHSET;
		else if(strncmp("ENDLSET",(const char *)USART2_RX_BUF,7)==0) res=CMD_ENDLSET;
		else if(strncmp("IDLESET",(const char *)USART2_RX_BUF,7)==0) res=CMD_IDLESET;
		else if(strncmp("MODE_RC",(const char *)USART2_RX_BUF,7)==0) res=CMD_MODERC;
		else if(strncmp("MODE_MCU",(const char *)USART2_RX_BUF,8)==0) res=CMD_MODEMCU;
		else if(strncmp("ECHO_CONFIG",(const char *)USART2_RX_BUF,11)==0) res=CMD_ECHOCONGIF;
		else if(strncmp("LOG_ENABLE",(const char *)USART2_RX_BUF,10)==0) res=CMD_LOGENABLE;		
		else if(strncmp("LOG_DISABLE",(const char *)USART2_RX_BUF,11)==0) res=CMD_LOGDISABLE;	
		else if(strncmp("TEST_ABORT",(const char *)USART2_RX_BUF,10)==0) res=CMD_TESTABORT;
		else if(strncmp("RPM_THROTTLE_TEST",(const char *)USART2_RX_BUF,17)==0) res=CMD_RPMTHROTTLETEST;
		else if(strncmp("MODE_RPMCTRL",(const char *)USART2_RX_BUF,12)==0) res=CMD_MODERPMCTRL;
		else if(strncmp("SETKP",(const char *)USART2_RX_BUF,5)==0) res=CMD_SETKP;
		else if(strncmp("SETKI",(const char *)USART2_RX_BUF,5)==0) res=CMD_SETKI;
		else if(strncmp("SETKD",(const char *)USART2_RX_BUF,5)==0) res=CMD_SETKD;
		else if(strncmp("SETRPMTARGET",(const char *)USART2_RX_BUF,12)==0) res=CMD_SETRPMTARGET;
		return res;
	}
	return 0;
}


//get the command value from data buffer
//return: 1=data correct, 0=data error
uint8_t CMD_Analysis(char* cmd_buf, uint16_t* cmdvalue)
{
	char tempvalue[5] = {'\0','\0','\0','\0','\0'};
 	char* ptrBuf = 0x00;
 	uint8_t offset = 0;
 	uint8_t digit_counter = 0;

	ptrBuf = cmd_buf;
	while(*ptrBuf!='=')
	{
		offset++; ptrBuf++;
		if(offset>=USART2_MAX_RECV_LEN) return 0; //command error
	}
	ptrBuf++; //point to first digit of IP1
	while(*ptrBuf!='\0')
	{
		tempvalue[digit_counter] = *ptrBuf;	
		digit_counter++;
		ptrBuf++;
		if(digit_counter>5) break; 
	}	
	*cmdvalue = atoi(tempvalue); 
	return 1;
}


//Engine Test for relation between RPM and Throttle
//StartThrottle: throttle start point of test (us)
//EndThrottle: throttle end point of test (us)
//Unitstep: throttle increasing step (us)
//Interval: time gap between each time throttle increasing (ms), must be 10x
//delay_s: delay after each thime the test is finished (second)
//retry: how many times the test will be held
uint8_t RpmThrottleTest(uint16_t StartThrottle, uint16_t EndThrottle, uint16_t Unitstep, uint16_t Interval, uint16_t delay_s, uint16_t retry)
{
	uint16_t loopcnt = 0;
	uint8_t  timecount = 5;
	uint8_t  CMD = 0;
	uint8_t  i=0;
	uint16_t cntfactor = 0;
	uint16_t serialoutput_cnt = 0;
	//Check the validation of input parameters 
	//Throttle 
	if((StartThrottle>ServoEndPointHigh)|(StartThrottle<ServoEndPointLow)|(EndThrottle>ServoEndPointHigh)|(EndThrottle<ServoEndPointLow))	
		{
			my_printf("\r\nInvalid test throttle parameters.");
			return 0;
		}
	//Unit step
	if((Unitstep<1)|(Unitstep>(EndThrottle-StartThrottle)))	
		{
			my_printf("\r\nInvalid test unit step parameters.");
			return 0;
		}
	//Interval
	if(Interval<(1000/LOOP_FREQ))	
		{
			my_printf("\r\nInvalid test Interval parameters.");
			return 0;
		}
	cntfactor = Interval/(1000/LOOP_FREQ); 	
	//10 second counting
	while(timecount)
	{
		if(SYS_CNT>=SYS_LOOP)
		{
			//clear counter
			SYS_CNT = 0;
			//Check if MCU get CMD
			if((USART2_RX_STA>>15)==1) 
			{
				CMD = SerialDataAnalysis();
				switch(CMD)
				{
					case CMD_TESTABORT:  
						my_printf("\r\nTest mission aborted by user.");
						//clear serial buffer
						for(i=0;i<USART2_MAX_RECV_LEN;i++)	USART2_RX_BUF[i] = 0x00; //clear buffer
						USART2_RX_STA = 0;	//clear usart2 rx flag and counter					
						//clear variables
						CMD = 0;
						return 0; //abandon test 
					break;

					default:
					break;
				}
				//clear serial buffer
				for(i=0;i<USART2_MAX_RECV_LEN;i++)	USART2_RX_BUF[i] = 0x00; //clear buffer
				USART2_RX_STA = 0;	//clear usart2 rx flag and counter					
				//clear variables
				CMD = 0;

			}
			//determine if 1sec pass
			if(loopcnt>=LOOP_FREQ)
			{
				loopcnt=0;
				my_printf("\r\nTest will be started in %d second.",timecount);
				timecount--;
			}
			loopcnt++;
		}
	}
	//Test Start
	loopcnt = 0;
	my_printf("\r\nTest Starting....");
	//Set throttle as StartThrottle
	ThrottlePwm = StartThrottle;
	Throttle_Output(ThrottlePwm);
	while(retry)
	{
		if(SYS_CNT>=SYS_LOOP)
		{
			//clear counter
			SYS_CNT = 0;
			//Check if MCU get CMD
			if((USART2_RX_STA>>15)==1) 
			{
				CMD = SerialDataAnalysis();
				switch(CMD)
				{
					case CMD_TESTABORT:  
						my_printf("\r\nTest mission aborted by user.");
						//clear serial buffer
						for(i=0;i<USART2_MAX_RECV_LEN;i++)	USART2_RX_BUF[i] = 0x00; //clear buffer
						USART2_RX_STA = 0;	//clear usart2 rx flag and counter					
						//clear variables
						CMD = 0;
						return 0; //abandon test 
					break;

					default:
					break;
				}
				//clear serial buffer
				for(i=0;i<USART2_MAX_RECV_LEN;i++)	USART2_RX_BUF[i] = 0x00; //clear buffer
				USART2_RX_STA = 0;	//clear usart2 rx flag and counter					
				//clear variables
				CMD = 0;

			}
			//print out RPM, Throttle and Temperature for logging
			//check if MAX6675 is ready to read out
			if(MAX6675_LoopCNT>=Max6675LoopFactor)
			{
				MAX6675_LoopCNT = 0;
				Read_MAX6675(&Engine_Temperature);
			}
			my_printf("\r\n%d, %d, %d, %d",engine_rpm,ThrottlePwm,Engine_Temperature,serialoutput_cnt);
			//LED Toggle
			if(GPIO_ReadOutputDataBit(LED_Port,LED_Pin))
				{LED_OFF();}
			else
				{LED_ON();}
			//determine if time is up to increase throttle
			if(loopcnt>=cntfactor)
			{
				//clear counter
				loopcnt = 0;
				//determine if the test is reach the end
				if((ThrottlePwm>EndThrottle)|((EndThrottle-ThrottlePwm)<Unitstep))	
					{
						//set throttle to start point
						ThrottlePwm = StartThrottle;
						Throttle_Output(ThrottlePwm);						
						//Delay delay_s second
						timecount = delay_s;
						loopcnt = 0;
						while(timecount)
						{
							if(SYS_CNT>=SYS_LOOP)
							{
								//clear counter
								SYS_CNT = 0;
								//Check if MCU get CMD
								if((USART2_RX_STA>>15)==1) 
								{
									CMD = SerialDataAnalysis();
									switch(CMD)
									{
										case CMD_TESTABORT:  
											my_printf("\r\nTest mission aborted by user.");
											//clear serial buffer
											for(i=0;i<USART2_MAX_RECV_LEN;i++)	USART2_RX_BUF[i] = 0x00; //clear buffer
											USART2_RX_STA = 0;	//clear usart2 rx flag and counter					
											//clear variables
											CMD = 0;
											return 0; //abandon test 
										break;

										default:
										break;
									}
									//clear serial buffer
									for(i=0;i<USART2_MAX_RECV_LEN;i++)	USART2_RX_BUF[i] = 0x00; //clear buffer
									USART2_RX_STA = 0;	//clear usart2 rx flag and counter					
									//clear variables
									CMD = 0;

								}
								//print out RPM, Throttle and Temperature for logging
								//check if MAX6675 is ready to read out
								if(MAX6675_LoopCNT>=Max6675LoopFactor)
								{
									MAX6675_LoopCNT = 0;
									Read_MAX6675(&Engine_Temperature);
								}
								my_printf("\r\n%d, %d, %d, %d",engine_rpm,ThrottlePwm,Engine_Temperature,serialoutput_cnt);
								//LED Toggle
								if(GPIO_ReadOutputDataBit(LED_Port,LED_Pin))
									{LED_OFF();}
								else
									{LED_ON();}
								//determine if 1sec pass
								if(loopcnt>=LOOP_FREQ)
									{
										loopcnt=0;
										timecount--;
									}
								loopcnt++;
								serialoutput_cnt++;	
							}						
						}
						//decrease retry time
						retry --;
						if(retry==0)
						{
							my_printf("\r\nTest Finished.");
							//Set throttle into idle
							ThrottlePwm = ServoIdlePoint;
							Throttle_Output(ThrottlePwm);
							//End of the test
							return 0;
						}							
					}
					else
					{
						//increasing throttle with one unit step
						ThrottlePwm += Unitstep;
						if(ThrottlePwm>ServoEndPointHigh)
							ThrottlePwm = ServoEndPointHigh;
						else if (ThrottlePwm<ServoEndPointLow)
							ThrottlePwm = ServoEndPointLow;
						Throttle_Output(ThrottlePwm);	
					}
			}
			loopcnt++;
			serialoutput_cnt++;
		}
	}	
		

	return 1;
}

//cmd_buf: command buffer
//return: 1 = command error, 0 = success
uint8_t GetTestParameters(char* cmd_buf, uint16_t* startthrottle,uint16_t* endtthrottle,uint16_t* unitstep,uint16_t* interval, uint16_t* delay_s, uint16_t* retry)
{
 	char data_temp[7][6] = {{'\0','\0','\0','\0','\0','\0'},{'\0','\0','\0','\0','\0','\0'},{'\0','\0','\0','\0','\0','\0'},{'\0','\0','\0','\0','\0','\0'},{'\0','\0','\0','\0','\0','\0'},{'\0','\0','\0','\0','\0','\0'},{'\0','\0','\0','\0','\0','\0'},};
 	uint16_t tempdata1,tempdata2,tempdata3,tempdata4,tempdata5,tempdata6;
 	char* ptrBuf = 0x00;
 	uint8_t offset = 0;
 	uint8_t dot_counter = 0;
 	uint8_t digit_counter = 0;

	ptrBuf = cmd_buf;
	while(*ptrBuf!='=')
	{
		offset++; ptrBuf++;
		if(offset>=USART2_MAX_RECV_LEN) return 1; //command error
	}
	ptrBuf++; //point to first digit of IP1
	while(*ptrBuf!='\0')
	{
		if(*ptrBuf=='.')  //find '.'
		{
			data_temp[dot_counter][digit_counter] = '\0'; //last one fill in '\0' as string end
			dot_counter++;
			digit_counter = 0;
			ptrBuf++;
			if(dot_counter>5) return 1; //command error
		}
		else //IP information
		{
			data_temp[dot_counter][digit_counter] = *ptrBuf;
			digit_counter++;
			ptrBuf++;
		}
	}
	if (dot_counter<5) 
	{
		my_printf("\r\nCMD Error.");
		return 1; //data corrupt
	}
	tempdata1 = atoi(data_temp[0]); 
	tempdata2 = atoi(data_temp[1]); 
	tempdata3 = atoi(data_temp[2]); 
	tempdata4 = atoi(data_temp[3]); 
	tempdata5 = atoi(data_temp[4]);
	tempdata6 = atoi(data_temp[5]);

	*startthrottle = tempdata1;
	*endtthrottle = tempdata2;
	*unitstep = tempdata3;
	*interval = tempdata4;
	*delay_s = tempdata5;
	*retry = tempdata6;	
	//my_printf("\r\n%d,%d,%d,%d,%d,%d",*startthrottle,*endtthrottle,*unitstep,*interval,*delay_s,*retry);
	return 0;
}

//cmd_buf: command buffer
//return: 1 = command error, 0 = success
uint8_t GetPIDParameters(char* cmd_buf, uint16_t* var_pid)
{
 	char data_temp[6] = {'\0','\0','\0','\0','\0','\0'};
 	uint16_t tempdata1;
 	char* ptrBuf = 0x00;
 	uint8_t offset = 0;
 	uint8_t digit_counter = 0;

	ptrBuf = cmd_buf;
	while(*ptrBuf!='=')
	{
		offset++; ptrBuf++;
		if(offset>=USART2_MAX_RECV_LEN) return 1; //command error
	}
	ptrBuf++; //point to first digit of IP1
	while(*ptrBuf!='\0')
	{
			data_temp[digit_counter] = *ptrBuf;
			digit_counter++;
			ptrBuf++;
			if(digit_counter>6)	return 1;
	}

	tempdata1 = atoi(data_temp); 

	*var_pid = tempdata1;

	return 0;
}

//cmd_buf: command buffer
//return: 1 = command error, 0 = success
uint8_t GetTargetRPM(char* cmd_buf, uint16_t* var_rpm)
{
 	char data_temp[6] = {'\0','\0','\0','\0','\0','\0'};
 	uint16_t tempdata1;
 	char* ptrBuf = 0x00;
 	uint8_t offset = 0;
 	uint8_t digit_counter = 0;

	ptrBuf = cmd_buf;
	while(*ptrBuf!='=')
	{
		offset++; ptrBuf++;
		if(offset>=USART2_MAX_RECV_LEN) return 1; //command error
	}
	ptrBuf++; //point to first digit of IP1
	while(*ptrBuf!='\0')
	{
			data_temp[digit_counter] = *ptrBuf;
			digit_counter++;
			ptrBuf++;
			if(digit_counter>6)	return 1;
	}

	tempdata1 = atoi(data_temp); 

	*var_rpm = tempdata1;

	return 0;
}

//Linearize Servo 
//throttle_percentage: desired throttle percentage, 1000 = 100%
//return : linearized PWM output
#define SECTION1_STA	1
#define SECTION1_END	855

#define SECTION2_STA	855
#define SECTION2_END	1000


uint16_t ServoLinear(int16_t throttle_percentage)
{
	uint16_t det_x = 0;
	uint16_t mapped_y = 0;

	if(throttle_percentage>=1000)	throttle_percentage = 1000;	//Full throttle RPM=7600
	if(throttle_percentage<=1)		throttle_percentage = 1;	//idle RPM=3500

	if((throttle_percentage>=1)&&(throttle_percentage<855)) //section 1
	{
		det_x = throttle_percentage -  SECTION1_STA;
		mapped_y = 1845 - 0.2748*det_x;
	}
	else if((throttle_percentage>=855)&&(throttle_percentage<1000))	//section 2
	{
		det_x = throttle_percentage -  SECTION2_STA;
		mapped_y = 1610 - 0.8275*det_x;
	}
	// else if((throttle_percentage>=490)&&(throttle_percentage<=1000))//section 3
	// {
	// 	det_x = throttle_percentage -  SECTION3_STA;
	// 	mapped_y = 1288 + 0.3368*det_x;
	// }

	return mapped_y;
}
