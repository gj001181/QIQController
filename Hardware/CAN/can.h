#ifndef __CAN_H
#define __CAN_H	 

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

extern uint8_t canbuf[8];

//CAN Rx Interrupt Enable/Disable
#define CAN_RX0_INT_ENABLE	0		//0,Disable;1,Enable.								    
										 							 				    
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN Initialization
 
u8 Can_Send_Msg(u8* msg,uint32_t msgid);						//Tx Data

u8 Can_Receive_Msg(u8 *buf,uint32_t msgid);							//Rx Data
#endif

















