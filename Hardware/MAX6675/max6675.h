#ifndef __MAX6675_H
#define __MAX6675_H
#include "stm32f10x.h"
#include "stm32f10x_conf.h"


extern int32_t Engine_Temperature; 
//*********************************************************
#define MAX6675_CS_H()	GPIO_SetBits( GPIOB, GPIO_Pin_7 );		//Disable Max6675
#define MAX6675_CS_L()	GPIO_ResetBits( GPIOB, GPIO_Pin_7 );	//Enable Max6675
/*函數原型宣告*/
void MAX6675_Init(void);
uint8_t Read_MAX6675(int32_t* temp);

//*********************************************************
#endif
