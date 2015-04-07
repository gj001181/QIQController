#ifndef _LED_H
#define _LED_H

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

/*LED Pin Definition*/
#define LED_Port		GPIOA
#define LED_CLK			RCC_APB2Periph_GPIOA
#define LED_Pin			GPIO_Pin_10
#define LED_ON()		GPIO_SetBits( LED_Port, LED_Pin );  
#define LED_OFF()		GPIO_ResetBits( LED_Port, LED_Pin );

/*Function Prototype*/
//LED Initial 
void LED_Init(void);

#endif