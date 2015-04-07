#ifndef _BUTTON_H
#define _BUTTON_H

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

/*Button Pin Definition*/
#define BTN_Port			GPIOA
#define BTN_CLK				RCC_APB2Periph_GPIOA
#define BTN_Pin				GPIO_Pin_3
#define IF_BTN_PRESSED()	GPIO_ReadInputDataBit(BTN_Port,BTN_Pin)
/*Function Prototype*/
//Button Initial 
void Button_Init(void);

#endif