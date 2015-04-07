#include "button.h"

//button configuration 
void Button_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    //Enable Button Clock
	RCC_APB2PeriphClockCmd(BTN_CLK, ENABLE);
	//Configure Button Pin
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IPU; //Pull Up
	GPIO_InitStructure.GPIO_Pin = 	BTN_Pin; 	
	GPIO_Init(BTN_Port, &GPIO_InitStructure);
}
