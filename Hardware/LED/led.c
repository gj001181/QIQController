#include "led.h"

//LED configuration 
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    //Enable LED Clock
	RCC_APB2PeriphClockCmd(LED_CLK, ENABLE);
	//Disable JTAG and Enable SWD to use PB.03 as GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB , ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	//Configure LED Pin
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = 	LED_Pin; 	
	GPIO_Init(LED_Port, &GPIO_InitStructure);
}
