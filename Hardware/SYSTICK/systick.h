#ifndef _SYSTICK_H
#define _SYSTICK_H

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#define SYS_LOOP 1 
#define LOOP_FREQ 50 //Hz
#define DET_TIME 1000/LOOP_FREQ	//delta time for control loop(ms)
#define SYSTICK_PRESCALER (72*1000*1000/LOOP_FREQ)

#define Max6675LoopFactor 200/(1000/(LOOP_FREQ)) //5Hz is max. spped for MAX6675 read out

extern uint32_t SYS_CNT;
extern uint32_t MAX6675_LoopCNT;

/*Function Prototype*/
void SysTick_Handler(void);

#endif
