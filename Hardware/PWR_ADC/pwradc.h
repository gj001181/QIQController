#ifndef _PWRADC_H
#define _PWRADC_H

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

extern uint32_t pwr_adc_Value;
extern int32_t intpwr_volt;
extern double dpwr_volt;

/*Function Prototype*/
void PWRADC_Init(void);

#endif