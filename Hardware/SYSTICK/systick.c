#include "systick.h"

uint32_t SYS_CNT = 0; //system timer count
uint32_t MAX6675_LoopCNT = 0; //timer count for MAX6675

void SysTick_Handler(void)
{
  if(SYS_CNT<SYS_LOOP)
    SYS_CNT++;
  if(MAX6675_LoopCNT<=Max6675LoopFactor)
  	MAX6675_LoopCNT++;

}
