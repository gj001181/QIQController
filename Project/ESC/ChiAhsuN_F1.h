#include "can.h"

uint8_t engcommand(uint8_t num);
void engtransdata(uint16_t rpm,uint16_t servopwm,float servoV,uint8_t servoT,uint16_t throPa,uint8_t statusnum);