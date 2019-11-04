#ifndef TIMER_H
#define TIMER_H

#include "stm32f0xx_hal.h"

void InitTimerCallBack(void);
void TIM14_interruptCallBack(void);
void TIM3CH3out(uint16_t percent);
void TIM3CH4out(uint16_t percent);

#endif