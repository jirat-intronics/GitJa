#ifndef ADCITN_H
#define ADCITN_H

#include "stm32f0xx_hal.h"

void calibrateADC(void);
void enableADC(void);
void disableADC(void);
void convertADC(uint32_t ch,uint8_t no_ch,uint16_t *dat);


#endif