#ifndef DRIVERHC259_H
#define DRIVERHC259_H

#include "main.h"

/*Set DELAY259 if system clock change*/
#define DELAY259            1   
#define DELAY259EN         (4*DELAY259)
#define DELAY259LATCH      (2*DELAY259)

void setOutputIC259(uint8_t bit_pos,uint8_t bit_dat);

#endif /*End if DRIVERHC259_H*/