
#include "timer.h"

void TIM3CH3out(uint16_t percent)
{
  /*input 0 = 0.0 %  
    input 508 = 50.8 %
    force out 100% if input more than 99%*/
  if(percent > 990) percent = 1200;
  TIM3->CCR3 = percent;    
}

void TIM3CH4out(uint16_t percent)
{
  /*input 0 = 0.0 %  
    input 508 = 50.8 %
    force out 100% if input more than 99%*/
  if(percent > 990) percent = 1200;
  TIM3->CCR4 = percent;    
}