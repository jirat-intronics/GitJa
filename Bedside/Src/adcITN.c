
#include "adcITN.h"


/*
- Call calibrateADC to calibrate calibration factor
  After calibration ADC still disable
- After calibrate then call enableADC and no disableADC 
  if no need power concerned
- set speed of conversion / resolution in STM32cube
- Call convertADC to convert multichannel upward direction

*/


void calibrateADC(void)
{
  /* (1) Ensure that ADEN = 0 */
  /* (2) Clear ADEN by setting ADDIS*/
  /* (3) Clear DMAEN */
  /* (4) Launch the calibration by setting ADCAL */
  /* (5) Wait until ADCAL=0 */
  if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
  {
    ADC1->CR |= ADC_CR_ADDIS; /* (2) */
  }
  while ((ADC1->CR & ADC_CR_ADEN) != 0)
  {
    /* For robust implementation, add here time-out management */
  }
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
  ADC1->CR |= ADC_CR_ADCAL; /* (4) */
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
  {
    /* For robust implementation, add here time-out management */
  }
}

void enableADC(void)
{
  /* (1) Ensure that ADRDY = 0 */
  /* (2) Clear ADRDY */
  /* (3) Enable the ADC */
  /* (4) Wait until ADC ready */
  if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) /* (1) */
  {
    ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
  }
  ADC1->CR |= ADC_CR_ADEN; /* (3) */
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */
  {
    /* For robust implementation, add here time-out management */
  }
}

void disableADC(void){
  /* (1) Stop any ongoing conversion */
  /* (2) Wait until ADSTP is reset by hardware i.e. conversion is stopped */
  /* (3) Disable the ADC */
  /* (4) Wait until the ADC is fully disabled */
  ADC1->CR |= ADC_CR_ADSTP; /* (1) */
  while ((ADC1->CR & ADC_CR_ADSTP) != 0) /* (2) */
  {
    /* For robust implementation, add here time-out management */
  }
  ADC1->CR |= ADC_CR_ADDIS; /* (3) */
  while ((ADC1->CR & ADC_CR_ADEN) != 0) /* (4) */
  {
    /* For robust implementation, add here time-out management */
  }  
}

void convertADC(uint32_t ch,uint8_t no_ch,uint16_t *dat)
{
  
  ADC1->CHSELR = ch;                     /* Select channel*/   
  ADC1->CFGR1 &= ~ADC_CFGR1_SCANDIR;     /* scan upward direction */
  
  /* Performs the AD conversion */
  ADC1->CR |= ADC_CR_ADSTART; /* Start the ADC conversion */
  for (uint8_t i=0; i < no_ch; i++)
  {
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) /* Wait end of conversion */
    {
      /* For robust implementation, add here time-out management */
    }
    *dat = ADC1->DR; /* Store the ADC conversion result */
    dat++;
  }    
  
}