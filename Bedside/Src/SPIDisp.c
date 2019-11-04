#include "SPIDisp.h"

/*  !!!define Chip select IO in  SPIDISP.h
  call SPI_interruptCallback in SPI interrupt
  call sendSPIDisplay when start to send data
*/


void sendSPIDisplay(SPI_TypeDef *SPICH,SPIDataHandler *spi){    
  /*Enable Tx empty*/
  if((SPICH->CR2 & SPI_CR2_TXEIE) == 0)  SPICH->CR2 |= SPI_CR2_TXEIE;
  /*Enable RX not empty*/
  if((SPICH->CR2 & SPI_CR2_RXNEIE) == 0) SPICH->CR2 |= SPI_CR2_RXNEIE;
  /*Set RX threshold to 8 bit*/
  SPICH->CR2 |= SPI_CR2_FRXTH;
  
  /*Write first byte to tx FIFO before enable SPI*/
  *(uint8_t*)&SPICH->DR = *(spi->TXPointer);  
  /*increase TX counter*/
  spi->TXCount = 1;
  /*Reset Rx counter*/
  spi->RXCount = 0;
  /*increase pointer*/
  spi->TXPointer++;
  /*Enable SPI */  
  if((SPICH->CR1 & SPI_CR1_SPE) == 0)  SPICH->CR1 |= SPI_CR1_SPE;    
}

void SPI_interruptCallback(SPI_TypeDef *SPICH,SPIDataHandler *spi)
{
  uint8_t temp;  
  
  if(spi->RXCount == 3){
    __no_operation();
  }
  
  if(((SPICH->SR) & SPI_SR_RXNE) ==  SPI_SR_RXNE){
    /*If Rx not empty*/    
    temp = (uint8_t) (SPICH->DR);
    if(spi->RXCount < spi->RXMaxByte){
      /*If buffer not full still receieved*/
      *(spi->RXPointer) = temp;
      spi->RXPointer++;
      spi->RXCount++;
    }
  }
  
  if(((SPICH->SR) & SPI_SR_TXE) == SPI_SR_TXE){
    /*if TX empty*/
    if(spi->TXCount < (spi->TXMaxByte)){
      /* if sending not complete then send */
      *(uint8_t*)&SPICH->DR = *(spi->TXPointer);  
      /*increase pointer*/
      spi->TXPointer++;
      /*increase TX counter*/
      spi->TXCount++;
      
    }else{
      /*Last byte Wait for busy flag*/
      while((SPICH->SR & SPI_SR_BSY) == SPI_SR_BSY);      
      /*Disable chip select*/
      CS_PORT->ODR |= 1<<CS_PIN;
      /*Disable spi data*/
      SPICH->CR1 &= ~SPI_CR1_SPE;
    }      
  }    
}

