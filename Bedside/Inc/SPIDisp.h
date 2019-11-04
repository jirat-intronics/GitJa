#ifndef SPIDISP_H
#define SPIDISP_H

#include "stm32f0xx_hal.h"


/*User define port of Chip select pin*/
#define CS_PORT         GPIOA
#define CS_PIN          (4)
#define SPI_TX_SIZE     (38)
/*include first void byte*/
#define SPI_RX_SIZE     (24+1)

enum{
  SPI_PREPARE,
  SPI_WAIT,
  SPI_DATA_READY
};

typedef struct{
  uint8_t *RXPointer;
  uint8_t RXCount;
  uint8_t RXMaxByte;
  uint8_t *TXPointer;
  uint8_t TXCount;
  uint8_t TXMaxByte;
  uint8_t SPIState;
}SPIDataHandler;

/*Prototype function*/
void sendSPIDisplay(SPI_TypeDef *SPICH,SPIDataHandler *spi);
void SPI_interruptCallback(SPI_TypeDef *SPICH,SPIDataHandler *spi);

#endif