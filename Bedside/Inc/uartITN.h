#ifndef UARTITN_H
#define UARTITN_H

#include "stm32f0xx_hal.h"



/*Macro function*/
//#define DisableUART(x)   x->CR1 &= ~(USART_CR1_TE | USART_CR1_RE)
#define EnableTXUART(x)  x->CR1 |= USART_CR1_TE
#define DisableTXUART(x) x->CR1 &= ~USART_CR1_TE
#define EnableRXUART(x)  x->CR1 |= USART_CR1_RE
#define DisableRXUART(x) x->CR1 &= ~USART_CR1_RE
#define EnableRXEmptyInterrupt(x)  x->CR1 |= USART_CR1_RXNEIE
#define DisableRXEmptyInterrupt(x) x->CR1 &= ~USART_CR1_RXNEIE
#define EnableTXEmptyInterrupt(x)  x->CR1 |= USART_CR1_TXEIE
#define DisableTXEmptyInterrupt(x) x->CR1 &= ~USART_CR1_TXEIE
#define EnableTXCptInterrupt(x)  x->CR1 |= USART_CR1_TCIE
#define DisableTXCptInterrupt(x) x->CR1 &= ~USART_CR1_TCIE
#define FlushRXData(x)  x->RQR |= USART_RQR_RXFRQ

enum{
  UART_CH1,
  UART_CH2,
  UART_CH3,
  UART_CH4,
};

typedef struct{
  uint8_t Instance;
  uint8_t *RXPointer;
  uint8_t RXCount;
  uint8_t RXMaxByte;
  uint16_t RXTimeOut;
  uint16_t RXTimeOutReload;
  uint8_t *TXPointer;
  uint8_t TXCount;
  uint8_t TXMaxByte;
  uint16_t TXTimeOut;
}USARTHandler;

/*Prototype function*/
void InitUARTHandler(void);
void USART_IRQHandler(USART_TypeDef *USART,USARTHandler *UARTDat);
void DecreaseUARTRXTimeout(USARTHandler *UARTDat);
void UART_RXComplete_CallBack(USARTHandler *UARTDat);
void UART_TXComplete_CallBack(USARTHandler *UARTDat);
                          
#endif /*UARTITN_H*/                         