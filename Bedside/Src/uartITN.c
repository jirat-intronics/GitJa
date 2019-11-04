
#include "uartITN.h"

/*
  - Must write initial UART Handler to define size and time out
  - Must Call USART_IRQHandler in interrupt depend on interrupt vector
  - Must DecreaseUARTRXTimeout to decrease time out
  - Write UART_RXComplete_CallBack or UART_TXComplete_CallBack
  in main in order to process when RX and tx complete respectively

*/

volatile uint8_t testram;

void USART_IRQHandler(USART_TypeDef *USART,USARTHandler *UARTDat){
  /*Handler USART*/
  if (USART->ISR & (USART_ISR_ORE|USART_ISR_NE|USART_ISR_FE|USART_ISR_PE)){ 
  
    /*Error found interrupt*/
    if (USART->ISR & USART_ISR_ORE) USART->ICR |= USART_ICR_ORECF;          
    if (USART->ISR & USART_ISR_NE) USART->ICR |= USART_ICR_NCF;
    if (USART->ISR & USART_ISR_FE) USART->ICR |= USART_ICR_FECF;
    if (USART->ISR & USART_ISR_PE) USART->ICR |= USART_ICR_PECF;
    /*Flush all receive data*/
    USART->RQR |= USART_RQR_RXFRQ;
  }else{
    /*Handler depend on mode*/
    if(USART->CR1&USART_CR1_RXNEIE){
      /*RX not empty interrupt enable*/
      if(USART->ISR & USART_ISR_RXNE){
        if(UARTDat->RXCount < UARTDat->RXMaxByte){                    
          /*Save data to buffer*/          
          *(UARTDat->RXPointer) = USART->RDR; 
          testram = USART->RDR;
          /*Increase pointer*/
          UARTDat->RXPointer++;
          /*Increase data count*/
          UARTDat->RXCount++;
          /*Reload timeout*/
          UARTDat->RXTimeOut = UARTDat->RXTimeOutReload;                    
        }
      }
      
    }else if(USART->CR1 & USART_CR1_TCIE){
      /*TX complete interrupt enable*/
      if(USART->ISR & USART_ISR_TC){
        if(UARTDat->TXCount == UARTDat->TXMaxByte){
          /*Clear TX complete flag*/
          USART->ICR |= USART_ICR_TCCF;
          /*Disable both TX empty and Tx complete interrupt*/
          DisableTXEmptyInterrupt(USART);
          DisableTXCptInterrupt(USART);  
          /*Call back routine*/
          UART_TXComplete_CallBack(UARTDat);
        }
      }
      
    }else if(USART->CR1 & USART_CR1_TXEIE){      
      /*TX not empty interrupt enable*/
      if(USART->ISR &USART_ISR_TXE){ 
        /*TX not empty flag set*/
        USART->TDR = *(UARTDat->TXPointer);
        /*Increase pointer*/
        UARTDat->TXPointer++;
        /*Increase data count*/
        UARTDat->TXCount++;
        if(UARTDat->TXCount >= UARTDat->TXMaxByte){
          EnableTXCptInterrupt(USART);
          DisableTXEmptyInterrupt(USART);
        }
      }      
    }          
  }
}

void DecreaseUARTRXTimeout(USARTHandler *UARTDat){
  if((UARTDat->RXTimeOut) > 0){  
    UARTDat->RXTimeOut--;
    if(UARTDat->RXTimeOut ==0){
      UART_RXComplete_CallBack(UARTDat);
    }
  }
}

__weak void UART_RXComplete_CallBack(USARTHandler *UARTDat){
  
}
      
__weak void UART_TXComplete_CallBack(USARTHandler *UARTDat){
  
}
