#ifndef FLASH_EMULATED_H
#define FLASH_EMULATED_H

#include "stm32f0xx_hal.h"

#define EEPROM
#define EEPROM_DELAY  3
#define PAGE_SIZE        (2048)

/*Redefine here*/
#define BANK_0_STRT      (0x801D000)
#define BANK_0_SIZE      (4*1024)
#define BANK_1_SIZE      (4*1024)
#define BANK_2_SIZE      (4*1024)

#define BANK_1_STRT      (BANK_0_STRT+BANK_0_SIZE) 
#define BANK_2_STRT      (BANK_1_STRT+BANK_1_SIZE)
 
#define BLOCK_SIZE_BANK0        (16)
#define BLOCK_SIZE_BANK1        (32)
#define BLOCK_SIZE_BANK2        (32)


#define EEP_TAG_A1  (1<<0)
#define EEP_TAG_A3  (1<<1)
#define EEP_TAG_AF  (1<<2)


typedef struct{
  uint32_t blankaddress;
  enum{
   EEP_IDLE,   
   EEP_ERASE_1ST_PAGE,   
   EEP_ERASE_LEFT         
  }state;  
}EEPROMHandler;

typedef enum{
  FLASH_COMPLETED,
  FLASH_ERROR,
  FLASH_TIMEOUT,
}FlashStatus;

FlashStatus writeFlash(uint32_t address,uint16_t *data,uint8_t len);
void readFlash(uint32_t addr,uint16_t *dat,uint8_t byteperblock);
void findBlankAddress(uint32_t pageaddress,
                      uint16_t pagesize,
                      uint8_t byteperblock,                      
                      uint32_t *blankaddr,
                      uint8_t *status);

/*Prepare to save TAG_A1 data*/
void requestSaveTagA1(void);
void clearTagA1(void);

/*Prepare to save TAG_A3 data*/
void requestSaveTagA3(void);
void clearTagA3(void);

/*Prepare to save TAG_AF data*/
void requestSaveTagAF(void);
uint8_t checkRequestSaveError(void);
void clearTagAF(void);

void checkRequestSave(void);
uint8_t checkSaveFlag(void);
void clearSaveFlag(void);
uint8_t checkSaveBank(void);
void increaseEEPTimer(void);

/*Private function*/
void erasePage(uint8_t page);
static void eraseFlash(uint32_t pageaddress);

/*For validate data*/
uint8_t checkValidData(uint16_t *dat,uint8_t size);
void calChecksum(uint16_t *dat,uint8_t size);
void clearBuffer(uint16_t *dat,uint8_t len);

#endif