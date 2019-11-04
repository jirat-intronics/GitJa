#include "FlashEmulated.h"
#include "delayITN.h"

#define WRITE_TIME_OUT     20
#define ERASE_TIME_OUT     50

uint8_t eepromtimer;
uint8_t eepromWriteFlag;
uint8_t saveFlag;

FlashStatus writeFlash(uint32_t address,uint16_t *data,uint8_t len)
{
  /* (1) Set the PG bit in the FLASH_CR register to enable programming */
  /* (2) Perform the data write (half-word) at the desired address */
  /* (3) Wait until the BSY bit is reset in the FLASH_SR register */
  /* (4) Check the EOP flag in the FLASH_SR register */
  /* (5) clear it by software by writing it at 1 */
  /* (6) Reset the PG Bit to disable programming */  

  uint16_t *ptr = (uint16_t*)address;
  FlashStatus status = FLASH_COMPLETED; 
  /* Wait busy clear */
  setTimeOutMS(WRITE_TIME_OUT);
  while(((FLASH->SR&FLASH_SR_BSY)==FLASH_SR_BSY)&& (!(isTimeOut()))){       
  }
    
  /*Set Flash programming*/
  FLASH->CR |= FLASH_CR_PG;			/* Set PG : Flash Programming Chosen*/
  
  for(uint8_t i=0;i<len;i++){
    /*Write data to specific address*/
    *ptr = *data;                
    /*Wait Busy flag or time out*/    
    setTimeOutMS(WRITE_TIME_OUT);
    while(((FLASH->SR&FLASH_SR_BSY)==FLASH_SR_BSY)&& (!(isTimeOut()))){       
    }      
    /*Check time out*/
    if(isTimeOut()) status = FLASH_TIMEOUT;
    
    /* Check End of Operation Flag  */
    if((FLASH->SR&FLASH_SR_EOP)==FLASH_SR_EOP){	
      FLASH->SR |= FLASH_SR_EOP;		/* reset EOP by writing 1       */      
    }
    /* Check Write Protection Error */
    if((FLASH->SR&FLASH_SR_WRPRTERR)==FLASH_SR_WRPRTERR) {	
      FLASH->SR |= FLASH_SR_WRPRTERR;		/* reset flag by writing 1      */
      status = FLASH_ERROR;
    }
    /* Check Programming Error      */
    if((FLASH->SR&FLASH_SR_PGERR)==FLASH_SR_PGERR){
      FLASH->SR |= FLASH_SR_PGERR;		/* reset flag by writing 1      */
      status = FLASH_ERROR;
    }
    
    /*increase source and destination address*/
    data++;
    ptr++;
  }  
  
  /*Clear Programing flash*/
  FLASH->CR &= ~(FLASH_CR_PG);			/* Reset PG                     */
  
  return(status);
}

void readFlash(uint32_t src_addr,uint16_t *dat,uint8_t byteperblock){
  /*    src_addr keep address in flash          */
  /*    dat point to destination memory         */
  /*    destination memory must be unit16_t (16bit) !!!         */
  /*    byteperblock must be 2^n at least 8 bytes = 8,16,32     */
  
  uint16_t *addr;
  uint16_t i,count;
  count =  byteperblock/2;
  addr = (uint16_t*) (src_addr);
  for(i=0;i<count;i++){
    *dat = *addr;
    dat++;
    addr++;
  }        
}

static void eraseFlash(uint32_t pageaddress){  
  /* (1) Set the PER bit in the FLASH_CR register to enable page erasing */
  /* (2) Program the FLASH_AR register to select a page to erase */
  /* (3) Set the STRT bit in the FLASH_CR register to start the erasing */
  /* (4) Wait until the BSY bit is reset in the FLASH_SR register */
  /* (5) Check the EOP flag in the FLASH_SR register */
  /* (6) Clear EOP flag by software by writing EOP at 1 */
  /* (7) Reset the PER Bit to disable the page erase */  
  
  
  /*Erase specific page*/
  FLASH->CR |= FLASH_CR_PER;                    /* Set PER : Page Erase Chosen*/
  FLASH->AR  = pageaddress;
  FLASH->CR |= FLASH_CR_STRT;		        /* Set STRT : Start , Trigger Erase Operation*/
  /*Wait for busy flag or time out*/  
  setTimeOutMS(ERASE_TIME_OUT);
  while(((FLASH->SR&FLASH_SR_BSY)==FLASH_SR_BSY)&&(!(isTimeOut()))){    
  }
  
  if((FLASH->SR&FLASH_SR_EOP)==FLASH_SR_EOP)	/* Check End of Operation Flag*/
    FLASH->SR |= FLASH_SR_EOP;			/* reset EOP by writing 1*/
  /* Check Write Protection Error */
  if((FLASH->SR&FLASH_SR_WRPRTERR)==FLASH_SR_WRPRTERR) {	
    FLASH->SR |= FLASH_SR_WRPRTERR;		/* reset flag by writing 1*/
  }
  /* Check Programming Error*/
  if((FLASH->SR&FLASH_SR_PGERR)==FLASH_SR_PGERR) {	
    FLASH->SR |= FLASH_SR_PGERR;		/* reset flag by writing 1*/
  }
  FLASH->CR &= ~(FLASH_CR_PER);		        /* Reset PER*/
}

void findBlankAddress(uint32_t pageaddress,
                      uint16_t pagesize,
                      uint8_t byteperblock,                      
                      uint32_t *blankaddr,
                      uint8_t *status)
{
  uint16_t offset = pagesize / 2;
  uint8_t *addr;
  uint32_t currentaddr;  
  uint8_t BankFull= RESET;
  
  currentaddr = pageaddress + offset;
  addr = (uint8_t*) currentaddr;
  
  do{
    offset >>=1;
    if((*addr) ==0xff){      
      currentaddr -= offset;      
    }else{      
      currentaddr += offset;      
    }
    addr =  (uint8_t*) currentaddr;
  }while(offset > byteperblock);  

  if(*addr ==0xff){
    /*Check in case of full blank*/
    addr = (uint8_t*) pageaddress;    
    if(*addr == 0xff) currentaddr = pageaddress;
    /*No page full*/
    BankFull= RESET;    
  }else{
    /*Check last block*/
    if(currentaddr >=  (pageaddress + pagesize - byteperblock)){
      /*If last Block not blank then set full page*/
      /*Reset blank address to Pageaddress*/
      BankFull= SET;
      currentaddr = pageaddress;
    }else{
      /*in case of value != 0xff and not last block*/
      currentaddr = currentaddr+byteperblock;
    }       
  }   
  /*Return value*/
  *blankaddr = currentaddr;
  *status = BankFull;
}

void requestSaveTagA1(void){
  /*Restart EEPROM timer*/
  eepromtimer = EEPROM_DELAY;
  eepromWriteFlag |= EEP_TAG_A1;
}

void requestSaveTagA3(void){
  /*Restart EEPROM timer*/
  eepromtimer = EEPROM_DELAY;
  eepromWriteFlag |= EEP_TAG_A3;
}

void requestSaveTagAF(void){
  /*Restart EEPROM timer*/
  eepromtimer = EEPROM_DELAY;
  eepromWriteFlag |= EEP_TAG_AF;
}

uint8_t checkRequestSaveError(void){
  if((eepromWriteFlag & EEP_TAG_AF)== EEP_TAG_AF)  return(1);
  else return(0);
}

void clearTagA1(void){
  eepromWriteFlag &= ~EEP_TAG_A1;
}

void clearTagA3(void){
  eepromWriteFlag &= ~EEP_TAG_A3;
}

void clearTagAF(void){
  eepromWriteFlag &= ~EEP_TAG_AF;
}

void checkRequestSave(void){
  /*if eepromtimer expired no save*/
  if(eepromtimer > 0)
  {
    /*decrease timer*/
    eepromtimer--;
    /*if timer decrease to zero then save*/
    if(eepromtimer==0) saveFlag = SET;
  }
}

uint8_t checkSaveFlag(void){
  return(saveFlag);
}

void clearSaveFlag(void){
  saveFlag = RESET;
}

uint8_t checkSaveBank(void){
  if((eepromWriteFlag & EEP_TAG_A1) == EEP_TAG_A1){
    return(0);
  }else if((eepromWriteFlag & EEP_TAG_A3) == EEP_TAG_A3){
    return(1);
  }else if((eepromWriteFlag & EEP_TAG_AF) == EEP_TAG_AF){
    return(2);
  }else{
    return(0xFF);
  }
}

void erasePage(uint8_t page){
  /*Specific page to erase depend on page size*/
  HAL_FLASH_Unlock();   /*Unlock flash*/
  eraseFlash(BANK_0_STRT+(page*PAGE_SIZE));
  HAL_FLASH_Lock();     /*Lock Flash access*/
}

uint8_t checkValidData(uint16_t *dat,uint8_t size){
   /*convert pointer to 8 bit to calculate check sum 8 bit*/
   uint8_t *ptr = (uint8_t*) dat;   
   uint8_t sum = 0;
   
   for(uint8_t i =0;i<size-1;i++){
     sum+= *ptr;
     ptr++;
   }     
   /* now ptr point at check sum data*/
   if(((*ptr)^0xFF) == sum) return (SET);
   else return (RESET);
}

void calChecksum(uint16_t *dat,uint8_t size){
   /*convert pointer to 8 bit to calculate check sum 8 bit*/
   uint8_t *ptr = (uint8_t*) dat;
   uint8_t sum = 0;
   for(uint8_t i =0;i<size-1;i++){
     sum+= *ptr;
     ptr++;
   }     
   /* Save check sum*/
   *ptr = sum^0xFF;
}

void clearBuffer(uint16_t *dat,uint8_t len){
  for(uint8_t i = 0;i<len;i++){
    *dat=0;
    dat++;
  }  
}

void increaseEEPTimer(void){
  eepromtimer = 1;
}










