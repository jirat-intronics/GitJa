
/*---------------------------------------------------
         How to use code
   - include "driverhc259.h" in main.c           
   - Need to define port output hereafter in main.h!!!
     and don't forget enable peripheral clock each port
     and config all port direction to output !!!
   + EN_259PORTx   port which use to enable IC259 group x   
   + EN_259PINx    pin which use to enable IC259 group x  
   + DATA_259PORT  port which use as data of IC259x   
   + DATA_259PIN   pin which use as data of IC259x   
   + A0_259PORT    port which use as A0 of IC259x   
   + A0_259PIN     pin which use as A0 of IC259x   
   + A1_259PORT    port which use as A1 of IC259x   
   + A1_259PIN     pin which use as A1 of IC259x   
   + A2_259PORT    port which use as A2 of IC259x   
   + A2_259PIN     pin which use as A2 of IC259x   
   
   - set DELAY259(multiplier) if change system frequency clock 
     in driverhc259.h
   
   - call setOutputIC259 used to manage output of 74HC259

----------------------------------------------------*/

#include "driverhc259.h"


void setOutputIC259(uint8_t bit_pos,uint8_t bit_dat){
  /* bit_pos is bit position have beed defined in main.h
     bit_dat can be 0 or 1 */
  
  /*set address pin A0*/
  if((bit_pos & (1<<0))==0){   
    /*clear Port A0*/
    A0_259PORT->BRR = (1<<A0_259PIN);
  }else{
    /*Set port A0*/
    A0_259PORT->BSRR = (1<<A0_259PIN);
  }
  /*set address pin A1*/
  if((bit_pos & (1<<1))==0){   
    /*clear Port A1*/
    A1_259PORT->BRR = (1<<A1_259PIN);
  }else{
    /*Set port A1*/
    A1_259PORT->BSRR = (1<<A1_259PIN);
  }
  /*set address pin A2*/
  if((bit_pos & (1<<2))==0){   
    /*clear Port A2*/
    A2_259PORT->BRR = (1<<A2_259PIN);
  }else{
    /*Set port A*/
    A2_259PORT->BSRR = (1<<A2_259PIN);
  }
  
  if(bit_dat){
    /*Set port data of hc259*/
    DATA_259PORT->BSRR = (1<<DATA_259PIN);
  }else{
    /*clear port data of hc259*/
    DATA_259PORT->BRR = (1<<DATA_259PIN);
  }  
  /*Wait before latch Enable*/
  waitDelay(DELAY259EN);  
  /*Select ICx*/  
  if(bit_pos<=7){
    /*IC No.1 group 1*/
    EN_259PORT1->BRR = (1<<EN_259PIN1);  // clear Port En1 to enable latch
    waitDelay(DELAY259LATCH);            // Wait for latch
    EN_259PORT1->BSRR = (1<<EN_259PIN1); // disable latch       
  }else if(bit_pos<=15){
    /*IC No.2 group 2*/
    EN_259PORT2->BRR = (1<<EN_259PIN2);  // clear Port En2 to enable latch
    waitDelay(DELAY259LATCH);            // Wait for latch
    EN_259PORT2->BSRR = (1<<EN_259PIN2); // disable latch       
    
  }else if(bit_pos<=23){
    /*IC No.3 group 3*/
    EN_259PORT3->BRR = (1<<EN_259PIN3);  // clear Port En3 to enable latch
    waitDelay(DELAY259LATCH);            // Wait for latch
    EN_259PORT3->BSRR = (1<<EN_259PIN3); // disable latch               
  }else if(bit_pos<=31){
    /*IC No.4 group 4*/
    EN_259PORT4->BRR = (1<<EN_259PIN4);  // clear Port En4 to enable latch
    waitDelay(DELAY259LATCH);            // Wait for latch
    EN_259PORT4->BSRR = (1<<EN_259PIN4); // disable latch               
  }else if(bit_pos<=39){
    /*IC No.5 group 5*/
    EN_259PORT5->BRR = (1<<EN_259PIN5);  // clear Port En5 to enable latch
    waitDelay(DELAY259LATCH);            // Wait for latch
    EN_259PORT5->BSRR = (1<<EN_259PIN5); // disable latch       
  }else if(bit_pos<=47){
    /*IC No.6 group 6*/
    EN_259PORT6->BRR = (1<<EN_259PIN6);  // clear Port En6 to enable latch
    waitDelay(DELAY259LATCH);            // Wait for latch
    EN_259PORT6->BSRR = (1<<EN_259PIN6); // disable latch              
  }else{
    /*IC No.7 group 7*/
    EN_259PORT7->BRR = (1<<EN_259PIN7);  // clear Port En7 to enable latch
    waitDelay(DELAY259LATCH);            // Wait for latch
    EN_259PORT7->BSRR = (1<<EN_259PIN7); // disable latch             
  }
}