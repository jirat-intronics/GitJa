/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_dma.h"

#include "stm32f0xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define IAc_FHiR                   (RlCct_ID+1)                  // fan high speed relay
#define IAc_FMdR                   (RlCct_ID+2)                  // fan medium speed relay
#define IAc_FLwR                   (RlCct_ID+3)                  // fan low speed relay
#define IAc_ValR                   (RlCct_ID+4)                  // valve relay

#define IcAcRN_U                   (RlCct_ID+4)                  // upper limit of included air-con relay circuit no.

  
#define SsE_Vl1T                   300                           // valve interval cycle on time of sensor error (300s = 5m)
#define SsE_Vl0T                   300                           // valve interval cycle off time of sensor error (300s = 5m)  


//------------------------------------------------------------------------------
// table temperature index (0.5C/step)
#define TbTI__0C         20                            // table temperature index of 0.0C
#define COs_TbTI         (2*(0))                       // common offset of table temperature index (0.0C)
                                                       // for cover negative calibration
                                                       // (must support minimum table temperature index offset -10.0C)
//-------------------------------------------
#define FI_TmpTb         0                             // 1st index of temperature table
#define EI_TmpTb         143                           // end index of temperature table
//-------------------------------------------
#define TbI__M9C         (2*(-9)+TbTI__0C+COs_TbTI)    // table temperature index of  -9.0C
#define TbI___0C         (2*(0) +TbTI__0C+COs_TbTI)    // table temperature index of   0.0C
#define TbI__15C         (2*(15)+TbTI__0C+COs_TbTI)    // table temperature index of  15.0C
#define TbI__16C         (2*(16)+TbTI__0C+COs_TbTI)    // table temperature index of  16.0C
#define TbI__18C         (2*(18)+TbTI__0C+COs_TbTI)    // table temperature index of  18.0C
#define TbI__22C         (2*(22)+TbTI__0C+COs_TbTI)    // table temperature index of  22.0C
#define TbI__23C         (2*(23)+TbTI__0C+COs_TbTI)    // table temperature index of  23.0C
#define TbI__24C         (2*(24)+TbTI__0C+COs_TbTI)    // table temperature index of  24.0C
#define TbI__25C         (2*(25)+TbTI__0C+COs_TbTI)    // table temperature index of  25.0C
#define TbI__26C         (2*(26)+TbTI__0C+COs_TbTI)    // table temperature index of  26.0C
#define TbI__27C         (2*(27)+TbTI__0C+COs_TbTI)    // table temperature index of  27.0C
#define TbI__30C         (2*(30)+TbTI__0C+COs_TbTI)    // table temperature index of  30.0C
#define TbI__31C         (2*(31)+TbTI__0C+COs_TbTI)    // table temperature index of  31.0C
#define TbI__40C         (2*(40)+TbTI__0C+COs_TbTI)    // table temperature index of  40.0C
#define TbI__46C         (2*(46)+TbTI__0C+COs_TbTI)    // table temperature index of  46.0C
#define TbI__60C         (2*(60)+TbTI__0C+COs_TbTI)    // table temperature index of  60.0C
//-------------------------------------------
#define MxL_TbId         TbI__60C                      // maximum limit of table temperature index (60.0C)
#define MnL_TbId         TbI___0C                      // minimum limit of table temperature index ( 0.0C)
//------------------------------------------------------------------------------   
//-------------------------------------------   
//unsigned char FnAuSp_S;                      // fan auto speed state
#define FnAuS_Lw                   0                             // fan auto low speed
#define FnAuS_Md                   1                             // fan auto medium speed
#define FnAuS_Hi                   2                             // fan auto high speed
    
//-------------------------------------------
#if(RCUM_PCB == iS1K6P_M)||(RCUM_PCB == iS1K6__M)
  #define OwRlC_ID                 15                            // number of own relay circuit index
  #if(U_ShrBus&&U_ShrCct)
    #if(U_DSzRlM)
      #define RlCct_ID             31                            // number of relay circuit index
    #else
      #define RlCct_ID             15                            // number of relay circuit index
    #endif
  #else
    #define RlCct_ID               15                            // number of relay circuit index
  #endif
#elif(RCUM_PCB == iS1K4__M)
  #define OwRlC_ID                 4                             // number of own relay circuit index
  #define RlCct_ID                 4                             // number of relay circuit index
#endif    
    
    
#define HwTBf_Sz                   3                             // hardware testing buffer size (byte)
//unsigned char CmMd_Lv1;                      // command mode type level1

#define Idle___M                   0                             // idle mode
#define UsrSet_M                   1                             // user setting mode
#define PrgLst_M                   2                             // program list mode
#define PrgSet_M                   3                             // program setting mode
#define DfuPrg_M                   4                             // default program mode
#define NtwAdr_M                   5                             // networking address mode
#define HwTsLs_M                   6                             // hardware testing list mode
#define HwTsSt_M                   7                             // hardware testing step mode
#define SfwrVr_M                   8                             // software version mode
#define TmBomb_M                   9                             // time bome mode
#define ClokSt_M                   10                            // clock setting mode
//-------------------------------------------
//unsigned char CmMd_Lv2;                      // command mode type level2

// UsrSet_M
#define RmStTp_M                   0                             // room setting temperature
#define AcOpMd_M                   1                             // air-con operating mode
#define AcFnSp_M                   2                             // air-con fan speed
#define AlCkEn_M                   3                             // alarm clock enable
#define AlCkTm_M                   4                             // alarm clock time
#define QClkSt_M                   5                             // quick clock setting
#define WldTme_M                   6                             // world time
#define SenPrS_M                   7                             // sensor parameter status  
#define InputT_M                   0                             // input testing
#define OtLwVT_M                   1                             // output low volt testing
#define OtHiVT_M                   2                             // output high volt testing

#define N_HwTsLs                   2                             // number of sub-program
 //-------------------------------------------  
//unsigned char CmMd_Lv3;                      // command mode type level3

#define Rlc001_I                   0                             // rc001
#define Rlc002_I                   1                             // rc002
#define Rlc003_I                   2                             // rc003
#define Rlc004_I                   3                             // rc004
#define Rlc005_I                   4                             // rc005
#define Rlc006_I                   5                             // rc006
#define Rlc007_I                   6                             // rc007
#define Rlc008_I                   7                             // rc008
#define Rlc009_I                   8                             // rc009
#define Rlc010_I                   9                             // rc010
#define Rlc011_I                   10                            // rc011
#define Rlc012_I                   11                            // rc012
#define Rlc013_I                   12                            // rc013
#define Rlc014_I                   13                            // rc014
#define Rlc015_I                   14                            // rc015
#define Rlc016_I                   15                            // rc016
#define RlcA01_I                        16                            // rcA01
#define RlcA02_I                        17                            // rcA02
#define RlcA03_I                        18                            // rcA03
#define RlcA04_I                        19                            // rcA04
#define RlcA05_I                        20                            // rcA05
#define RlcA06_I                        21                            // rcA06
#define RlcA07_I                        22                            // rcA07
#define RlcA08_I                        23                            // rcA08
#define RlcA09_I                        24                            // rcA09
#define RlcA10_I                        25                            // rcA10
#define RlcA11_I                        26                            // rcA11
#define RlcA12_I                        27                            // rcA12
#define RlcA13_I                        28                            // rcA13
#define RlcA14_I                        29                            // rcA14
#define RlcA15_I                        30                            // rcA15
#define RlcA16_I                        31                            // rcA16    
 //-------------------------------------------    
  
  
// merged 8 bits unsigned integer
typedef union {
  unsigned char uint8;
  struct {
    unsigned nb0                   :4;
    unsigned nb1                   :4;
  } nibble;
  struct {
    unsigned b0                    :1;
    unsigned b1                    :1;
    unsigned b2                    :1;
    unsigned b3                    :1;
    unsigned b4                    :1;
    unsigned b5                    :1;
    unsigned b6                    :1;
    unsigned b7                    :1;
  } bit;
} MbtUint8;  
  //-------------------------------------------  
//-------------------------------------------

#define Init_Rdy                   GenFg_01.bit.b7               // initialize ready flag
#define Hrdw_Tst                   GenFg_01.bit.b6               // hardware testing flag
#define NegRs_Fg                   GenFg_01.bit.b5               // negative result flag for minus number
#define NvMem_Er                   GenFg_01.bit.b4               // non-volatile memory error flag
#define I2C_NAck                   GenFg_01.bit.b3               // I2C bus no-acknowledge flag
#define RTCWri_F                   GenFg_01.bit.b2               // RTC writing flag
#define Us_RTCFt                   GenFg_01.bit.b1               // using RTC feature status flag
#define PrTbC_CC                   GenFg_01.bit.b0               // parameter table code calculation complete flag
//#define FqCk_Rdy                   GenFg_01.bit.b1               // frequency checking ready flag
//#define Freq_60H                   GenFg_01.bit.b0               // 60Hz system flag

#define NtwB_Tsm                   GenFg_02.bit.b7               // networking bus transmitting flag
#define NtwB_RCp                   GenFg_02.bit.b6               // networking bus received complete flag
#define EpsB_Tsm                   NtwB_Tsm                      // expansion bus transmitting flag
#define EpsB_RCp                   NtwB_RCp                      // expansion bus received complete flag
#define NtwB_TBC                   GenFg_02.bit.b5               // networking bus transmit break character flag
#define TwWB_Tsm                   GenFg_02.bit.b4               // 2-wire bus transmitting flag
#define TwWB_RCp                   GenFg_02.bit.b3               // 2-wire bus received complete flag
#define SPI_Prog                   GenFg_02.bit.b2               // SPI progressing flag
#define SPIS2_Sl                   GenFg_02.bit.b1               // SPI slave2 select flag

#define OtOfR_S1                   GenFg_03.bit.b7               // sensor1 out of range flag
#define RmTpS_Er                   GenFg_03.bit.b6               // room temperature sensor error flag
#define FnArcn_S                   GenFg_03.bit.b5               // final air-con status flag
#define StTp_Lmt                   GenFg_03.bit.b4               // setting temperature limited flag
#define C1Cp0D_F                   GenFg_03.bit.b3               // cleared first compressor off delay flag
#define NwCmEv_F                   GenFg_03.bit.b2               // new common event flag
#define DpKbd_Pr                   GenFg_03.bit.b1               // display keyboard pressed flag
#define RmHmS_Er                   GenFg_03.bit.b0               // room humidity sensor error flag

#define TmBom_Ac                   GenFg_04.bit.b7               // time bomb activated flag
#define NLnS_CrD                   GenFg_04.bit.b6               // normal line status current data
#define NmLn_Otg                   GenFg_04.bit.b5               // normal line outage flag
#define BcATS_Nm                   GenFg_04.bit.b4               // broadcast ATS saying normal status flag
#define Ig_ATSCm                   GenFg_04.bit.b3               // ignore ATS command flag
#define SOn_EmLg                   GenFg_04.bit.b2               // switch on emergency light flag
#define FOn_EmLg                   GenFg_04.bit.b1               // force on emergency light flag
#define SbAcS0_S                   GenFg_04.bit.b0               // standby air-con shut off status flag

#define DmDpLd_F                   GenFg_05.bit.b7               // dimming display led flag
#define T0DpBl_F                   GenFg_05.bit.b6               // turning off display backlight flag
#define TuOfDp_F                   GenFg_05.bit.b5               // turning off display flag
#define IRmC_TRq                   GenFg_05.bit.b4               // infrared remote code transmitting request flag
#define IRmC_Tsm                   GenFg_05.bit.b3               // infrared remote code transmitting flag
#define IRT_SrFm                   GenFg_05.bit.b2               // IRC transmitting start frame flag
#define KbBI_RCp                   GenFg_05.bit.b1               // key box bus input received complete flag
#define KbdPt_CA                   GenFg_05.bit.b0               // keyboard protocol command acknowledge flag

#define Dm1B_Tsm                   GenFg_06.bit.b7               // dimmer1 bus transmitting flag
#define Dm2B_Tsm                   GenFg_06.bit.b6               // dimmer2 bus transmitting flag
#define Dm3B_Tsm                   GenFg_06.bit.b5               // dimmer3 bus transmitting flag
#define Dm4B_Tsm                   GenFg_06.bit.b4               // dimmer4 bus transmitting flag
#define Dm5B_Tsm                   GenFg_06.bit.b3               // dimmer5 bus transmitting flag
#define Ct1OTu_C                   GenFg_06.bit.b2               // curtain1 open turning direction command flag
#define Ct1ORl_S                   GenFg_06.bit.b1               // curtain1 open relay status flag
#define Ct1CRl_S                   GenFg_06.bit.b0               // curtain1 close relay status flag

#define Dm1B_TRq                   GenFg_07.bit.b7               // dimmer1 box transmitting request flag
#define Dm2B_TRq                   GenFg_07.bit.b6               // dimmer2 box transmitting request flag
#define Dm3B_TRq                   GenFg_07.bit.b5               // dimmer3 box transmitting request flag
#define Dm4B_TRq                   GenFg_07.bit.b4               // dimmer4 box transmitting request flag
#define MrgRom_F                   GenFg_07.bit.b3               // merged room status flag
#define MgRm_DND                   GenFg_07.bit.b2               // merged room do not disturb status flag
#define CmLd_IRd                   GenFg_07.bit.b1               // common led initialize ready flag

#define LdEp_Tsm                   GenFg_08.bit.b7               // led expansion transmitting flag
#define LdEp_TRq                   GenFg_08.bit.b6               // led expansion transmitting request flag
#define SrBI_RCp                   GenFg_08.bit.b5               // sharing bus input received complete flag
#define SrBI_Prc                   GenFg_08.bit.b4               // sharing bus input processing flag
#define SrBO_Tsm                   GenFg_08.bit.b3               // sharing bus output transmitting flag
#define SrSvS_TR                   GenFg_08.bit.b2               // sharing service status transmitting request flag
#define SrCcS_TR                   GenFg_08.bit.b1               // sharing circuit status transmitting request flag
#define SrSte_TR                   GenFg_08.bit.b0               // sharing state transmitting request flag

#define EtDrOp_S                   GenFg_09.bit.b7               // entrance door opened status flag
#define EtDrCh_S                   GenFg_09.bit.b6               // entrance door changed status flag
#define RomMtD_S                   GenFg_09.bit.b5               // room motion detected status flag
#define Sm1_Stay                   GenFg_09.bit.b4               // someone stays flag
#define RMtOc_Sh                   GenFg_09.bit.b3               // room motion detected & occupied status showing flag
#define ORmOcI_S                   GenFg_09.bit.b2               // other rooms occupied input status flag
#define Cbn5Op_S                   GenFg_09.bit.b1               // cabinet5 opened status flag
#define Cbn5Rl_S                   GenFg_09.bit.b0               // cabinet5 relay status flag

#define CSpkRl_S                   GenFg_10.bit.b7               // chime speaker relay status flag
#define DNDSRl_S                   GenFg_10.bit.b6               // do not disturb sign relay status flag
#define MURSRl_S                   GenFg_10.bit.b5               // make up room sign relay status flag
#define PWaSRl_S                   GenFg_10.bit.b4               // please wait sign relay status flag
#define AcBDO0_F                   GenFg_10.bit.b3               // air-con balcony door open shut off mode status flag
#define A2BDO0_F                   GenFg_10.bit.b2               // air-con2 balcony door open shut off mode status flag
#define A3BDO0_F                   GenFg_10.bit.b1               // air-con3 balcony door open shut off mode status flag
#define A4BDO0_F                   GenFg_10.bit.b0               // air-con4 balcony door open shut off mode status flag

#define BcDrCs_S                   GenFg_11.bit.b7               // balcony door closed status flag
#define BDr2Cs_S                   GenFg_11.bit.b6               // balcony door2 closed status flag
#define BDr3Cs_S                   GenFg_11.bit.b5               // balcony door3 closed status flag
#define BDr4Cs_S                   GenFg_11.bit.b4               // balcony door4 closed status flag
#define ExAcRl_S                   GenFg_11.bit.b3               // external air-con relay status flag
#define EAc2Rl_S                   GenFg_11.bit.b2               // external air-con2 relay status flag
#define EAc3Rl_S                   GenFg_11.bit.b1               // external air-con3 relay status flag
#define EAc4Rl_S                   GenFg_11.bit.b0               // external air-con4 relay status flag

#define Cbn1Op_S                   GenFg_12.bit.b7               // cabinet1 opened status flag
#define Cbn1Rl_S                   GenFg_12.bit.b6               // cabinet1 relay status flag
#define Cbn2Op_S                   GenFg_12.bit.b5               // cabinet2 opened status flag
#define Cbn2Rl_S                   GenFg_12.bit.b4               // cabinet2 relay status flag
#define Cbn3Op_S                   GenFg_12.bit.b3               // cabinet3 opened status flag
#define Cbn3Rl_S                   GenFg_12.bit.b2               // cabinet3 relay status flag
#define Cbn4Op_S                   GenFg_12.bit.b1               // cabinet4 opened status flag
#define Cbn4Rl_S                   GenFg_12.bit.b0               // cabinet4 relay status flag

#define HKySSy_R                   GenFg_13.bit.b7               // have key status synchronizing request flag
#define HvKyS_Sy                   GenFg_13.bit.b6               // have key status synchronizing flag
#define MtSwSy_R                   GenFg_13.bit.b5               // master switch synchronizing request flag
#define MstSw_Sy                   GenFg_13.bit.b4               // master switch synchronizing flag
#define MtOfS_Sy                   GenFg_13.bit.b3               // master off status synchronizing flag


//-------------------------------------------
// relay circuit group format
typedef union {
#if(U_DSzRlM)
  unsigned long uintAl;
  struct {
    unsigned short us1;
    unsigned short us0;
  } uint16;
  struct {
    unsigned char uc3;
    unsigned char uc2;
    unsigned char uc1;
    unsigned char uc0;
  } uint8;
  struct {
    unsigned RcA09                 :1;       // (b24)
    unsigned RcA10                 :1;       // (b25)
    unsigned RcA11                 :1;       // (b26)
    unsigned RcA12                 :1;       // (b27)
    unsigned RcA13                 :1;       // (b28)
    unsigned RcA14                 :1;       // (b29)
    unsigned RcA15                 :1;       // (b30)
    unsigned RcA16                 :1;       // (b31)
    
    unsigned RcA01                 :1;       // (b16)
    unsigned RcA02                 :1;       // (b17)
    unsigned RcA03                 :1;       // (b18)
    unsigned RcA04                 :1;       // (b19)
    unsigned RcA05                 :1;       // (b20)
    unsigned RcA06                 :1;       // (b21)
    unsigned RcA07                 :1;       // (b22)
    unsigned RcA08                 :1;       // (b23)
    
    unsigned Rc009                 :1;       // (b8)
    unsigned Rc010                 :1;       // (b9)
    unsigned Rc011                 :1;       // (b10)
    unsigned Rc012                 :1;       // (b11)
    unsigned Rc013                 :1;       // (b12)
    unsigned Rc014                 :1;       // (b13)
    unsigned Rc015                 :1;       // (b14)
    unsigned Rc016                 :1;       // (b15)
    
    unsigned Rc001                 :1;       // (b0)
    unsigned Rc002                 :1;       // (b1)
    unsigned Rc003                 :1;       // (b2)
    unsigned Rc004                 :1;       // (b3)
    unsigned Rc005                 :1;       // (b4)
    unsigned Rc006                 :1;       // (b5)
    unsigned Rc007                 :1;       // (b6)
    unsigned Rc008                 :1;       // (b7)
  } bit;
#else
  unsigned short uintAl;
  struct {
    unsigned char uc1;
    unsigned char uc0;
  } uint8;
  struct {
    unsigned Rc009                 :1;       // (b8)
    unsigned Rc010                 :1;       // (b9)
    unsigned Rc011                 :1;       // (b10)
    unsigned Rc012                 :1;       // (b11)
    unsigned Rc013                 :1;       // (b12)
    unsigned Rc014                 :1;       // (b13)
    unsigned Rc015                 :1;       // (b14)
    unsigned Rc016                 :1;       // (b15)
    
    unsigned Rc001                 :1;       // (b0)
    unsigned Rc002                 :1;       // (b1)
    unsigned Rc003                 :1;       // (b2)
    unsigned Rc004                 :1;       // (b3)
    unsigned Rc005                 :1;       // (b4)
    unsigned Rc006                 :1;       // (b5)
    unsigned Rc007                 :1;       // (b6)
    unsigned Rc008                 :1;       // (b7)
  } bit;
#endif  //#if(U_DSzRlM)
} RlyCctGp;

#if(U_DSzRlM)
/* expand output group bit position*/
#define RcA09_BIT    ((8*0)+0)           // RLY1 circuit01 relay
#define RcA10_BIT    ((8*0)+1)           // RLY2 circuit02 relay
#define RcA11_BIT    ((8*0)+2)           // RLY3 circuit03 relay
#define RcA12_BIT    ((8*0)+3)           // RLY4 circuit04 relay
#define RcA13_BIT    ((8*0)+4)           // RLY5 circuit05 relay
#define RcA14_BIT    ((8*0)+5)           // RLY6 circuit06 relay
#define RcA15_BIT    ((8*0)+6)           // RLY7 circuit07 relay
#define RcA16_BIT    ((8*0)+7)           // RLY8 circuit08 relay

#define RcA01_BIT    ((8*1)+0)           // RLY9 circuit11 relay
#define RcA02_BIT    ((8*1)+1)           // RLY10 circuit12 relay
#define RcA03_BIT    ((8*1)+2)           // spare1
#define RcA04_BIT    ((8*1)+3)           // spare2
#define RcA05_BIT    ((8*1)+4)           // RLY11 circuit13 relay
#define RcA06_BIT    ((8*1)+5)           // RLY12 circuit14 relay
#define RcA07_BIT    ((8*1)+6)           // RLY13 circuit15 relay
#define RcA08_BIT    ((8*1)+7)           // RLY14 circuit16 relay

#define Rc009_BIT ((8*2)+0)           // RLY22 circuit10 relay
#define Rc010_BIT ((8*2)+1)           // RLY21 circuit09 relay
#define Rc011_BIT ((8*2)+2)           // RLY20 circuit17 relay
#define Rc012_BIT ((8*2)+3)           // RLY19 circuit18 relay
#define Rc013_BIT ((8*2)+4)           // RLY15 fan high speed relay
#define Rc014_BIT ((8*2)+5)           // RLY16 fan medium speed relay
#define Rc015_BIT ((8*2)+6)           // RLY17 fan low speed relay
#define Rc016_BIT ((8*2)+7)           // RLY18 valve relay
        
#define Rc001_BIT    ((8*3)+0)           // spare3
#define Rc002_BIT    ((8*3)+1)           // service panel make up room led
#define Rc003_BIT    ((8*3)+2)           // key box led
#define Rc004_BIT    ((8*3)+3)           // master panel led
#define Rc005_BIT    ((8*3)+4)           // night led
#define Rc006_BIT    ((8*3)+5)           // front panel do not disturb led
#define Rc007_BIT    ((8*3)+6)           // front panel make up room led
#define Rc008_BIT    ((8*3)+7)           // service panel do not disturb led

#else


#define Rc009_BIT    ((8*0)+0)           // RLY1 circuit01 relay
#define Rc010_BIT    ((8*0)+1)           // RLY2 circuit02 relay
#define Rc011_BIT    ((8*0)+2)           // RLY3 circuit03 relay
#define Rc012_BIT    ((8*0)+3)           // RLY4 circuit04 relay
#define Rc013_BIT    ((8*0)+4)           // RLY5 circuit05 relay
#define Rc014_BIT    ((8*0)+5)           // RLY6 circuit06 relay
#define Rc015_BIT    ((8*0)+6)           // RLY7 circuit07 relay
#define Rc016_BIT    ((8*0)+7)           // RLY8 circuit08 relay
            
#define Rc001_BIT    ((8*1)+0)           // RLY9 circuit11 relay
#define Rc002_BIT    ((8*1)+1)           // RLY10 circuit12 relay
#define Rc003_BIT    ((8*1)+2)           // spare1
#define Rc004_BIT    ((8*1)+3)           // spare2
#define Rc005_BIT    ((8*1)+4)           // RLY11 circuit13 relay
#define Rc006_BIT    ((8*1)+5)           // RLY12 circuit14 relay
#define Rc007_BIT    ((8*1)+6)           // RLY13 circuit15 relay
#define Rc008_BIT    ((8*1)+7)           // RLY14 circuit16 relay

#endif  //#if(U_DSzRlM)

 
 
 
 
 
 
 
 

 
 
 
 
 
 
 
 









//---------------------PARAMETER TAB----------------------//

typedef union {
  //unsigned char array[EE_BlockSze01-1];
  struct {
                                                                                // @@@ start of array
    MbtUint8 VitFg_A1;                       // vital flag A1
    #define Key_Stay               ParaTb01.P.VitFg_A1.bit.b7    // key stay flag (with delay)
    #define DNDstb_S               ParaTb01.P.VitFg_A1.bit.b6    // do not disturb status flag
    #define MkUpRm_S               ParaTb01.P.VitFg_A1.bit.b5    // make up room status flag
    #define AirCon_S               ParaTb01.P.VitFg_A1.bit.b4    // air-con status flag
    #define LvR_AcPw               ParaTb01.P.VitFg_A1.bit.b3    // leave room air-con power
    #define BDrOp_WD               ParaTb01.P.VitFg_A1.bit.b2    // balcony door opened with delay status flag
    #define RTC_Fail               ParaTb01.P.VitFg_A1.bit.b1    // clock fail flag
    #define AlmCk_En               ParaTb01.P.VitFg_A1.bit.b0    // alarm clock enable flag

    MbtUint8 VitFg_A2;                       // vital flag A2
    #define PeWlcm_F               ParaTb01.P.VitFg_A2.bit.b7    // pre-welcome flag
    #define TAcnAc_F               ParaTb01.P.VitFg_A2.bit.b6    // temporary air-con accessing flag
    #define Ct1OTu_S               ParaTb01.P.VitFg_A2.bit.b5    // curtain1 open turning direction status flag
    #define LgScMc_O               ParaTb01.P.VitFg_A2.bit.b4    // lighting scene matching occur flag
    #define BDr2O_WD               ParaTb01.P.VitFg_A2.bit.b3    // balcony door2 opened with delay status flag
    #define KBWEDM_S               ParaTb01.P.VitFg_A2.bit.b2    // key box working with entrance door & motion sensor status flag
    #define PlWait_S               ParaTb01.P.VitFg_A2.bit.b1    // please wait status flag

    MbtUint8 VitFg_A3;                       // vital flag A3
    #define BDr3O_WD               ParaTb01.P.VitFg_A3.bit.b7    // balcony door3 opened with delay status flag
    #define BDr4O_WD               ParaTb01.P.VitFg_A3.bit.b6    // balcony door4 opened with delay status flag
    #define AlCAct_F               ParaTb01.P.VitFg_A3.bit.b5    // alarm clock active flag
    #define AlCSnz_F               ParaTb01.P.VitFg_A3.bit.b4    // alarm clock snooze flag
    #define AcNMt0_F               ParaTb01.P.VitFg_A3.bit.b3    // air-con no-motion shut off mode status flag
    #define A2NMt0_F               ParaTb01.P.VitFg_A3.bit.b2    // air-con2 no-motion shut off mode status flag
    #define ReGuSM_R               ParaTb01.P.VitFg_A3.bit.b1    // restoring guest setting memory request flag
    #define EMtWSc_R               ParaTb01.P.VitFg_A3.bit.b0    // entrance motion welcome scene request flag

    MbtUint8 VitFg_A4;                       // vital flag A4
    #define EDrEv_CC               ParaTb01.P.VitFg_A4.bit.b7    // entrance door was ever changed to close status flag
    #define HvKey_St               ParaTb01.P.VitFg_A4.bit.b6    // have key status flag (no delay)
    #define SbAA0_TR               ParaTb01.P.VitFg_A4.bit.b5    // standby air-con auto shut off time reach status flag
    #define SbAcPw_S               ParaTb01.P.VitFg_A4.bit.b4    // standby air-con power status flag
    #define SbHmAt_S               ParaTb01.P.VitFg_A4.bit.b3    // standby humidity active status flag

    //---------------------------------------
    unsigned char Arcn_Mod;                  // air-con operating mode
    #define AcMd_Fan               0                             // fan mode
    #define AcMd_Col               1                             // cooling mode
    #define AcMd_Dry               2                             // dry mode
    #define AcMd_Het               3                             // heating mode
    #define AcMd_Aut               4                             // auto mode

    #define Mx_AcMod               1                             // maximun number of air-con operating mode
    //---------------------------------------
    unsigned char Fan_Sped;                  // fan speed
    #define Fan__Low               0                             // fan low speed
    #define Fan__Med               1                             // fan medium speed
    #define Fan_High               2                             // fan high speed
    #define Fan_Auto               3                             // fan auto speed
    
    #define Mx_FanSp               3                             // maximun number of fan speed
    //---------------------------------------
    unsigned char Ac_SetTp;                  // air-con setting temperature index
    unsigned char Random_V;                  // random value
    RlyCctGp RlCc_Dat;                       // relay circuit data
    RlyCctGp LvRm_LpC;                       // leave room lamp circuit status
    RlyCctGp BMOf_LpC;                       // before master off lamp circuit status
    unsigned char LvR_AcMd;                  // leave room air-con operating mode
    unsigned char LvR_FnSp;                  // leave room fan speed
    unsigned char LvR_StTp;                  // leave room setting temperature
//    struct Dim_GrpFmt Dimm_Sta;              // dimmer status
//    DimChnGp LvRm_DmC;                       // leave room dimmer channel status
//    DimChnGp BMOf_DmC;                       // before master off dimmer channel status
    //---------------------------------------
    unsigned char RmOcpy_M;                  // room occupy mode
    #define RO_ChkIn               0                             // check in
    #define RO_CkOut               1                             // check out
    #define RO_Mnten               2                             // maintenance
    #define RO_LwSsn               3                             // low season

    #define Mx_ROcpM               1                             // maximun number of room occupy mode
    //---------------------------------------
    unsigned char Late_KyC;                  // latest key card
//    Clock_format  Alrm_Clk;                  // alarm clock time
    //---------------------------------------
    unsigned char RmMtDt_M;                  // room motion detecting mode
    #define RM_ROcpy               0                             // room occupied
    #define RM_ROcCf               1                             // room occupied confirmation
    #define RM_RUocp               2                             // room unoccupied
    #define RM_RUoCf               3                             // room unoccupied confirmation
    //---------------------------------------
                                                                                // @@@ end of array
  } P;
} Typedef_ParaTb01;                                  // parameter table 1
//-------------------------------------------




typedef union {
 // unsigned char array[EE_BlockSze2a-1];
  struct {
                                                                                // @@@ start of array
    MbtUint8 VitFg_01;                       // vital flag #1
    #define W1APwM_F               ParaTb2a.P.VitFg_01.bit.b7    // welcome1 air-con power memory flag
    #define W2APwM_F               ParaTb2a.P.VitFg_01.bit.b6    // welcome2 air-con power memory flag
    #define W1AMdM_F               ParaTb2a.P.VitFg_01.bit.b5    // welcome1 air-con operating mode memory flag
    #define W2AMdM_F               ParaTb2a.P.VitFg_01.bit.b4    // welcome2 air-con operating mode memory flag
    #define W1STpM_F               ParaTb2a.P.VitFg_01.bit.b3    // welcome1 setting temperature memory flag
    #define W2STpM_F               ParaTb2a.P.VitFg_01.bit.b2    // welcome2 setting temperature memory flag
    #define W1FSpM_F               ParaTb2a.P.VitFg_01.bit.b1    // welcome1 fan speed memory flag
    #define W2FSpM_F               ParaTb2a.P.VitFg_01.bit.b0    // welcome2 fan speed memory flag

    MbtUint8 VitFg_02;                       // vital flag #2
    #define W1APw_Ps               ParaTb2a.P.VitFg_02.bit.b7    // welcome1 air-con power flag for preset
    #define W2APw_Ps               ParaTb2a.P.VitFg_02.bit.b6    // welcome2 air-con power flag for preset
    #define PeWc_APw               ParaTb2a.P.VitFg_02.bit.b5    // pre-welcome air-con power flag
    #define DmDIdD_E               ParaTb2a.P.VitFg_02.bit.b4    // dimming display idle delay enable flag
    #define T0DBID_E               ParaTb2a.P.VitFg_02.bit.b3    // turning off display backlight idle delay enable flag
    #define DBB0ID_E               ParaTb2a.P.VitFg_02.bit.b2    // turning off display button backlight idle delay enable flag
    #define T0DIdD_E               ParaTb2a.P.VitFg_02.bit.b1    // turning off display idle delay enable flag
    #define T0DNKD_E               ParaTb2a.P.VitFg_02.bit.b0    // turning off display no-key delay enable flag

    MbtUint8 VitFg_03;                       // vital flag #3
    #define AcnMdu_E               ParaTb2a.P.VitFg_03.bit.b7    // air-con module enable flag
    #define EAcnRl_E               ParaTb2a.P.VitFg_03.bit.b6    // external air-con relay enable flag
    #define LStBCt_E               ParaTb2a.P.VitFg_03.bit.b5    // limit setting temperature by control setting temperature limit enable flag
    #define ACoHyT_E               ParaTb2a.P.VitFg_03.bit.b4    // air-con colder hysteresis temperature enable flag
    #define FnAuSp_E               ParaTb2a.P.VitFg_03.bit.b3    // fan auto speed enable flag
    #define NS_AcSTp               ParaTb2a.P.VitFg_03.bit.b2    // normally show air-con setting temperature flag
    #define TA0FCl_E               ParaTb2a.P.VitFg_03.bit.b1    // force cooling mode when turning air-con off enable flag
    #define VRV2CR_E               ParaTb2a.P.VitFg_03.bit.b0    // VRV 2-dry contact relay enable flag

    MbtUint8 VitFg_04;                       // vital flag #4
    #define StbyTp_E               ParaTb2a.P.VitFg_04.bit.b7    // standby temperature enable flag
    #define StbyHm_E               ParaTb2a.P.VitFg_04.bit.b6    // standby humidity enable flag
    #define SbAPw_F1               ParaTb2a.P.VitFg_04.bit.b5    // standby air-con power force on flag (0=maintain,1=force on)
    #define SbAFP_F1               ParaTb2a.P.VitFg_04.bit.b4    // standby air-con fan power force on flag (0=as valve,1=force on)
    #define SbAcA0_E               ParaTb2a.P.VitFg_04.bit.b3    // standby air-con auto shut off enable flag
    #define BcnDrS_E               ParaTb2a.P.VitFg_04.bit.b2    // balcony door sensor enable flag
    #define BcDrS_NC               ParaTb2a.P.VitFg_04.bit.b1    // balcony door sensor type flag (0=NO: O/C when door opened)
                                                                 //                               (1=NC: C/C when door opened)
    #define BDOFP_F0               ParaTb2a.P.VitFg_04.bit.b0    // balcony door open fan power force off flag (0=maintain,1=force off)

    MbtUint8 VitFg_05;                       // vital flag #5
    #define MtPSw_Tg               ParaTb2a.P.VitFg_05.bit.b7    // master panel switch type flag (0=push button,1=toggle)
    #define MtPLd_EL               ParaTb2a.P.VitFg_05.bit.b6    // master panel led type flag (0=energize high,1=energize low)
    #define NKMstS_E               ParaTb2a.P.VitFg_05.bit.b5    // no-key master switch enable flag
    #define MstPSw_E               ParaTb2a.P.VitFg_05.bit.b4    // master panel switch enable flag
    #define FrPLd_EL               ParaTb2a.P.VitFg_05.bit.b3    // front panel led type flag (0=energize high,1=energize low)
    #define ACcDND_E               ParaTb2a.P.VitFg_05.bit.b2    // automatic cancel do not disturb enable flag
    #define NKChmS_E               ParaTb2a.P.VitFg_05.bit.b1    // no-key chime switch enable flag
    #define CSpkRl_E               ParaTb2a.P.VitFg_05.bit.b0    // chime speaker relay enable flag

    MbtUint8 VitFg_06;                       // vital flag #6
    #define SvPSw_FP               ParaTb2a.P.VitFg_06.bit.b7    // service panel switch type flag (0=push button,1=fixed on/off position)
    #define SDNDL_EL               ParaTb2a.P.VitFg_06.bit.b6    // service panel do not disturb led type flag (0=energize high,1=energize low)
    #define SMURL_EL               ParaTb2a.P.VitFg_06.bit.b5    // service panel make up room led type flag (0=energize high,1=energize low)
    #define NKSevS_E               ParaTb2a.P.VitFg_06.bit.b4    // no-key service switch enable flag
    #define DNDSRl_E               ParaTb2a.P.VitFg_06.bit.b3    // do not disturb sign relay enable flag
    #define MURSRl_E               ParaTb2a.P.VitFg_06.bit.b2    // make up room sign relay enable flag
    #define AAcOCO_E               ParaTb2a.P.VitFg_06.bit.b1    // allow air-con operate when check out enable flag
    #define TAcnAc_E               ParaTb2a.P.VitFg_06.bit.b0    // temporary air-con accessing enable flag

    MbtUint8 VitFg_07;                       // vital flag #7
    #define EmrLig_E               ParaTb2a.P.VitFg_07.bit.b7    // emergency lighting enable flag
    #define PwFSCc_E               ParaTb2a.P.VitFg_07.bit.b6    // power fail saving circuit enable flag
    #define Cb1Sw_NC               ParaTb2a.P.VitFg_07.bit.b5    // cabinet1 switch type flag (0=NO: O/C when cabinet opened or motion detected)
                                                                 //                           (1=NC: C/C when cabinet opened or motion detected)
    #define Cbn1Sw_E               ParaTb2a.P.VitFg_07.bit.b4    // cabinet1 switch enable flag
    #define Cbn2Sw_E               ParaTb2a.P.VitFg_07.bit.b3    // cabinet2 switch enable flag
    #define Cbn3Sw_E               ParaTb2a.P.VitFg_07.bit.b2    // cabinet3 switch enable flag
    #define Cbn4Sw_E               ParaTb2a.P.VitFg_07.bit.b1    // cabinet4 switch enable flag
    #define Cbn5Sw_E               ParaTb2a.P.VitFg_07.bit.b0    // cabinet5 switch enable flag

    MbtUint8 VitFg_08;                       // vital flag #8
    #define NKMSLL_E               ParaTb2a.P.VitFg_08.bit.b7    // no-key matching scene lighting led enable flag
    #define McScLO_E               ParaTb2a.P.VitFg_08.bit.b6    // matching scene lighting output enable flag
    #define StbyCc_E               ParaTb2a.P.VitFg_08.bit.b5    // standby circuit enable flag
    #define KyIBDr_E               ParaTb2a.P.VitFg_08.bit.b4    // key switch including balcony door sensorK enable flag
    #define AcPwFS_E               ParaTb2a.P.VitFg_08.bit.b3    // air-con power fail saving enable flag
    #define IRCBsO_E               ParaTb2a.P.VitFg_08.bit.b2    // infrared remote code bus output enable flag
    #define EAcPBA_E               ParaTb2a.P.VitFg_08.bit.b1    // external air-con power button accessing enable flag
    #define FChmSw_E               ParaTb2a.P.VitFg_08.bit.b0    // front panel chime switch enable flag

    MbtUint8 VitFg_09;                       // vital flag #9
    #define EtDrS_NC               ParaTb2a.P.VitFg_09.bit.b7    // entrance door sensor type flag (0=NO: O/C when door opened)
                                                                 //                                (1=NC: C/C when door opened)
    #define PmnROc_E               ParaTb2a.P.VitFg_09.bit.b6    // permanent room occupied enable flag
    #define EtMtWc_E               ParaTb2a.P.VitFg_09.bit.b5    // entrance motion welcome enable flag
    #define ROcSSr_E               ParaTb2a.P.VitFg_09.bit.b4    // room occupied status sharing enable flag
    #define KBWEDr_E               ParaTb2a.P.VitFg_09.bit.b3    // key box working with entrance door sensor enable flag
    #define LcGMT_Sg               ParaTb2a.P.VitFg_09.bit.b2    // local UTC/GMT offset signed flag
    #define TmZnTm_E               ParaTb2a.P.VitFg_09.bit.b1    // time zone time enable flag
    #define SrBcRO_E               ParaTb2a.P.VitFg_09.bit.b0    // sharing bus broadcast receiving only enable flag

    MbtUint8 VitFg_10;                       // vital flag #10
    #define EAc2Rl_E               ParaTb2a.P.VitFg_10.bit.b7    // external air-con2 relay enable flag
    #define EAc3Rl_E               ParaTb2a.P.VitFg_10.bit.b6    // external air-con3 relay enable flag
    #define EAc4Rl_E               ParaTb2a.P.VitFg_10.bit.b5    // external air-con4 relay enable flag
    #define BcDr2S_E               ParaTb2a.P.VitFg_10.bit.b4    // balcony door2 sensor enable flag
    #define BcDr3S_E               ParaTb2a.P.VitFg_10.bit.b3    // balcony door3 sensor enable flag
    #define BcDr4S_E               ParaTb2a.P.VitFg_10.bit.b2    // balcony door4 sensor enable flag
    #define LdEBsO_E               ParaTb2a.P.VitFg_10.bit.b1    // led expansion bus output enable flag
    #define EtMtLg_E               ParaTb2a.P.VitFg_10.bit.b0    // entrance motion light enable flag

    MbtUint8 VitFg_11;                       // vital flag #11
    #define AlCSnz_E               ParaTb2a.P.VitFg_11.bit.b7    // alarm clock snooze enable flag
    #define AuLgOf_E               ParaTb2a.P.VitFg_11.bit.b6    // automatic light off enable flag
    #define AuL0MS_E               ParaTb2a.P.VitFg_11.bit.b5    // automatic light off motion sensor enable flag
    #define Dm4BsO_E               ParaTb2a.P.VitFg_11.bit.b4    // dimmer4 bus output enable flag
    #define Dm3BsO_E               ParaTb2a.P.VitFg_11.bit.b3    // dimmer3 bus output enable flag
    #define Dm2BsO_E               ParaTb2a.P.VitFg_11.bit.b2    // dimmer2 bus output enable flag
    #define Dm1BsO_E               ParaTb2a.P.VitFg_11.bit.b1    // dimmer1 bus output enable flag

    MbtUint8 VitFg_12;                       // vital flag #12
    #define AcNMt0_E               ParaTb2a.P.VitFg_12.bit.b7    // air-con no-motion shut off enable flag
    #define A2NMt0_E               ParaTb2a.P.VitFg_12.bit.b6    // air-con2 no-motion shut off enable flag
    #define Curtn1_E               ParaTb2a.P.VitFg_12.bit.b5    // curtain1 enable flag
    #define ExhFan_E               ParaTb2a.P.VitFg_12.bit.b4    // exhaust fan enable flag
    #define SDNDSw_E               ParaTb2a.P.VitFg_12.bit.b3    // service panel do not disturb switch enable flag
    #define SMURSw_E               ParaTb2a.P.VitFg_12.bit.b2    // service panel make up room switch enable flag
    #define SPWaSw_E               ParaTb2a.P.VitFg_12.bit.b1    // service please wait switch enable flag
    #define PWaSRl_E               ParaTb2a.P.VitFg_12.bit.b0    // please wait sign relay enable flag

    MbtUint8 VitFg_13;                       // vital flag #13
    #define MtnLig_E               ParaTb2a.P.VitFg_13.bit.b7    // motion light enable flag
    #define MLgARs_E               ParaTb2a.P.VitFg_13.bit.b6    // motion light timer always restart enable flag
    //---------------------------------------
                                                                                // @@@ end of array
  } P;
} Typedef_ParaTb2a;                                  // parameter table 2a




typedef union {
  //unsigned char array[EE_BlockSze2b-1];
  struct {
    //unsigned char ExAcn_RN;                  // external air-con relay circuit no.
                                                                               // @@@ start of array
    unsigned char AlCkDu_P;                  // alarm clock duration parameter (m)
    #define AlCkDu_L               1                             // lower limit
    #define AlCkDu_U               15                            // upper limit
    //---------------------------------------
    unsigned char ACSnzT_P;                  // alarm clock snooze time (m)
    #define ACSnzT_L               1                             // lower limit
    #define ACSnzT_U               30                            // upper limit
    //---------------------------------------
    unsigned char AlTone_P;                  // alarm tone (0=off, tone1-10)
    #define AlTone_L               0                             // lower limit
    #define AlTone_U               10                            // upper limit
    //---------------------------------------
//    Clock_format  LcGMT_Os;                  // local UTC/GMT offset time
    //---------------------------------------
//    Clock_format  DaySr_Tm;                  // day start time
//    Clock_format  NghSr_Tm;                  // night start time
    //---------------------------------------
    unsigned char KBxLd_Op;                  // key box led operation
    unsigned char MtPLd_Op;                  // master panel led operation
    unsigned char SDNDL_Op;                  // service panel do not disturb led operation
    unsigned char SMURL_Op;                  // service panel make up room led operation
                                                                 // ptrn01_I to Patrn_ID
    #define L4_McScL               (Patrn_ID+1)                  // matching scene lighting led
    #define L4_OwnFt               (Patrn_ID+2)                  // own function led
    #define L4_KySta               (Patrn_ID+3)                  // key status led
    #define L4_KyAct               (Patrn_ID+4)                  // key active led
    #define L4_MstOf               (Patrn_ID+5)                  // master off led
    #define L4_DNDst               (Patrn_ID+6)                  // do not disturb led
    #define L4_MkURm               (Patrn_ID+7)                  // make up room led
    #define L4_PWait               (Patrn_ID+8)                  // please wait led
    
    #define SwPLdO_U               (Patrn_ID+8)                  // upper limit of switch panel led operation

    #define L4_CtmBs               (SwPLdO_U+1)                  // custom bus
    //---------------------------------------
                                             // local switch type data(10) : 00 = push button
                                             //                              01 = toggle switch
                                             //                              10 = fixed on/off position
                                             //                              11 = BAS input (changeover on/off position)
 //   LocSwGrp LSwTy_D1;                       // local switch type data1
 //   LocSwGrp LSwTy_D0;                       // local switch type data0
 //   LocSwGrp NK_LSwAc;                       // no-key local switch accessing
    //---------------------------------------
    unsigned char W1AMd_Ps;                  // welcome1 air-con operating mode for preset
    unsigned char W2AMd_Ps;                  // welcome2 air-con operating mode for preset
    unsigned char W1FSp_Ps;                  // welcome1 fan speed for preset
    unsigned char W2FSp_Ps;                  // welcome2 fan speed for preset
    unsigned char W1STp_Ps;                  // welcome1 setting temperature for preset
    unsigned char W2STp_Ps;                  // welcome2 setting temperature for preset
    //---------------------------------------
    unsigned char PeWc_AMd;                  // pre-welcome air-con operating mode
    unsigned char PeWc_FSp;                  // pre-welcome fan speed
    unsigned char PeWc_STp;                  // pre-welcome setting temperature
    //---------------------------------------
    unsigned char RlCSpD_P;                  // relay port changed separation delay parameter (x10ms)(10-1000ms, 10ms/step)
    #define RlCSpD_L               1                   // lower limit
    #define RlCSpD_U               100                 // upper limit
    //---------------------------------------
    unsigned char SwCCLv_P;                  // switch close circuit level (0-100 for 1.00-2.00V)(ex. 90 -> 1.90V)
    #define SwCCLv_L               0                   // lower limit
    #define SwCCLv_U               100                 // upper limit
    //---------------------------------------
    unsigned char DmMn_PcB;                  // dimmer minimum %brightness
    #define DMnPcB_L               1                   // lower limit
    #define DMnPcB_U               10                  // upper limit
    //---------------------------------------
    unsigned char Epd5P_Op;                  // expand5 port operation
    #define E5_MdVal               0                             // modulating valve
    #define E5_CtmBs               1                             // custom bus
    #define E5_SPI2S               2                             // SPI bus slave2 select
    
    #if(U_MdVaPI)
      #define Ep5POp_L             0                             // lower limit
    #else
      #define Ep5POp_L             1                             // lower limit
    #endif
    
    #if(U_SPIBus&&U_SPI2Sl)
      #define Ep5POp_U             2                             // upper limit
    #else
      #define Ep5POp_U             1                             // upper limit
    #endif
    //---------------------------------------
    unsigned char IRCBO_PN;                  // infrared remote code bus output port no.
    unsigned char LdEBO_PN;                  // led expansion bus output port no.
    #define Ct01_BsP               0                             // custom1 bus port
    #define Ct02_BsP               1                             // custom2 bus port
    #define Ct03_BsP               2                             // custom3 bus port
    #define Ct04_BsP               3                             // custom4 bus port
    #define Ct05_BsP               4                             // custom5 bus port
    #define Ct06_BsP               5                             // custom6 bus port
    #define Ct07_BsP               6                             // custom7 bus port
    
    #if(RCUM_PCB == iS1K6P_M)
      #define CtmBsP_L             0                             // lower limit
      #define CtmBsP_U             6                             // upper limit
    #elif(RCUM_PCB == iS1K6__M)
      #if(U_SPIBus)
        #define CtmBsP_L           3                             // lower limit
      #else
        #define CtmBsP_L           0                             // lower limit
      #endif
      #define CtmBsP_U             3                             // upper limit
    #elif(RCUM_PCB == iS1K4__M)
      #define CtmBsP_L             0                             // lower limit
      #define CtmBsP_U             0                             // upper limit
    #endif
    //---------------------------------------
    unsigned char MnCtST_P;                  // minimum control setting temperature parameter
    unsigned char MxCtST_P;                  // maximum control setting temperature parameter
    #define SetTmp_L               TbI__15C                      // lower limit
    #define SetTmp_U               TbI__30C                      // upper limit
    //---------------------------------------
    unsigned char CmpOfD_P;                  // compressor off delay parameter (m)
    #define CmpOfD_L               0                             // lower limit
    #define CmpOfD_U               10                            // upper limit
    //---------------------------------------
    signed char TpSCbO_P;                    // temperature sensor calibration offset (signed 0.5C)
    //---------------------------------------
    unsigned char AcMd_Mdl;                  // air-con operating mode model
    #define AcM_ClOl               0                             // cooling only
    #define AcM_FnCl               1                             // fan, cooling

    #define Mx_AMdMo               1                             // maximun number of air-con operating mode model
    //---------------------------------------
    unsigned char MVPcKp_P;                  // modulating valve %Kp (1-100%)
    #define MVPcKp_L               1                             // lower limit
    #define MVPcKp_U               100                           // upper limit
    //---------------------------------------
    unsigned char MV10Ti_P;                  // modulating valve 10xTi (x10s)(10-400s, 10s/step)
    #define MV10Ti_L               1                             // lower limit
    #define MV10Ti_U               40                            // upper limit
    //---------------------------------------
    unsigned char MVCOUd_P;                  // modulating valve control output update period (s)(2-100s)
    #define MVCOUd_L               2                             // lower limit
    #define MVCOUd_U               100                           // upper limit
    //---------------------------------------
    unsigned char DmDIdD_P;                  // dimming display idle delay parameter (s)
    #define DmDIdD_U               120                           // upper limit
    //---------------------------------------
    unsigned char T0DBID_P;                  // turning off display backlight idle delay (s)
    #define T0DBID_U               120                           // upper limit
    //---------------------------------------
    unsigned char T0DIdD_P;                  // turning off display idle delay parameter (s)
    #define T0DIdD_L               1                             // lower limit
    #define T0DIdD_U               120                           // upper limit
    //---------------------------------------
    unsigned char T0DNKD_P;                  // turning off display no-key delay parameter (m)
    #define T0DNKD_U               60                            // upper limit
    //---------------------------------------
    unsigned char M1DKAT_P;                  // maintaining on display key-action time parameter (m)
    #define M1DKAT_U               60                            // upper limit
    //---------------------------------------
    unsigned char KeyBx_Ty;                  // key box type
    #define KyBx_NOC               0                             // normally open contact when no key
    #define KyBx_NCC               1                             // normally close contact when no key
    #define KyBx_BtS               2                             // push button switch
    #define KyBx_TgS               3                             // toggle switch
    #define KyBx_3Dg               4                             // 3-bit digital (class of service)
    #define KyBx_3An               5                             // 3-bit analog voltage (class of service)
    #define KyBx_EDS               6                             // entrance door sensor (with room motion sensor)

    #define KyBxTy_U               6                             // upper limit
    //---------------------------------------
    unsigned char NoKeyD_P;                  // no-key delay parameter (s)
    #define NoKeyD_U               90                            // upper limit
    MbtUint8 GusC_BrC;                       // guest card bar code (bit210 = bar123)(light=0,dark=1)
    MbtUint8 MadC_BrC;                       // maid card bar code (bit210 = bar123)(light=0,dark=1)
    unsigned char EtDrS_SN;                  // entrance door sensor local switch no.
    //---------------------------------------
    unsigned char RUocpT_P;                  // room unoccupied time (m)
    #define RUocpT_L               1                             // lower limit
    #define RUocpT_U               180                           // upper limit
    unsigned char RmMtS_SN;                  // room motion sensor local switch no.
    unsigned char ORmOc_SN;                  // other rooms occupied input local switch no.
    unsigned char MtRsAT_P;                  // motion sensor restart avoid time (s)
    #define MtRsAT_L               0                             // lower limit
    #define MtRsAT_U               180                           // upper limit
    unsigned char MAADCT_P;                  // motion checking avoid time after entrance door closing (s)
    #define MAADCT_L               0                             // lower limit
    #define MAADCT_U               180                           // upper limit
    unsigned char MtDCfT_P;                  // motion detected confirmed time (20ms)
    #define MtDCfT_L               4                             // lower limit (4 x20ms = 80ms)(must >= 80ms)
    #define MtDCfT_U               150                           // upper limit (150 x20ms = 3000ms)
    unsigned char UoMDCT_P;                  // unoccupied motion detected confirmed time (20ms)
    #define UoMDCT_L               5                             // lower limit (5 x20ms = 100ms)(must > 80ms)
    #define UoMDCT_U               150                           // upper limit (150 x20ms = 3000ms)
    unsigned char AcTSSD_P;                  // air-con setting temperature saving step delay (m)
    #define AcTSSD_L               1                             // lower limit
    #define AcTSSD_U               90                            // upper limit
    #define AcTS_TPS               (2*(0)+1)                     // air-con setting temperature saving temperature per step (0.5C)
    unsigned char AcTSOL_P;                  // air-con setting temperature saving offset limit (0.5C)
    #define AcTSOL_L               (2*(0)+0)                     // lower limit
    #define AcTSOL_U               (2*(9)+0)                     // upper limit
    //---------------------------------------
    unsigned char MstPn_SN;                  // master panel local switch no.
    //---------------------------------------
    unsigned char CmSpk_RN;                  // chime speaker relay circuit no.
    unsigned char FrChm_SN;                  // front panel chime local switch no.
    //---------------------------------------
    unsigned char DNDSg_RN;                  // do not disturb sign relay circuit no.
    unsigned char MURSg_RN;                  // make up room sign relay circuit no.
    unsigned char PWaSg_RN;                  // please wait sign relay circuit no.
    unsigned char SvDND_SN;                  // service panel do not disturb local switch no.
    unsigned char SvMUR_SN;                  // service panel make up room local switch no.
    unsigned char SvPWa_SN;                  // service panel please wait local switch no.
    //---------------------------------------
    unsigned char PwODt_Sc;                  // power outage detector source
    #if(RCUM_PCB == iS1K6P_M)||(RCUM_PCB == iS1K6__M)
      #define No_PwrDt             0                             // no power detector
      #define RCU_NLnD             1                             // RCU normal line detector
      #define Bdct_ATS             2                             // broadcast ATS (automatic transfer switch)
      
      #if(U_485NwB&&Us_BcATS)
        #define PwODSc_U           2                             // upper limit
      #else
        #define PwODSc_U           1                             // upper limit
      #endif
    #elif(RCUM_PCB == iS1K4__M)
      #define No_PwrDt             0                             // no power detector
      #define Bdct_ATS             1                             // broadcast ATS (automatic transfer switch)
      #define RCU_NLnD             2                             // RCU normal line detector
      
      #if(U_485NwB&&Us_BcATS)
        #define PwODSc_U           1                             // upper limit
      #else
        #define PwODSc_U           0                             // upper limit
      #endif
    #endif
    //---------------------------------------
    unsigned char Cb1CsD_P;                  // cabinet1 closed delay parameter (s)
    #define CbnCsD_L               1                             // lower limit
    #define CbnCsD_U               60                            // upper limit
    unsigned char Cbnt1_SN;                  // cabinet1 local switch no.
    unsigned char Cbnt1_RN;                  // cabinet1 relay circuit no.
    unsigned char Cbnt2_SN;                  // cabinet2 local switch no.
    unsigned char Cbnt2_RN;                  // cabinet2 relay circuit no.
    unsigned char Cbnt3_SN;                  // cabinet3 local switch no.
    unsigned char Cbnt3_RN;                  // cabinet3 relay circuit no.
    unsigned char Cbnt4_SN;                  // cabinet4 local switch no.
    unsigned char Cbnt4_RN;                  // cabinet4 relay circuit no.
    unsigned char Cbnt5_SN;                  // cabinet5 local switch no.
    unsigned char Cbnt5_RN;                  // cabinet5 relay circuit no.
    //---------------------------------------
    unsigned char Cut1O_RN;                  // curtain1 open relay circuit no.
    unsigned char Cut1C_RN;                  // curtain1 close relay circuit no.
    unsigned char Cut1O_SN;                  // curtain1 open local switch no.
    unsigned char Cut1C_SN;                  // curtain1 close local switch no.
    //---------------------------------------
    unsigned char Ct1PTu_P;                  // curtain1 pressing turning time parameter (s)
    #define CutPTu_L               1                             // lower limit
    #define CutPTu_U               180                           // upper limit
    //---------------------------------------
    unsigned char Ct1ChP_P;                  // curtain1 change direction protection delay parameter (s)
    #define CutChP_L               0                             // lower limit
    #define CutChP_U               10                            // upper limit
    //---------------------------------------
    unsigned char Stby_Tmp;                  // standby temperature index
    unsigned char Stby_Hum;                  // standby humidity (%RH)
    #define StbHum_L               30                            // lower limit
    #define StbHum_U               90                            // upper limit
    unsigned char Sb_HumHy;                  // standby humidity hysteresis (%RH)
    #define SbHmHy_L                0                            // lower limit
    #define SbHmHy_U               30                            // upper limit
    unsigned char SbHm_Tmp;                  // standby humidity temperature index
    unsigned char SbHm_FSp;                  // standby humidity fan speed
    unsigned char SbAI1T_P;                  // standby air-con interval cycle on time (m)(1-240m)
    #define SbAI1T_L               1                             // lower limit
    #define SbAI1T_U               240                           // upper limit
    unsigned char SbAI0T_P;                  // standby air-con interval cycle off time (m)(0-240m)
    #define SbAI0T_U               240                           // upper limit
    unsigned char SbAA0T_P;                  // standby air-con auto shut off time (h)(1-24h)
    #define SbAA0T_L               1                             // upper limit
    #define SbAA0T_U               24                            // upper limit
    //---------------------------------------
    unsigned char BcDrOM_P;                  // balcony door open delay parameter (m)
    unsigned char BcDrOS_P;                  // balcony door open delay parameter (s)
    #define BcDrOM_U               10                            // upper limit (m)
    #define BcDrOS_U               59                            // upper limit (s)
    unsigned char ABDO0D_P;                  // air-con balcony door open shut off delay time parameter (m)
    #define ABDO0D_U               10                            // upper limit
    //---------------------------------------
    unsigned char ANMt0T_P;                  // air-con no-motion shut off time (m)
    #define ANMt0T_L               1                             // lower limit
    #define ANMt0T_U               180                           // upper limit
    unsigned char AcMtS_SN;                  // air-con motion sensor local switch no.
    //---------------------------------------
    unsigned char ExAcn_RN;                  // external air-con relay circuit no.
    //---------------------------------------
    unsigned char ExAc2_RN;                  // external air-con2 relay circuit no.
    unsigned char BcDr2_SN;                  // balcony door2 sensor local switch no.
    unsigned char A2MtS_SN;                  // air-con2 motion sensor local switch no.
    //---------------------------------------
    unsigned char ExAc3_RN;                  // external air-con3 relay circuit no.
    unsigned char BcDr3_SN;                  // balcony door3 sensor local switch no.
    //---------------------------------------
    unsigned char ExAc4_RN;                  // external air-con4 relay circuit no.
    unsigned char BcDr4_SN;                  // balcony door4 sensor local switch no.
    //---------------------------------------
    unsigned char Ntw_Prtc;                  // networking protocol
    #define Itn_iNET               0                             // Intronics iNET (baud rate 9600bps, total 10 bits (start bit, 8-bit data, no parity, 1 stop bit)
    #define Itn_Mdbs               1                             // Intronics Modbus (RTU mode, baud rate 9600bps, total 11 bits (start bit, 8-bit data, even parity, 1 stop bit))
    //---------------------------------------
 //   MbtUint16 Ntwk_Adr;                      // networking address (BCD 4 digits)
                                             // !!! must be end of array to exclude CRC-16 of parameter table
    //---------------------------------------
                                                                              // @@@ end of array
  } P;
} Typedef_ParaTb2b;                                  // parameter table 2b

typedef union {
  //unsigned char array[EE_BlockSze04-1];
  struct {
   
                                                                              // @@@ start of array
    RlyCctGp KyO0_RlC;                       // key out turning off relay circuit (force turn off when remove key card)
 //   DimChnGp KyO0_DmC;                       // key out turning off dimmer channel (force turn off when remove key card)
    //---------------------------------------
    RlyCctGp OtLt_RlC;                       // outlet relay (force turn on when insert key card)
    //---------------------------------------
    RlyCctGp WcLmp_Gp;                       // welcome lamp group (when insert key card)
    RlyCctGp W1Lmp_MU;                       // welcome1 lamp memory mode use
    RlyCctGp W1Lmp_Ps;                       // welcome1 lamp preset status
    RlyCctGp W2Lmp_MU;                       // welcome2 lamp memory mode use
    RlyCctGp W2Lmp_Ps;                       // welcome2 lamp preset status

 //   DimChnGp WcDmm_Gp;                       // welcome dimmer group
 //   DimChnGp W1Dmm_MU;                       // welcome1 dimmer memory mode use
 //   struct Dim_GrpFmt W1Dmm_Ps;              // welcome1 dimmer preset status
//    DimChnGp W2Dmm_MU;                       // welcome2 dimmer memory mode use
//    struct Dim_GrpFmt W2Dmm_Ps;              // welcome2 dimmer preset status

    RlyCctGp PeWc_Rly;                       // pre-welcome relay
 //   struct Dim_GrpFmt PeWc_Dmm;              // pre-welcome dimmer
    //---------------------------------------
    RlyCctGp MtLmp_Gp;                       // master lamp group
    RlyCctGp M1Lmp_MU;                       // master1 lamp memory mode use
    RlyCctGp M1Lmp_Ps;                       // master1 lamp preset status
    RlyCctGp M2Lmp_MU;                       // master2 lamp memory mode use
    RlyCctGp M2Lmp_Ps;                       // master2 lamp preset status

//    DimChnGp MtDmm_Gp;                       // master dimmer group
 //   DimChnGp M1Dmm_MU;                       // master1 dimmer memory mode use
 //   struct Dim_GrpFmt M1Dmm_Ps;              // master1 dimmer preset status
  //  DimChnGp M2Dmm_MU;                       // master2 dimmer memory mode use
 //   struct Dim_GrpFmt M2Dmm_Ps;              // master2 dimmer preset status
    //---------------------------------------
//    LocSwGrp LcSwPt_E;                       // local switch pattern enable flag
    unsigned char LS01_PtN;                  // local switch 1 pattern number
    unsigned char LS02_PtN;                  // local switch 2 pattern number
    unsigned char LS03_PtN;                  // local switch 3 pattern number
    unsigned char LS04_PtN;                  // local switch 4 pattern number
    unsigned char LS05_PtN;                  // local switch 5 pattern number
    unsigned char LS06_PtN;                  // local switch 6 pattern number
    unsigned char LS07_PtN;                  // local switch 7 pattern number
    unsigned char LS08_PtN;                  // local switch 8 pattern number
    unsigned char LS09_PtN;                  // local switch 9 pattern number
    unsigned char LS10_PtN;                  // local switch 10 pattern number
    unsigned char LS11_PtN;                  // local switch 11 pattern number
    unsigned char LS12_PtN;                  // local switch 12 pattern number
    unsigned char LS13_PtN;                  // local switch 13 pattern number
    unsigned char LS14_PtN;                  // local switch 14 pattern number
    unsigned char LS15_PtN;                  // local switch 15 pattern number
    unsigned char LS16_PtN;                  // local switch 16 pattern number
    unsigned char LS17_PtN;                  // local switch 17 pattern number
    unsigned char LS18_PtN;                  // local switch 18 pattern number
    unsigned char LS19_PtN;                  // local switch 19 pattern number
    unsigned char LS20_PtN;                  // local switch 20 pattern number
    unsigned char LS21_PtN;                  // local switch 21 pattern number
    //---------------------------------------
    unsigned char KbS01_Op;                  // keyboard switch 1 operation
    unsigned char KbS02_Op;                  // keyboard switch 2 operation
    unsigned char KbS03_Op;                  // keyboard switch 3 operation
    unsigned char KbS04_Op;                  // keyboard switch 4 operation
    unsigned char KbS05_Op;                  // keyboard switch 5 operation
    unsigned char KbS06_Op;                  // keyboard switch 6 operation
    unsigned char KbS07_Op;                  // keyboard switch 7 operation
    unsigned char KbS08_Op;                  // keyboard switch 8 operation
    unsigned char KbS09_Op;                  // keyboard switch 9 operation
    unsigned char KbS10_Op;                  // keyboard switch 10 operation
    unsigned char KbS11_Op;                  // keyboard switch 11 operation
    unsigned char KbS12_Op;                  // keyboard switch 12 operation
    unsigned char KbS13_Op;                  // keyboard switch 13 operation
    unsigned char KbS14_Op;                  // keyboard switch 14 operation
    unsigned char KbS15_Op;                  // keyboard switch 15 operation
    unsigned char KbS16_Op;                  // keyboard switch 16 operation
                                                       // ptrn01_I to Patrn_ID
    #define S4_Mastr               (Patrn_ID+1)                  // master switch
    #define S4_DNDst               (Patrn_ID+2)                  // do not disturb switch
    #define S4_MkURm               (Patrn_ID+3)                  // make up room switch
    #define S4_PWait               (Patrn_ID+4)                  // please wait switch
    
    #define DpSwOp_U               (Patrn_ID+4)                  // upper limit of display switch operation
                                                                               // @@@ end of array
  } P;
} Typedef_ParaTb04;                                  // parameter table 4



//-------------------------------------------
typedef union {
  //unsigned char array[EE_BlockSze05-1];
  struct {
                                                                                // @@@ start of array
#if(U_ExhFan)
    unsigned char EhFan_RN;                  // exhaust fan relay circuit no.
    unsigned char EFn_TCDu;                  // exhaust fan temperature control duty cycle (m/h)
    unsigned char EFn_SrTp;                  // exhaust fan start temperature index
    unsigned char EFn_SpTp;                  // exhaust fan stop temperature index
    RlyCctGp Bath_LpC;                       // bathroom lamp circuit
    DimChnGp Bath_DmC;                       // bathroom dimmer channel
#endif
    //---------------------------------------
    RlyCctGp Shar_RlC;                       // sharing relay circuit
//    DimChnGp Shar_DmC;                       // sharing dimmer channel
    //---------------------------------------
#if(U_CpyDmm)
    DimChnGp CpDm_Gp1;                       // copying dimmer group1
    DimChnGp CpDm_Gp2;                       // copying dimmer group2
#endif
    //---------------------------------------
//    struct ClkScn_format CkSc01_P;           // clock scene1
//    struct ClkScn_format CkSc02_P;           // clock scene2
    //---------------------------------------
//    PatrnGrp PtLd_MSc;                       // pattern led matching scene (0=scene1, 1=scene2)
    //---------------------------------------
    unsigned char AuLg0T_P;                  // automatic light off time (m)
    #define AuLg0T_L               1                             // lower limit
    #define AuLg0T_U               180                           // upper limit
    unsigned char AuL0M_SN;                  // automatic light off motion sensor local switch no.
    RlyCctGp AuL0_LpC;                       // automatic light off lamp circuit
//    DimChnGp AuL0_DmC;                       // automatic light off dimmer channel
    //---------------------------------------
    unsigned char MtLgHT_P;                  // motion light holding time (m)
    #define MtLgHT_L               1                             // lower limit
    #define MtLgHT_U               180                           // upper limit
    unsigned char MtnLg_SN;                  // motion light sensor local switch no.
    RlyCctGp MtLg_LpC;                       // motion light lamp circuit
//    DimChnGp MtLg_DmC;                       // motion light dimmer channel
    //---------------------------------------
    unsigned char EtMLHT_P;                  // entrance motion light holding time (s)
    #define EtMLHT_L               1                             // lower limit
    #define EtMLHT_U               180                           // upper limit
    unsigned char EntMt_SN;                  // entrance motion sensor local switch no.
    RlyCctGp EtML_LpC;                       // entrance motion light lamp circuit
//    DimChnGp EtML_DmC;                       // entrance motion light dimmer channel
    //---------------------------------------
    RlyCctGp Emer_LpC;                       // emergency lamp circuit (automatic switch on when normal line outage)
//    DimChnGp Emer_DmC;                       // emergency dimmer channel (automatic switch on when normal line outage)
    //---------------------------------------
    RlyCctGp PwFS_RlC;                       // power fail saving relay circuit (off during normal line outage)
//    DimChnGp PwFS_DmC;                       // power fail saving dimmer channel (off during normal line outage)
    //---------------------------------------
    RlyCctGp CkIn_RlC;                       // check in relay circuit (always turn on during check in)
    //---------------------------------------
    RlyCctGp McScIL_U;                       // matching scene input lamp use
    RlyCctGp McScIL_S;                       // matching scene input lamp status
//    DimChnGp McScID_U;                       // matching scene input dimmer use
//    DimChnGp McScID_S;                       // matching scene input dimmer on/off status
    RlyCctGp McScO_LC;                       // matching scene output lamp circuit
//    DimChnGp McScO_DC;                       // matching scene output dimmer channel
    //---------------------------------------
//    struct PtScNo_format DpNgTc_P;           // display night touch
    //---------------------------------------
    unsigned char SbyCAT_P;                  // standby circuit active time (m)
    #define SbyCAT_L               1                             // lower limit
    #define SbyCAT_U               180                           // upper limit
    RlyCctGp Stby_RlC;                       // standby relay circuit
//    DimChnGp Stby_DmC;                       // standby dimmer channel
    //---------------------------------------
    RlyCctGp P01S1L_U;                       // pattern1 scene1 lamp use
    RlyCctGp P01S1L_S;                       // pattern1 scene1 lamp status
    RlyCctGp P01S2L_U;                       // pattern1 scene2 lamp use
    RlyCctGp P01S2L_S;                       // pattern1 scene2 lamp status

    RlyCctGp P02S1L_U;                       // pattern2 scene1 lamp use
    RlyCctGp P02S1L_S;                       // pattern2 scene1 lamp status
    RlyCctGp P02S2L_U;                       // pattern2 scene2 lamp use
    RlyCctGp P02S2L_S;                       // pattern2 scene2 lamp status

    RlyCctGp P03S1L_U;                       // pattern3 scene1 lamp use
    RlyCctGp P03S1L_S;                       // pattern3 scene1 lamp status
    RlyCctGp P03S2L_U;                       // pattern3 scene2 lamp use
    RlyCctGp P03S2L_S;                       // pattern3 scene2 lamp status

    RlyCctGp P04S1L_U;                       // pattern4 scene1 lamp use
    RlyCctGp P04S1L_S;                       // pattern4 scene1 lamp status
    RlyCctGp P04S2L_U;                       // pattern4 scene2 lamp use
    RlyCctGp P04S2L_S;                       // pattern4 scene2 lamp status

    RlyCctGp P05S1L_U;                       // pattern5 scene1 lamp use
    RlyCctGp P05S1L_S;                       // pattern5 scene1 lamp status
    RlyCctGp P05S2L_U;                       // pattern5 scene2 lamp use
    RlyCctGp P05S2L_S;                       // pattern5 scene2 lamp status

    RlyCctGp P06S1L_U;                       // pattern6 scene1 lamp use
    RlyCctGp P06S1L_S;                       // pattern6 scene1 lamp status
    RlyCctGp P06S2L_U;                       // pattern6 scene2 lamp use
    RlyCctGp P06S2L_S;                       // pattern6 scene2 lamp status
                                                                                // @@@ end of array
  } P;
} Typedef_ParaTb05;                                  // parameter table 5
 /*-------------------------------------------
           expand output group format 
  ------------------------------------------*/




//-----------------------------------------------------------------------------
typedef union {
  struct {
    unsigned char Group1;
    unsigned char Group2;
    unsigned char Group3;
    unsigned char Group4;
    unsigned char Group5;
    unsigned char Group6;
    unsigned char Group7;
  } uint8;
  struct {
    unsigned Relay01              :1;       // (b0)   ;RLY1 circuit01 relay
    unsigned Relay02              :1;       // (b1)   ;RLY2 circuit02 relay
    unsigned Relay03              :1;       // (b2)   ;RLY3 circuit03 relay
    unsigned Relay04              :1;       // (b3)   ;RLY4 circuit04 relay
    unsigned Relay05              :1;       // (b4)   ;RLY5 circuit05 relay
    unsigned Relay06              :1;       // (b5)   ;RLY6 circuit06 relay
    unsigned Relay07              :1;       // (b6)   ;RLY7 circuit07 relay
    unsigned Relay08              :1;       // (b7)   ;RLY8 circuit08 relay

    unsigned Relay09               :1;       // (b8)   ;RLY9 circuit11 relay
    unsigned Relay10              :1;       // (b9)   ;RLY10 circuit12 relay
    unsigned Spare1               :1;       // (b10)  ;spare1
    unsigned Spare2               :1;       // (b11)  ;spare2
    unsigned Relay11              :1;       // (b12)  ;RLY11 circuit13 relay
    unsigned Relay12              :1;       // (b13)  ;RLY12 circuit14 relay
    unsigned Relay13              :1;       // (b14)  ;RLY13 circuit15 relay
    unsigned Relay14              :1;       // (b15)  ;RLY14 circuit16 relay

    unsigned Relay22              :1;       // (b16)  ;RLY22 circuit10 relay
    unsigned Relay21              :1;       // (b17)  ;RLY21 circuit09 relay
    unsigned Relay20              :1;       // (b18)  ;RLY20 circuit17 relay
    unsigned Relay19              :1;       // (b19)  ;RLY19 circuit18 relay
    unsigned RelayFanHi           :1;       // (b20)  ;RLY15 fan high speed relay
    unsigned RelayFanMed          :1;       // (b21)  ;RLY16 fan medium speed relay
    unsigned RelayFanLow          :1;       // (b22)  ;RLY17 fan low speed relay
    unsigned RlValv_P             :1;       // (b23)  ;RLY18 valve relay

    unsigned Spare3                :1;       // (b24)  ;spare3
    unsigned ServiceMURLED         :1;       // (b25)  ;service panel make up room led
    unsigned KeyboxLED             :1;       // (b26)  ;key box led
    unsigned MasterPanelLED        :1;       // (b27)  ;master panel led
    unsigned NightLED              :1;       // (b28)  ;night led
    unsigned FrontDNTLED           :1;       // (b29)  ;front panel do not disturb led
    unsigned FrontMURLED           :1;       // (b30)  ;front panel make up room led
    unsigned ServiceDNTLED         :1;       // (b31)  ;service panel do not disturb led

    unsigned LocalSW8               :1;       // (b32)  ;LcSw08 enable
    unsigned LocalSW7               :1;       // (b33)  ;LcSw07 enable
    unsigned LocalSW6               :1;       // (b34)  ;LcSw06 enable
    unsigned LocalSW5               :1;       // (b35)  ;LcSw05 enable
    unsigned LocalSW4               :1;       // (b36)  ;LcSw04 enable
    unsigned LocalSW3               :1;       // (b37)  ;LcSw03 enable
    unsigned LocalSW2               :1;       // (b38)  ;LcSw02 enable
    unsigned LocalSW1               :1;       // (b39)  ;LcSw01 enable

    unsigned LocalSW16              :1;       // (b40)  ;LcSw16 enable
    unsigned LocalSW15              :1;       // (b41)  ;LcSw15 enable
    unsigned LocalSW14              :1;       // (b42)  ;LcSw14 enable
    unsigned LocalSW13              :1;       // (b43)  ;LcSw13 enable
    unsigned LocalSW12              :1;       // (b44)  ;LcSw12 enable
    unsigned LocalSW11              :1;       // (b45)  ;LcSw11 enable
    unsigned LocalSW10              :1;       // (b46)  ;LcSw10 enable
    unsigned LocalSW09              :1;       // (b47)  ;LcSw09 enable

    unsigned Spare4                :1;       // (b48)  ;spare4
    unsigned ServiceMURSW_EN       :1;       // (b49)  ;service panel make up room switch enable
    unsigned ServiceDNTSW_EN       :1;       // (b50)  ;service panel do not disturb switch enable
    unsigned MotionSW_EN           :1;       // (b51)  ;motion sensor switch enable
    unsigned ChimeSW_EN            :1;       // (b52)  ;chime switch enable
    unsigned KeyboxSW_EN           :1;       // (b53)  ;key box switch enable
    unsigned BalconySW_EN          :1;       // (b54)  ;balcony door switch enable
    unsigned MasterPanelSW_EN      :1;       // (b55)  ;master panel switch enable
  } bit;
}EpO259Gp;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

//EpO259Gp EpO_Data;                           // expand output data
#define RlCc01_S                   EpO_Data.bit.Relay01         // circuit01 relay
#define RlCc02_S                   EpO_Data.bit.Relay02         // circuit02 relay
#define RlCc03_S                   EpO_Data.bit.Relay03         // circuit03 relay
#define RlCc04_S                   EpO_Data.bit.Relay04         // circuit04 relay
#define RlCc05_S                   EpO_Data.bit.Relay05         // circuit05 relay
#define RlCc06_S                   EpO_Data.bit.Relay06         // circuit06 relay
#define RlCc07_S                   EpO_Data.bit.Relay07         // circuit07 relay
#define RlCc08_S                   EpO_Data.bit.Relay08         // circuit08 relay
#define RlCc09_S                   EpO_Data.bit.Relay09         // circuit09 relay
#define RlCc10_S                   EpO_Data.bit.Relay10         // circuit10 relay
#define RlCc11_S                   EpO_Data.bit.Relay11         // circuit11 relay
#define RlCc12_S                   EpO_Data.bit.Relay12         // circuit12 relay
#define RlCc13_S                   EpO_Data.bit.Relay13         // circuit13 relay
#define RlCc14_S                   EpO_Data.bit.Relay14         // circuit14 relay
#define RlCc15_S                   EpO_Data.bit.Relay15         // circuit15 relay
#define RlCc16_S                   EpO_Data.bit.Relay16         // circuit16 relay
#define RlFnHi_S                   EpO_Data.bit.RelayFanHi         // fan high speed relay
#define RlFnMd_S                   EpO_Data.bit.RelayFanMed         // fan medium speed relay
#define RlFnLw_S                   EpO_Data.bit.RelayFanLow         // fan low speed relay
#define RlValv_S                   EpO_Data.bit.RlValv_P         // valve relay
#define KyBxLd_S                   EpO_Data.bit.KeyboxLED       // key led
#define FDNDLd_S                   EpO_Data.bit.FrontDNTLED     // front panel do not disturb led
#define FMURLd_S                   EpO_Data.bit.FrontMURLED     // front panel make up room led
#define MstPLd_S                   EpO_Data.bit.MasterPanelLED   // master panel led



/* expand output group bit position*/
#define Relay01_BIT    ((8*0)+0)           // RLY1 circuit01 relay
#define Relay02_BIT    ((8*0)+1)           // RLY2 circuit02 relay
#define Relay03_BIT    ((8*0)+2)           // RLY3 circuit03 relay
#define Relay04_BIT    ((8*0)+3)           // RLY4 circuit04 relay
#define Relay05_BIT    ((8*0)+4)           // RLY5 circuit05 relay
#define Relay06_BIT    ((8*0)+5)           // RLY6 circuit06 relay
#define Relay07_BIT    ((8*0)+6)           // RLY7 circuit07 relay
#define Relay08_BIT    ((8*0)+7)           // RLY8 circuit08 relay

#define Relay9_BIT     ((8*1)+0)           // RLY9 circuit11 relay
#define Relay10_BIT    ((8*1)+1)           // RLY10 circuit12 relay
#define Spare1_BIT     ((8*1)+2)           // spare1
#define Spare2_BIT     ((8*1)+3)           // spare2
#define Relay11_BIT    ((8*1)+4)           // RLY11 circuit13 relay
#define Relay12_BIT    ((8*1)+5)           // RLY12 circuit14 relay
#define Relay13_BIT    ((8*1)+6)           // RLY13 circuit15 relay
#define Relay14_BIT    ((8*1)+7)           // RLY14 circuit16 relay

#define Relay22_BIT        ((8*2)+0)           // RLY22 circuit10 relay
#define Relay21_BIT        ((8*2)+1)           // RLY21 circuit09 relay
#define Relay20_BIT        ((8*2)+2)           // RLY20 circuit17 relay
#define Relay19_BIT        ((8*2)+3)           // RLY19 circuit18 relay
#define RelayFanHi_BIT     ((8*2)+4)           // RLY15 fan high speed relay
#define RelayFanMed_BIT    ((8*2)+5)           // RLY16 fan medium speed relay
#define RelayFanLow_BIT    ((8*2)+6)           // RLY17 fan low speed relay
#define Relay18_BIT        ((8*2)+7)           // RLY18 valve relay

#define Spare3_BIT            ((8*3)+0)           // spare3
#define ServiceMURLED_BIT     ((8*3)+1)           // service panel make up room led
#define KeyboxLED_BIT         ((8*3)+2)           // key box led
#define MasterPanelLED_BIT    ((8*3)+3)           // master panel led
#define NightLED_BIT          ((8*3)+4)           // night led
#define FrontDNTLED_BIT       ((8*3)+5)           // front panel do not disturb led
#define FrontMURLED_BIT       ((8*3)+6)           // front panel make up room led
#define ServiceDNTLED_BIT     ((8*3)+7)           // service panel do not disturb led

#define LocalSW8_BIT    ((8*4)+0)           // LcSw08 enable
#define LocalSW7_BIT    ((8*4)+1)           // LcSw07 enable
#define LocalSW6_BIT    ((8*4)+2)           // LcSw06 enable
#define LocalSW5_BIT    ((8*4)+3)           // LcSw05 enable
#define LocalSW4_BIT    ((8*4)+4)           // LcSw04 enable
#define LocalSW3_BIT    ((8*4)+5)           // LcSw03 enable
#define LocalSW2_BIT    ((8*4)+6)           // LcSw02 enable
#define LocalSW1_BIT    ((8*4)+7)           // LcSw01 enable

#define LocalSW16_BIT    ((8*5)+0)           // LcSw16 enable
#define LocalSW15_BIT    ((8*5)+1)           // LcSw15 enable
#define LocalSW14_BIT    ((8*5)+2)           // LcSw14 enable
#define LocalSW13_BIT    ((8*5)+3)           // LcSw13 enable
#define LocalSW12_BIT    ((8*5)+4)           // LcSw12 enable
#define LocalSW11_BIT    ((8*5)+5)           // LcSw11 enable
#define LocalSW10_BIT    ((8*5)+6)           // LcSw10 enable
#define LocalSW09_BIT    ((8*5)+7)           // LcSw09 enable

#define Spare4             ((8*6)+0)           // spare4
#define ServiceMURSW_EN    ((8*6)+1)           // service panel make up room switch enable
#define ServiceDNTSW_EN    ((8*6)+2)           // service panel do not disturb switch enable
#define MotionSW_EN        ((8*6)+3)           // motion sensor switch enable
#define ChimeSW_EN         ((8*6)+4)           // chime switch enable
#define KeyboxSW_EN        ((8*6)+5)           // key box switch enable
#define BalconySW_EN       ((8*6)+6)           // balcony door switch enable
#define MasterPanelSW_EN   ((8*6)+7)           // master panel switch enable



/*74HC259 port definition use in driverhc259 file*/

#define EN_259PORT1  GPIOD 
#define EN_259PIN1   2 

#define EN_259PORT2  GPIOB 
#define EN_259PIN2   7 

#define EN_259PORT3  GPIOB 
#define EN_259PIN3   6 

#define EN_259PORT4  GPIOC
#define EN_259PIN4   7

#define EN_259PORT5  GPIOB
#define EN_259PIN5   12

#define EN_259PORT6  GPIOB
#define EN_259PIN6   2

#define EN_259PORT7  GPIOC
#define EN_259PIN7   6
 
#define DATA_259PORT  GPIOC
#define DATA_259PIN   12

#define A0_259PORT    GPIOB
#define A0_259PIN     5

#define A1_259PORT    GPIOB
#define A1_259PIN     4
                      
#define A2_259PORT    GPIOB
#define A2_259PIN     3

/* USER CODE END EC */


/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* Model definition (Preprocessor directive) ---------------------------------*/

#define Kybrd_Ty    WT30_2Wr  // keyboard type
#define DT50_2Wr    0         // DT05 (2x4")
#define WT10_2Wr    1         // wall thermostat 1.01 (2x4")          => no mode led
#define WT11_2Wr    2         // wall thermostat 1.1 (2x4")           => no mode led, no mode button
#define WT20_2Wr    3         // wall thermostat 2 (2x4",touch)       => no mode led
#define WT30_2Wr    4         // wall thermostat 3 (2x4")             => no mode led                     -> default of 2-wire display
#define WT40_2Wr    5         // wall thermostat 4 (3x3")             => no mode led, no mode button
#define SP10_2Wr    6         // switch panel 1 (2x4",touch)          => no mode led, no mode button
#define LW20_2Wr    7         // lcd wire 2 (2x4")                    => 0.5C/step
#define LW40_2Wr    8         // lcd wire 4E, DT07E (3x3",touch)
#define DT81_2Wr    9         // DT08.1                               => no mode led, no mode button, no fan speed led
#define TT7S_SPI    10        // table top 7-segment                  => 0.5C/step, no world time, no mode led, no mode button
#define ULCD_SPI    11        // universal LCD keyboard               => 0.5C/step
#define STFT_SPI    12        // single-page TFT keyboard             => 0.5C/step, no world time
#define MTFT_SPI    13        // multi-page TFT keyboard              => 0.5C/step

#define TwWBs_Dp    0         // 2-wire bus display
#define SPIBs_Dp    1         // SPI bus display


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void waitDelay(uint16_t count);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define C_DgtOt1_Pin GPIO_PIN_13
#define C_DgtOt1_GPIO_Port GPIOC
#define C_DgtOt2_Pin GPIO_PIN_14
#define C_DgtOt2_GPIO_Port GPIOC
#define C_DgtOt3_Pin GPIO_PIN_15
#define C_DgtOt3_GPIO_Port GPIOC
#define F_OscInp_Pin GPIO_PIN_0
#define F_OscInp_GPIO_Port GPIOF
#define F_OscOut_Pin GPIO_PIN_1
#define F_OscOut_GPIO_Port GPIOF
#define C_DgtOt4_Pin GPIO_PIN_0
#define C_DgtOt4_GPIO_Port GPIOC
#define C_DgtOt5_Pin GPIO_PIN_1
#define C_DgtOt5_GPIO_Port GPIOC
#define C_DgtIn1_Pin GPIO_PIN_2
#define C_DgtIn1_GPIO_Port GPIOC
#define C_DgtIn2_Pin GPIO_PIN_3
#define C_DgtIn2_GPIO_Port GPIOC
#define A_Ln1Zrc_Pin GPIO_PIN_0
#define A_Ln1Zrc_GPIO_Port GPIOA
#define A_iNetRS_Pin GPIO_PIN_1
#define A_iNetRS_GPIO_Port GPIOA
#define A_iNetTX_Pin GPIO_PIN_2
#define A_iNetTX_GPIO_Port GPIOA
#define A_iNetRX_Pin GPIO_PIN_3
#define A_iNetRX_GPIO_Port GPIOA
#define A_KbrdSS_Pin GPIO_PIN_4
#define A_KbrdSS_GPIO_Port GPIOA
#define A_KbrdMO_Pin GPIO_PIN_7
#define A_KbrdMO_GPIO_Port GPIOA
#define C_KBxBRX_Pin GPIO_PIN_4
#define C_KBxBRX_GPIO_Port GPIOC
#define ROOM_TMP_Pin GPIO_PIN_5
#define ROOM_TMP_GPIO_Port GPIOC
#define SW_SENSE_Pin GPIO_PIN_0
#define SW_SENSE_GPIO_Port GPIOB
#define B_ExpdRS_Pin GPIO_PIN_1
#define B_ExpdRS_GPIO_Port GPIOB
#define B_CrSwE2_Pin GPIO_PIN_2
#define B_CrSwE2_GPIO_Port GPIOB
#define B_ExpdTX_Pin GPIO_PIN_10
#define B_ExpdTX_GPIO_Port GPIOB
#define B_ExpdRX_Pin GPIO_PIN_11
#define B_ExpdRX_GPIO_Port GPIOB
#define B_CrSwE1_Pin GPIO_PIN_12
#define B_CrSwE1_GPIO_Port GPIOB
#define HMD_SCL_Pin GPIO_PIN_13
#define HMD_SCL_GPIO_Port GPIOB
#define HMD_SDA_Pin GPIO_PIN_14
#define HMD_SDA_GPIO_Port GPIOB
#define B_IRRecv_Pin GPIO_PIN_15
#define B_IRRecv_GPIO_Port GPIOB
#define C_CrSwE3_Pin GPIO_PIN_6
#define C_CrSwE3_GPIO_Port GPIOC
#define C_OExpE4_Pin GPIO_PIN_7
#define C_OExpE4_GPIO_Port GPIOC
#define C_AnlOt2_Pin GPIO_PIN_8
#define C_AnlOt2_GPIO_Port GPIOC
#define C_AnlOt1_Pin GPIO_PIN_9
#define C_AnlOt1_GPIO_Port GPIOC
#define A_ChmAud_Pin GPIO_PIN_8
#define A_ChmAud_GPIO_Port GPIOA
#define A_2WirTX_Pin GPIO_PIN_9
#define A_2WirTX_GPIO_Port GPIOA
#define A_2WirRX_Pin GPIO_PIN_10
#define A_2WirRX_GPIO_Port GPIOA
#define A_USB_DM_Pin GPIO_PIN_11
#define A_USB_DM_GPIO_Port GPIOA
#define A_USB_DP_Pin GPIO_PIN_12
#define A_USB_DP_GPIO_Port GPIOA
#define A_SWDIO_Pin GPIO_PIN_13
#define A_SWDIO_GPIO_Port GPIOA
#define A_SWCLK_Pin GPIO_PIN_14
#define A_SWCLK_GPIO_Port GPIOA
#define A_MdBsRS_Pin GPIO_PIN_15
#define A_MdBsRS_GPIO_Port GPIOA
#define C_MdBsTX_Pin GPIO_PIN_10
#define C_MdBsTX_GPIO_Port GPIOC
#define C_MdBsRX_Pin GPIO_PIN_11
#define C_MdBsRX_GPIO_Port GPIOC
#define C_OExpDt_Pin GPIO_PIN_12
#define C_OExpDt_GPIO_Port GPIOC
#define D_OExpE1_Pin GPIO_PIN_2
#define D_OExpE1_GPIO_Port GPIOD
#define B_OExpA2_Pin GPIO_PIN_3
#define B_OExpA2_GPIO_Port GPIOB
#define B_OExpA1_Pin GPIO_PIN_4
#define B_OExpA1_GPIO_Port GPIOB
#define B_OExpA0_Pin GPIO_PIN_5
#define B_OExpA0_GPIO_Port GPIOB
#define B_OExpE3_Pin GPIO_PIN_6
#define B_OExpE3_GPIO_Port GPIOB
#define B_OExpE2_Pin GPIO_PIN_7
#define B_OExpE2_GPIO_Port GPIOB
#define RTC_SDL_Pin GPIO_PIN_8
#define RTC_SDL_GPIO_Port GPIOB
#define RTC_SDA_Pin GPIO_PIN_9
#define RTC_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
