/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "timer.h"
#include "uartITN.h"
#include "SPIDisp.h"  
#include "adcITN.h"    
#include "driverhc259.h"    
#include "relayct.h"   
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//chanatip start 
 //-------------------------------------------
unsigned char CpOfD_Mn;                      // compressor off delay (m)
unsigned char CpOfD_Sc;                      // compressor off delay (s)
unsigned short Val_ItCy;                     // valve interval cycle timer (s)


//-------------------------------------------
unsigned char NbdSt_Mn;                      // nobody stays timer (m)
unsigned short NbdSt_ms;                     // nobody stays timer (20ms)
unsigned char AcTSS_Mn;                      // air-con setting temperature saving step timer (m)
unsigned char AcTSS_ms;                      // air-con setting temperature saving step timer (20ms)
unsigned char AcTS_Ofs;                      // air-con setting temperature saving offset temperature (0.5C)
//-------------------------------------------
unsigned char AcNMt_Mn;                      // air-con no-motion timer (m)
unsigned short AcNMt_ms;                     // air-con no-motion timer (20ms)
unsigned char A2NMt_Mn;                      // air-con2 no-motion timer (m)
unsigned short A2NMt_ms;                     // air-con2 no-motion timer (20ms)
//-------------------------------------------
//==============================================================================
// data table for convert different temperature index (0.5C/step) to fan auto speed
//------------------------------------------------------------------------------
// fan auto speed
// - low,medium,high
//-------------------------------------------
// lower threshold of the hysteresis
const unsigned char FnAS_HyL[] = {
  2*(1)+0,                         // < 1C low speed
  2*(2)+0,                         // < 2C medium speed
  0xFF                             // < max high speed
};
//-------------------------------------------
// upper threshold of the hysteresis
const unsigned char FnAS_HyU[] = {
  2*(1)+1,                         // < 1.5C low speed
  2*(2)+1,                         // < 2.5C medium speed
  0xFF                             // < max high speed
};    
    
//-------------------------------------------
unsigned char Sen1_Tmp;                      // sensor1 temperature index
unsigned char Room_Tmp;                      // room temperature index
unsigned char Room_Hum;                      // room humidity (0-100%RH)
//-------------------------------------------
unsigned char Ct_StTmp;                      // control setting temperature index
signed char VlC_DfTp;                        // valve control different temperature index
unsigned char FSC_DfTp;                      // fan speed control different temperature index
//-------------------------------------------
unsigned char FnAuSp_S;                      // fan auto speed state
//-------------------------------------------   
    
    
    
  
MbtUint8 HwTs_Buf[HwTBf_Sz];                 // hardware testing buffer


RlyCctGp FnRlC_St;                           // final relay circuit status
RlyCctGp Genr_RlC;                           // general relay circuit
RlyCctGp Tmp_RlCc;                           // temporary relay circuit data

MbtUint8 GenFg_01;                  // general purpose flag1
MbtUint8 GenFg_02;                  // general purpose flag2
MbtUint8 GenFg_03;                  // general purpose flag3
MbtUint8 GenFg_04;                  // general purpose flag4
MbtUint8 GenFg_05;                  // general purpose flag6
MbtUint8 GenFg_06;                  // general purpose flag5
MbtUint8 GenFg_07;                  // general purpose flag7
MbtUint8 GenFg_08;                  // general purpose flag8
MbtUint8 GenFg_09;                  // general purpose flag9
MbtUint8 GenFg_10;                  // general purpose flag10
MbtUint8 GenFg_11;                  // general purpose flag11
MbtUint8 GenFg_12;                  // general purpose flag12
MbtUint8 GenFg_13;                  // general purpose flag13

Typedef_ParaTb01 ParaTb01;                                  // parameter table 1
Typedef_ParaTb2a ParaTb2a; 
Typedef_ParaTb2b ParaTb2b;                                  // parameter table 2b
Typedef_ParaTb05 ParaTb05;                                  // parameter table 5




EpO259Gp EpO_PtSt;                           // expand output port status
EpO259Gp EpO_PtBf;                           // expand output port buffer
EpO259Gp EpO_Data;                           // expand output data
//-------------------------------------------
/*
union {
  unsigned char array[EE_BlockSze03-1];
  struct {
                                                                               // @@@ start of array
    //---------------------------------------
    unsigned char TmBom_En;                  // time bomb enable flag
    MbtUint16 TmBom_Hr;                      // time bomb hour down counter
    //---------------------------------------
                                                                               // @@@ end of array
  } P;
} ParaTb03;                                  // parameter table 3
*/
//-------------------------------------------

Typedef_ParaTb04  ParaTb04;                                  // parameter table 4
//-------------------------------------------
unsigned char CmMd_Lv1;
unsigned char CmMd_Lv2;

//chanatip end






/*Define for UART*/
#define TX1_SIZE (6)
#define TX2_SIZE (20)
#define TX3_SIZE (20)
#define TX4_SIZE (20)

#define RX1_SIZE (15)
#define RX2_SIZE (20)
#define RX3_SIZE (20)
#define RX4_SIZE (20)

#define UART1_RX_TIME_OUT  (4000/500)
#define UART2_RX_TIME_OUT  (4000/500)
#define UART3_RX_TIME_OUT  (4000/500)

#define D1D (1<<7)
#define D1E (1<<6)
#define D1C (1<<4)
#define D1F (1<<3)
#define D1G (1<<2)
#define D1B (1<<1)
#define D1A (1<<0)
#define D1DP (1<<5)

#define DG_0  ((uint8_t) (D1A | D1B | D1C | D1D | D1E | D1F       ))
#define DG_1  ((uint8_t) (      D1B | D1C                         ))
#define DG_2  ((uint8_t) (D1A | D1B |       D1D | D1E |       D1G ))
#define DG_3  ((uint8_t) (D1A | D1B | D1C | D1D |             D1G ))
#define DG_4  ((uint8_t) (      D1B | D1C |             D1F | D1G ))
#define DG_5  ((uint8_t) (D1A |       D1C | D1D |       D1F | D1G ))
#define DG_6  ((uint8_t) (D1A |       D1C | D1D | D1E | D1F | D1G ))
#define DG_7  ((uint8_t) (D1A | D1B | D1C                         ))
#define DG_8  ((uint8_t) (D1A | D1B | D1C | D1D | D1E | D1F | D1G ))
#define DG_9  ((uint8_t) (D1A | D1B | D1C | D1D |       D1F | D1G ))
#define DG_A  ((uint8_t) (D1A | D1B | D1C |       D1E | D1F | D1G ))
#define DG_B  ((uint8_t) (            D1C | D1D | D1E | D1F | D1G ))
#define DG_C  ((uint8_t) (D1A |             D1D | D1E | D1F       ))
#define DG_D  ((uint8_t) (      D1B | D1C | D1D | D1E |       D1G ))
#define DG_E  ((uint8_t) (D1A |             D1D | D1E | D1F | D1G ))
#define DG_F  ((uint8_t) (D1A |                   D1E | D1F | D1G ))

#define LCD1A (1<<6)
#define LCD1B (1<<4)
#define LCD1C (1<<1)
#define LCD1D (1<<0)
#define LCD1E (1<<2)
#define LCD1F (1<<5)
#define LCD1G (1<<3)  
    
    
#define LCD_0  ((uint8_t) (LCD1A | LCD1B | LCD1C | LCD1D | LCD1E | LCD1F         ))
#define LCD_1  ((uint8_t) (        LCD1B | LCD1C                                 ))
#define LCD_2  ((uint8_t) (LCD1A | LCD1B |         LCD1D | LCD1E |         LCD1G ))
#define LCD_3  ((uint8_t) (LCD1A | LCD1B | LCD1C | LCD1D |                 LCD1G ))
#define LCD_4  ((uint8_t) (        LCD1B | LCD1C |                 LCD1F | LCD1G ))
#define LCD_5  ((uint8_t) (LCD1A |         LCD1C | LCD1D |         LCD1F | LCD1G ))
#define LCD_6  ((uint8_t) (LCD1A |         LCD1C | LCD1D | LCD1E | LCD1F | LCD1G ))
#define LCD_7  ((uint8_t) (LCD1A | LCD1B | LCD1C                                 ))
#define LCD_8  ((uint8_t) (LCD1A | LCD1B | LCD1C | LCD1D | LCD1E | LCD1F | LCD1G ))
#define LCD_9  ((uint8_t) (LCD1A | LCD1B | LCD1C | LCD1D |         LCD1F | LCD1G ))
#define LCD_A  ((uint8_t) (LCD1A | LCD1B | LCD1C |         LCD1E | LCD1F | LCD1G ))
#define LCD_B  ((uint8_t) (                LCD1C | LCD1D | LCD1E | LCD1F | LCD1G ))
#define LCD_C  ((uint8_t) (LCD1A |                 LCD1D | LCD1E | LCD1F         ))
#define LCD_D  ((uint8_t) (        LCD1B | LCD1C | LCD1D | LCD1E |         LCD1G ))
#define LCD_E  ((uint8_t) (LCD1A |                 LCD1D | LCD1E | LCD1F | LCD1G ))
#define LCD_F  ((uint8_t) (LCD1A |                         LCD1E | LCD1F | LCD1G ))    
    
    
enum{
  PREPARE_DATA,
  WAIT_SEND_COMPLETE,
  CHANGE_TO_RX,
  RECEIVED_DATA,
  WAIT_IDLE
};




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

IWDG_HandleTypeDef hiwdg;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
uint8_t m0s0625C;
uint16_t m0s50_Ct; 
uint8_t slotcount;
uint8_t Sec_Cntr;

uint8_t testpos=0;
uint8_t testdat=1;


/*UART variable*/
USARTHandler UART1;
USARTHandler UART2;
USARTHandler UART3;
USARTHandler UART4;

uint8_t TX1Data[TX1_SIZE];
uint8_t RX1Data[RX1_SIZE];
uint8_t ConnectState = PREPARE_DATA;
uint8_t KeyData;

/*SPI variable*/
SPIDataHandler SPIDat;
uint8_t SPITXData[SPI_TX_SIZE];
uint8_t SPIRXData[SPI_RX_SIZE];
/*Adc vairable*/
uint16_t ADCValve[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

void WALLCommunication(USARTHandler *uart);
void SPICommunication(SPI_TypeDef *SPICH,SPIDataHandler *spi);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t DSP1_TAB[] = {
  DG_0,
  DG_1, 
  DG_2, 
  DG_3, 
  DG_4, 
  DG_5, 
  DG_6, 
  DG_7, 
  DG_8, 
  DG_9, 
  DG_A, 
  DG_B, 
  DG_C, 
  DG_D, 
  DG_E, 
  DG_F     
};

const uint8_t LCD1_TAB[] = {
  LCD_0,
  LCD_1, 
  LCD_2, 
  LCD_3, 
  LCD_4, 
  LCD_5, 
  LCD_6, 
  LCD_7, 
  LCD_8, 
  LCD_9, 
  LCD_A, 
  LCD_B, 
  LCD_C, 
  LCD_D, 
  LCD_E, 
  LCD_F     
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __disable_interrupt();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_USB_PCD_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_SPI1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  
  /*Initialize timer*/
  InitTimerCallBack();
  /*Initialize UART handler*/
  InitUARTHandler();
  /*Calibrate ADC*/
  calibrateADC();
  enableADC();
  
  
  __enable_interrupt();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(m0s0625C >= 8) {                         // check 0.0625ms counter for 0.5ms loop
      m0s0625C -= 8;
      m0s50_Ct++;                               // increase 0.5ms counter
      HAL_IWDG_Refresh(&hiwdg);                 // refresh watchdog
      /* 0.5ms service routine -------------------------------------------------*/
      //Sv_MnSlt();                             // service main slot routine
      //C4_m62s5();                             // check for service 62.5ms routine
      
      /*Toggle port*/
      
      DecreaseUARTRXTimeout(&UART1);            /*Decrease RX1 time out*/
      WALLCommunication(&UART1);                /*Wall 1-4 communication*/
      
      convertADC(ADC_CHSELR_CHSEL15,1,ADCValve);      
      relay_CT();                            // chanatip test relay control
      
      if((++slotcount) == 10 ){
        slotcount =0 ;        
        SPICommunication(SPI1,&SPIDat);
      }               
      /*------------------------------------------------------------------------*/
      if(m0s50_Ct >= 2000) {                    // check 0.5ms counter for 1000ms loop
        m0s50_Ct = 0;
        Sec_Cntr++;                             // increase 1s counter
        // 1s service routine --------------------------------------------------
        //G_AlOptn();                           // get all option
        //Sv_A1Sec();                           // service all 1 second timer
        setOutputIC259(testpos,testdat);
        //-----------------------------------
        if(Sec_Cntr >= 60) {                    // check 1s counter for 60s loop
          Sec_Cntr = 0;
          // 1m service routine ------------------------------------------------
          //Sv_A1Min();                         // service all 1 minute timer          
          //--------------------------------------------------------------------
        }
        //----------------------------------------------------------------------
      }
      //------------------------------------------------------------------------
    }  
    /* End of main loop ########################################################*/           
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**ADC GPIO Configuration  
  PC5   ------> ADC_IN15
  PB0   ------> ADC_IN8 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_8);
  /** Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_15);
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  ADC_InitStruct.Clock = LL_ADC_CLOCK_ASYNC;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_10B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration  
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI1 interrupt Init */
  NVIC_SetPriority(SPI1_IRQn, 0);
  NVIC_EnableIRQ(SPI1_IRQn);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_CC_IRQn, 0);
  NVIC_EnableIRQ(TIM1_CC_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_FROZEN;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration  
  PA8   ------> TIM1_CH1 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 2;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 500;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM3, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**TIM3 GPIO Configuration  
  PC8   ------> TIM3_CH3
  PC9   ------> TIM3_CH4 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);

  /* TIM14 interrupt Init */
  NVIC_SetPriority(TIM14_IRQn, 0);
  NVIC_EnableIRQ(TIM14_IRQn);

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  TIM_InitStruct.Prescaler = 3;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 749;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM14, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM14);
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM15);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM15 GPIO Configuration  
  PB15   ------> TIM15_CH2 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM15, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM15);
  LL_TIM_SetClockSource(TIM15, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM15, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM15);
  LL_TIM_IC_SetActiveInput(TIM15, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM15, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM15, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM15, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_EVEN;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART1);
  LL_USART_DisableOverrunDetect(USART1);
  LL_USART_DisableDMADeactOnRxErr(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration  
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 38400;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART2);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**USART3 GPIO Configuration  
  PB10   ------> USART3_TX
  PB11   ------> USART3_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_4_IRQn, 0);
  NVIC_EnableIRQ(USART3_4_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 38400;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART3);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART4);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  /**USART4 GPIO Configuration  
  PC10   ------> USART4_TX
  PC11   ------> USART4_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USART4 interrupt Init */
  NVIC_SetPriority(USART3_4_IRQn, 0);
  NVIC_EnableIRQ(USART3_4_IRQn);

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_EVEN;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART4, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART4);
  LL_USART_ConfigAsyncMode(USART4);
  LL_USART_Enable(USART4);
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, C_DgtOt1_Pin|C_DgtOt2_Pin|C_DgtOt3_Pin|C_DgtOt4_Pin 
                          |C_DgtOt5_Pin|C_OExpDt_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, A_iNetRS_Pin|A_KbrdSS_Pin|A_MdBsRS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, B_ExpdRS_Pin|HMD_SCL_Pin|HMD_SDA_Pin|RTC_SDL_Pin 
                          |RTC_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, B_CrSwE2_Pin|B_CrSwE1_Pin|B_OExpA2_Pin|B_OExpA1_Pin 
                          |B_OExpA0_Pin|B_OExpE3_Pin|B_OExpE2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, C_CrSwE3_Pin|C_OExpE4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D_OExpE1_GPIO_Port, D_OExpE1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : C_DgtOt1_Pin C_DgtOt2_Pin C_DgtOt3_Pin C_DgtOt4_Pin 
                           C_DgtOt5_Pin */
  GPIO_InitStruct.Pin = C_DgtOt1_Pin|C_DgtOt2_Pin|C_DgtOt3_Pin|C_DgtOt4_Pin 
                          |C_DgtOt5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : C_DgtIn1_Pin C_DgtIn2_Pin C_KBxBRX_Pin */
  GPIO_InitStruct.Pin = C_DgtIn1_Pin|C_DgtIn2_Pin|C_KBxBRX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : A_Ln1Zrc_Pin */
  GPIO_InitStruct.Pin = A_Ln1Zrc_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(A_Ln1Zrc_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A_iNetRS_Pin A_MdBsRS_Pin */
  GPIO_InitStruct.Pin = A_iNetRS_Pin|A_MdBsRS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : A_KbrdSS_Pin */
  GPIO_InitStruct.Pin = A_KbrdSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(A_KbrdSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_ExpdRS_Pin HMD_SCL_Pin HMD_SDA_Pin RTC_SDL_Pin 
                           RTC_SDA_Pin */
  GPIO_InitStruct.Pin = B_ExpdRS_Pin|HMD_SCL_Pin|HMD_SDA_Pin|RTC_SDL_Pin 
                          |RTC_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B_CrSwE2_Pin B_CrSwE1_Pin B_OExpA2_Pin B_OExpA1_Pin 
                           B_OExpA0_Pin B_OExpE3_Pin B_OExpE2_Pin */
  GPIO_InitStruct.Pin = B_CrSwE2_Pin|B_CrSwE1_Pin|B_OExpA2_Pin|B_OExpA1_Pin 
                          |B_OExpA0_Pin|B_OExpE3_Pin|B_OExpE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : C_CrSwE3_Pin C_OExpE4_Pin C_OExpDt_Pin */
  GPIO_InitStruct.Pin = C_CrSwE3_Pin|C_OExpE4_Pin|C_OExpDt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : D_OExpE1_Pin */
  GPIO_InitStruct.Pin = D_OExpE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D_OExpE1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*All function define in main*/
void WALLCommunication(USARTHandler *uart){
  static uint16_t dsp_timeout = 0;
  
  /*Check time out */
  dsp_timeout++;
  if(dsp_timeout > (100/0.5)){
    DisableRXUART(USART1);
    FlushRXData(USART1);
    dsp_timeout = 0;
    __disable_interrupt();
    DisableTXEmptyInterrupt(USART1);
    DisableTXCptInterrupt(USART1);
    DisableRXEmptyInterrupt(USART1);
    __enable_interrupt();
    ConnectState = PREPARE_DATA;
  }
  switch(ConnectState){
  case PREPARE_DATA:
    TX1Data[0] = 0xA1;
    TX1Data[1] = DSP1_TAB[((KeyData >>4)& 0x0F)];
    TX1Data[2] = DSP1_TAB[(KeyData & 0x0F)];;
    TX1Data[3] = (uint8_t) (~(1<<4));
    TX1Data[4] = 0;    
    TX1Data[5] = (TX1Data[0]+TX1Data[1]+TX1Data[2]+TX1Data[3]+TX1Data[4]) ^ 0xFF;
    
    /*Set pointer and counter*/
    /*-----------------------------------------------------*/
    /*Set Tx count*/
    uart->TXCount = 1;    
    /* point to data first byte which send interupt*/
    uart->TXPointer = &TX1Data[1];              
    /*Set Rx count*/
    uart->RXCount = 0;
    /*Set RX pointer*/
    uart->RXPointer = &RX1Data[0];
    /*Write 1st byte (header)*/
    USART1->TDR = TX1Data[0];
    /*Enable TX empty*/
    __disable_interrupt();
    EnableTXEmptyInterrupt(USART1);
    __enable_interrupt();
    /*-----------------------------------------------------*/
    
    ConnectState = WAIT_SEND_COMPLETE;              
    break;
    
  case WAIT_SEND_COMPLETE:
  case CHANGE_TO_RX:
    break;
    
  case RECEIVED_DATA:
    if(((uart->RXCount) == RX1_SIZE) && (RX1Data[0] == 0xA9)){
      /*Received all bytes and header correct*/
      KeyData = RX1Data[2];
      __no_operation();
    }
    break;
    
  case WAIT_IDLE:
    break;  
  }
}

void SPICommunication(SPI_TypeDef *SPICH,SPIDataHandler *spi){
  /*Example code for SPI*/
  static uint8_t spi_timer=0;
  
  if(++spi_timer == 20){
    spi_timer = 0;
    /*Resend data*/
    spi->SPIState = SPI_PREPARE;
  }
  
  switch(spi->SPIState){
  case SPI_PREPARE:
    /*Clear data before send*/
    for(uint8_t i=0;i<SPI_TX_SIZE;i++){
      SPITXData[i] =0;
    }
    SPITXData[0] = 0xA1;
    SPITXData[1] = 0x12;
    SPITXData[2] = 0x23;
    SPITXData[3] = Sec_Cntr;
    SPITXData[10] = 0xA9;
    /*Byte 18*/
    SPITXData[17] = LCD_1;
    /*byte 19*/
    SPITXData[18] = LCD_2;
    /*byte 20*/
    SPITXData[19] = LCD_3;
    /*byte 21*/
    SPITXData[20] = LCD_4;
    /*byte 22*/
    SPITXData[21] = LCD_5;
    /*byte 23*/
    SPITXData[22] = LCD_6;
    /*byte 24*/
    SPITXData[23] = LCD_7;
    /*byte 25*/
    SPITXData[24] = LCD_8;
    /*byte 26*/
    SPITXData[25] = LCD_9;
    
    /*Calculate check sum*/
    for(uint8_t i = 0;i< (SPI_TX_SIZE-1);i++){
      SPITXData[SPI_TX_SIZE-1] += SPITXData[i];
    }
    SPITXData[SPI_TX_SIZE-1] ^= 0xFF;    
    
    /*Clear RX buffer*/
    for(uint8_t i = 0;i < SPI_RX_SIZE;i++){
      SPIRXData[i] = 0;
    }
    
    /*Set SPI TX & RX pointer*/
    spi->TXPointer = SPITXData;
    spi->RXPointer = SPIRXData;
    
    /*Set Max  counter to limit data*/
    spi->RXMaxByte = SPI_RX_SIZE;
    spi->TXMaxByte = SPI_TX_SIZE;
    /*Chip select*/
    CS_PORT->ODR &= ~(1<<CS_PIN);
    
    
    /*Set to wait state*/
    spi->SPIState = SPI_WAIT;
    break;
    
  case SPI_WAIT:    
    
    /*Send SPI Data*/
    sendSPIDisplay(SPICH,spi);  
    
    spi->SPIState = SPI_DATA_READY;
    break;
    
  case SPI_DATA_READY:
    break;    
  }  
}



/*Call Back funtion from other file*/
void InitUARTHandler(void){
  
  UART1.Instance =UART_CH2;
  UART1.Instance =UART_CH3;
  UART1.Instance =UART_CH4;
  
  /*Setting data handler for UART channel 1*/
  UART1.Instance =UART_CH1;                     /*Assign instance*/
  UART1.RXTimeOutReload = UART1_RX_TIME_OUT;    /*Set RX time out*/
  UART1.RXMaxByte =RX1_SIZE;                    /*Set RX size*/
  UART1.TXMaxByte =TX1_SIZE;                    /*Set TX size*/
}

void InitTimerCallBack(void){  
  /*Initialze timer14 */
  /*first stop timer if running*/
  TIM14->CR1 &= ~TIM_CR1_CEN;
  /*enable update/overflow interrupt*/
  TIM14->DIER |= TIM_DIER_UIE;
  /*start timer*/
  TIM14->CR1 |= TIM_CR1_CEN;   
  /*PWM out*/
  TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
  TIM3->CR1 |= TIM_CR1_CEN;
}

void UART_TXComplete_CallBack(USARTHandler *UARTDat){
  switch(UARTDat->Instance){
  case UART_CH1:
    /*Enable RX ,interrupt and flush data*/
    EnableRXUART(USART1);
    /*Flush RX FIFO*/
    FlushRXData(USART1);
    /*Enalbe RX empty interrupt*/
    EnableRXEmptyInterrupt(USART1);
    /*Restart rx timeout first byte*/
    UARTDat->RXTimeOut = 200;
    
    break;
  case UART_CH2:
    break;
  case UART_CH3:
    break;
  case UART_CH4:    
    break;     
  }
}

void UART_RXComplete_CallBack(USARTHandler *UARTDat){
  switch(UARTDat->Instance){
  case UART_CH1:
    DisableRXUART(USART1);
    DisableRXEmptyInterrupt(USART1);
    FlushRXData(USART1);
    ConnectState = RECEIVED_DATA;
    break;
  case UART_CH2:
    break;
  case UART_CH3:
    break;
  case UART_CH4:
    break;
  }
}

/*Call Back funtion from interrupt*/
void TIM14_interruptCallBack(void){
  /*This routine call from interrupt timer 14*/
  
  static uint8_t Intr_SlC = 0;
  /*Check update flag and clear if set to clear pending*/
  if((TIM14->SR & TIM_SR_UIF)== TIM_SR_UIF)     TIM14->SR &= ~TIM_SR_UIF;
  /*Increase 0.625 ms*/
  m0s0625C++;     
    
  /*Increase interrupt slot*/
  if(++Intr_SlC >=5) Intr_SlC = 0;
  switch(Intr_SlC){
    
  default:
  case 0:
    break;
    
  case 1:
    break;
      
  case 2:
    break;
    
  case 3: 
      break;
      
  case 4:
    break;          
  }  
}

/*routine for delay*/
void waitDelay(uint16_t count){
  while(count--);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
