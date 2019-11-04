#include  "relayct.h"
extern RlyCctGp FnRlC_St;                           // final relay circuit status
extern RlyCctGp Genr_RlC;                           // general relay circuit
extern RlyCctGp Tmp_RlCc;                           // temporary relay circuit data

extern Typedef_ParaTb01 ParaTb01;
extern Typedef_ParaTb2a ParaTb2a; 
extern Typedef_ParaTb2b ParaTb2b; 
extern Typedef_ParaTb04 ParaTb04;
extern Typedef_ParaTb05 ParaTb05;                                  // parameter table 5
extern EpO259Gp EpO_Data,EpO_PtSt,EpO_PtBf;

extern MbtUint8 GenFg_01;                  // general purpose flag1
extern MbtUint8 GenFg_02;                  // general purpose flag2
extern MbtUint8 GenFg_03;                  // general purpose flag3
extern MbtUint8 GenFg_04;                  // general purpose flag4
extern MbtUint8 GenFg_05;                  // general purpose flag6
extern MbtUint8 GenFg_06;                  // general purpose flag5
extern MbtUint8 GenFg_07;                  // general purpose flag7
extern MbtUint8 GenFg_08;                  // general purpose flag8
extern MbtUint8 GenFg_09;                  // general purpose flag9
extern MbtUint8 GenFg_10;                  // general purpose flag10
extern MbtUint8 GenFg_11;                  // general purpose flag11
extern MbtUint8 GenFg_12;                  // general purpose flag12
extern MbtUint8 GenFg_13;                  // general purpose flag13
extern unsigned char CmMd_Lv1,CmMd_Lv2;
extern MbtUint8 HwTs_Buf[HwTBf_Sz];                 // hardware testing buffer
extern unsigned char Ct_StTmp;                      // control setting temperature index
extern signed char VlC_DfTp;
extern unsigned char FSC_DfTp;                      // fan speed control different temperature index
extern unsigned char FnAuSp_S;                      // fan auto speed state
extern unsigned char Sen1_Tmp;                      // sensor1 temperature index
extern unsigned char Room_Tmp;                      // room temperature index
extern unsigned char Room_Hum;                      // room humidity (0-100%RH)
extern const unsigned char FnAS_HyL[];
extern const unsigned char FnAS_HyU[];
//-------------------------------------------
extern unsigned char NbdSt_Mn;                      // nobody stays timer (m)
extern unsigned short NbdSt_ms;                     // nobody stays timer (20ms)
extern unsigned char AcTSS_Mn;                      // air-con setting temperature saving step timer (m)
extern unsigned char AcTSS_ms;                      // air-con setting temperature saving step timer (20ms)
extern unsigned char AcTS_Ofs;                      // air-con setting temperature saving offset temperature (0.5C)
//-------------------------------------------
extern unsigned char AcNMt_Mn;                      // air-con no-motion timer (m)
extern unsigned short AcNMt_ms;                     // air-con no-motion timer (20ms)
extern unsigned char A2NMt_Mn;                      // air-con2 no-motion timer (m)
extern unsigned short A2NMt_ms;                     // air-con2 no-motion timer (20ms)
//-------------------------------------------
extern unsigned char CpOfD_Mn;                      // compressor off delay (m)
extern unsigned char CpOfD_Sc;                      // compressor off delay (s)
extern unsigned short Val_ItCy;                     // valve interval cycle timer (s)
//==============================================================================
// minimum limited pointed byte data
unsigned char MnL_PtBy(unsigned char *ptr, unsigned char min) {
  if(*ptr < min) {
    *ptr = min;                    // force to minimum
    return(1);                     // set limited value flag
  }
  return(0);                       // clear limited value flag
}
//------------------------------------------------------------------------------
// maximum limited pointed byte data
unsigned char MxL_PtBy(unsigned char *ptr, unsigned char max) {
  if(*ptr > max) {
    *ptr = max;                    // force to maximum
    return(1);                     // set limited value flag
  }
  return(0);                       // clear limited value flag
}


//==============================================================================
// limit control setting temperature
unsigned char Lm_CtSTp(unsigned char *ptr) {
  if(MnL_PtBy(ptr,ParaTb2b.P.MnCtST_P)) {    // minimum limit
    return(1);                               // set limited value flag
  }
  if(MxL_PtBy(ptr,ParaTb2b.P.MxCtST_P)) {    // maximum limit
    return(1);                               // set limited value flag
  }
  return(0);                                 // clear limited value flag
}
//==============================================================================
// offset with air-con setting temperature saving offset temperature
void OW_AcTSO(unsigned char *temp_index) {
  unsigned char tmp1;
  if(AcTS_Ofs) {                             // check air-con setting temperature saving offset temperature
    tmp1 = *temp_index;
    if(ParaTb01.P.Arcn_Mod == AcMd_Col) {    // check cooling mode
      tmp1 += AcTS_Ofs;                      // add temperature index with air-con setting temperature saving offset temperature
    }
    MnL_PtBy(&tmp1,TbI__15C);                // minimum limit
    MxL_PtBy(&tmp1,TbI__30C);                // maximum limit
    *temp_index = tmp1;                      // save result to temperature index
  }
}
//==============================================================================
// get control different temperature
void G_CtDfTp(void) {
  unsigned char tmp1,diftmp,lower,upper;
  tmp1 = ParaTb01.P.Ac_SetTp;                          // get control setting temperature
  OW_AcTSO(&tmp1);                                     // offset with air-con setting temperature saving offset temperature
  StTp_Lmt = 0;                                        // assume clear setting temperature limited flag
  if(Key_Stay) {                                       // if set key stay flag
    if(Lm_CtSTp(&tmp1)) {                              // limit max/min control setting temperature
      StTp_Lmt = 1;                                    // set setting temperature limited flag
    }
  }
  Ct_StTmp = tmp1;
  //---------------------------------------------------
  VlC_DfTp = Room_Tmp-Ct_StTmp;                        // get valve control different temperature
  //---------------------------------------------------
  // fan auto speed control
  if(VlC_DfTp < 0) {                                   // if valve control different temperature < 0C
    FnAuSp_S = FnAuS_Lw;                               // set fan auto low speed
    FSC_DfTp = 2*(0)+0;                                // fan speed control different temperature = 0C
  } else {
    diftmp = VlC_DfTp;
    if(diftmp > FSC_DfTp) FSC_DfTp++;                  // if diff temperature is increasing
    else if(diftmp < FSC_DfTp) FSC_DfTp--;             // if diff temperature is decreasing
    //---------------------------------------
    // get lower threshold of the hysteresis
    // if data <= ROM data (on base 8 bit) then finish search
    lower = 0;
    while(FSC_DfTp > FnAS_HyL[lower]) lower++;         // increase fan speed
    // get upper threshold of the hysteresis
    // if data <= ROM data (on base 8 bit) then finish search
    upper = 0;
    while(FSC_DfTp > FnAS_HyU[upper]) upper++;         // increase fan speed
    if(lower == upper) FnAuSp_S = lower;               // check same state of lower and upper threshold to save fan auto speed state
    //---------------------------------------
  }
}
//==============================================================================
// to off valve relay
void Of_ValRl(void) {
  if(RlValv_S) {                             // check for valve is on
    CpOfD_Mn = ParaTb2b.P.CmpOfD_P;          // restart compressor off delay
    CpOfD_Sc = 0;
    Val_ItCy = SsE_Vl0T;                     // restart valve interval cycle off time
  }
  RlValv_S = 0;                              // off valve
}
//------------------------------------------------------------------------------
// clear compressor off delay time
void Cl_Cp0Dl(void) {
  CpOfD_Mn = 0;                              // clear compressor off delay timer
  CpOfD_Sc = 0;
  C1Cp0D_F = 1;                              // set cleared first compressor off delay flag
}
//------------------------------------------------------------------------------
// force on valve relay
void F1_ValRl(void) {
  if(!RlValv_S) {                            // check for valve is off
    Val_ItCy = SsE_Vl1T;                     // restart valve interval cycle on time
  }
  RlValv_S = 1;                              // on valve
  Cl_Cp0Dl();                                // clear compressor off delay timer
}
//------------------------------------------------------------------------------
// to on valve relay
void On_ValRl(void) {
  if((!CpOfD_Mn)&&(!CpOfD_Sc)) {             // check compressor off delay timer
    F1_ValRl();                              // force on valve relay
  } else Of_ValRl();                         // off valve if timer is not expired
}
//------------------------------------------------------------------------------
// valve with temperature control
void Val_TpCt(void) {
  if(RmTpS_Er) {                             // if set room temperature sensor error flag
#if(U_A1VlRE)
    On_ValRl();                              // to on valve
#else
    if(!Val_ItCy) {                          // check valve interval cycle timer is expired
      if(RlValv_S) Of_ValRl();               // off valve if valve is on
      else On_ValRl();                       // on valve if valve is off
    }
#endif
  } else {
    //---------------------------------------
    // temperature control for cooling mode
    if(ACoHyT_E) {                                     // if set air-con colder hysteresis temperature enable flag
      if(Key_Stay) {                                   // if set key stay flag
        if(VlC_DfTp >= (2*(0)+0)) On_ValRl();          // on valve if valve control different temperature >= 0C
        else if(VlC_DfTp <= (2*(-1)-0)) Of_ValRl();    // off valve if valve control different temperature <= -1C
      } else {
        if(VlC_DfTp >= (2*(1)+0)) On_ValRl();          // on valve if valve control different temperature >= 1C
        else if(VlC_DfTp <= (2*(-1)-0)) Of_ValRl();    // off valve if valve control different temperature <= -1C
      }
    } else {
      if(VlC_DfTp >= (2*(1)+0)) On_ValRl();            // on valve if valve control different temperature >= 1C
      else if(VlC_DfTp <= (2*(0)+0)) Of_ValRl();       // off valve if valve control different temperature <= 0C
    }
    //---------------------------------------
  }
}
//==============================================================================
// valve relay control
void ValRl_Ct(void) {
#if(U_VRV2CR)
  if(AcnMdu_E                                               // if set air-con module enable flag
     &&VRV2CR_E                                             //    and enable VRV 2-dry contact relay
    ) {
  #if(ExAc_Mdl == MtSuBS_M)
    switch(VRVDC_St) {                            // check VRV dry contact state
      case VCS_UsCt:                              // user control state
        if(!VRVC_ChD) {                           // check VRV dry contact changing delay is expired
          F1_ValRl();                             // force on valve relay
        }
        break;
      default:
        Of_ValRl();                               // off valve
        break;
    }
  #elif(ExAc_Mdl == ToSBCr_M)
    switch(VRVDC_St) {                            // check VRV dry contact state
      case VCS_UsCt:                              // user control state
      case VCS_PeCl:                              // pre-cool state
        F1_ValRl();                               // force on valve relay
        break;
      default:
        Of_ValRl();                               // off valve
        break;
    }
  #endif
#else
  if(0) {
#endif
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAc2Rl_E                                      //    and enable external air-con2 relay
            &&(ParaTb2b.P.ExAc2_RN == IAc_ValR)             //    by valve relay
           ) {
    if(EAc2Rl_S) {                                // if set external air-con2 relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAc3Rl_E                                      //    and enable external air-con3 relay
            &&(ParaTb2b.P.ExAc3_RN == IAc_ValR)             //    by valve relay
           ) {
    if(EAc3Rl_S) {                                // if set external air-con3 relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAc4Rl_E                                      //    and enable external air-con4 relay
            &&(ParaTb2b.P.ExAc4_RN == IAc_ValR)             //    by valve relay
           ) {
    if(EAc4Rl_S) {                                // if set external air-con4 relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&(!EAcnRl_E)                                   //    and disable external air-con relay
           ) {
    if(FnArcn_S                                   // if air-con is on
       &&(!AcBDO0_F)                              //    and clear air-con balcony door open shut off mode status flag
       &&(!AcNMt0_F)                              //    and clear air-con no-motion shut off mode status flag
      ) {
      switch(ParaTb01.P.Arcn_Mod) {               // check air-con operating mode
        case AcMd_Col:
          Val_TpCt();                             // valve with temperature control
          break;
        default:
          Of_ValRl();                             // off valve
          break;
      }
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAcnRl_E                                      //    and enable external air-con relay
            &&(ParaTb2b.P.ExAcn_RN == IAc_ValR)             //    by valve relay
           ) {
    if(ExAcRl_S) {                                // if set external air-con relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
#if(U_CSpkRl)
  } else if(CSpkRl_E                                        // if enable chime speaker relay
            &&(ParaTb2b.P.CmSpk_RN == IAc_ValR)             // by valve relay
           ) {
    if(CSpkRl_S) {                                // if set chime speaker relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
#endif
#if(U_SevSRl)
  } else if(DNDSRl_E                                        // if enable do not disturb sign relay
            &&(ParaTb2b.P.DNDSg_RN == IAc_ValR)             // by valve relay
           ) {
    if(DNDSRl_S) {                                // if set do not disturb sign relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
  } else if(MURSRl_E                                        // if enable make up room sign relay
            &&(ParaTb2b.P.MURSg_RN == IAc_ValR)             // by valve relay
           ) {
    if(MURSRl_S) {                                // if set make up room sign relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
  } else if(PWaSRl_E                                        // if enable please wait sign relay
            &&(ParaTb2b.P.PWaSg_RN == IAc_ValR)             // by valve relay
           ) {
    if(PWaSRl_S) {                                // if set please wait sign relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
#endif
#if(Us_CbnSw)
  } else if(Cbn1Sw_E                                        // if enable cabinet1 switch
            &&(ParaTb2b.P.Cbnt1_RN == IAc_ValR)             // by valve relay
           ) {
    if(Cbn1Rl_S) {                                // if set cabinet1 relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
  } else if(Cbn2Sw_E                                        // if enable cabinet2 switch
            &&(ParaTb2b.P.Cbnt2_RN == IAc_ValR)             // by valve relay
           ) {
    if(Cbn2Rl_S) {                                // if set cabinet2 relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
  #if(Ep_CbnSw)
  } else if(Cbn3Sw_E                                        // if enable cabinet3 switch
            &&(ParaTb2b.P.Cbnt3_RN == IAc_ValR)             // by valve relay
           ) {
    if(Cbn3Rl_S) {                                // if set cabinet3 relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
  } else if(Cbn4Sw_E                                        // if enable cabinet4 switch
            &&(ParaTb2b.P.Cbnt4_RN == IAc_ValR)             // by valve relay
           ) {
    if(Cbn4Rl_S) {                                // if set cabinet4 relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
  } else if(Cbn5Sw_E                                        // if enable cabinet5 switch
            &&(ParaTb2b.P.Cbnt5_RN == IAc_ValR)             // by valve relay
           ) {
    if(Cbn5Rl_S) {                                // if set cabinet5 relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
  #endif
#endif
#if(Us_Curtn)
  } else if(Curtn1_E                                        // if enable curtain1 open relay
            &&(ParaTb2b.P.Cut1O_RN == IAc_ValR)             // by valve relay
           ) {
    if(Ct1ORl_S) {                                // if set curtain1 open relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
  } else if(Curtn1_E                                        // if enable curtain1 close relay
            &&(ParaTb2b.P.Cut1C_RN == IAc_ValR)             // by valve relay
           ) {
    if(Ct1CRl_S) {                                // if set curtain1 close relay status flag
      F1_ValRl();                                 // force on valve relay
    } else Of_ValRl();                            // off valve
    //------------------------------------------------------
#endif
  } else Of_ValRl();                                        // off valve
}
//==============================================================================
// to off fan high speed relay
void Of_FHiRl(void) {
  RlFnHi_S = 0;                              // off fan high speed relay
}
//------------------------------------------------------------------------------
// to on fan high speed relay
void On_FHiRl(void) {
  RlFnHi_S = 1;                              // on fan high speed relay
}
//------------------------------------------------------------------------------
// to off fan medium speed relay
void Of_FMdRl(void) {
  RlFnMd_S = 0;                              // off fan medium speed relay
}
//------------------------------------------------------------------------------
// to on fan medium speed relay
void On_FMdRl(void) {
  RlFnMd_S = 1;                              // on fan medium speed relay
}
//------------------------------------------------------------------------------
// to off fan low speed relay
void Of_FLwRl(void) {
  RlFnLw_S = 0;                              // off fan low speed relay
}
//------------------------------------------------------------------------------
// to on fan low speed relay
void On_FLwRl(void) {
  RlFnLw_S = 1;                              // on fan low speed relay
}
//==============================================================================
// fan control
void FanRl_Ct(void) {
  //-----------------------------------------------------------------------
  // fan high speed relay control
#if(U_VRV2CR)
  if(AcnMdu_E                                               // if set air-con module enable flag
     &&VRV2CR_E                                             //    and enable VRV 2-dry contact relay
    ) {
  #if(ExAc_Mdl == MtSuBS_M)
    switch(VRVDC_St) {                            // check VRV dry contact state
      case VCS__Off:                              // off state
        On_FHiRl();                               // on fan high speed relay
        break;
      default:
        Of_FHiRl();                               // off fan high speed relay
        break;
    }
  #elif(ExAc_Mdl == ToSBCr_M)
    switch(VRVDC_St) {                            // check VRV dry contact state
      case VCS_UsCt:                              // user control state
      case VCS_PeCl:                              // pre-cool state
        On_FHiRl();                               // on fan high speed relay
        break;
      default:
        if(!VRVC_ChD) {                           // check VRV dry contact changing delay is expired
          Of_FHiRl();                             // off fan high speed relay
        }
        break;
    }
  #endif
#else
  if(0) {
#endif
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAc2Rl_E                                      //    and enable external air-con2 relay
            &&(ParaTb2b.P.ExAc2_RN == IAc_FHiR)             //    by fan high speed relay
           ) {
    if(EAc2Rl_S) {                                // if set external air-con2 relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAc3Rl_E                                      //    and enable external air-con3 relay
            &&(ParaTb2b.P.ExAc3_RN == IAc_FHiR)             //    by fan high speed relay
           ) {
    if(EAc3Rl_S) {                                // if set external air-con3 relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAc4Rl_E                                      //    and enable external air-con4 relay
            &&(ParaTb2b.P.ExAc4_RN == IAc_FHiR)             //    by fan high speed relay
           ) {
    if(EAc4Rl_S) {                                // if set external air-con4 relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&(!EAcnRl_E)                                   //    and disable external air-con relay
           ) {
    if(FnArcn_S                                   // if air-con is on
       &&((!AcBDO0_F)                             //    and clear air-con balcony door open shut off mode status flag
          ||(!BDOFP_F0)                           //        or clear balcony door open fan power force off flag (0=maintain,1=force off)
         )
       &&(!AcNMt0_F)                              //    and clear air-con no-motion shut off mode status flag
       &&(Key_Stay                                //    and set key stay flag
          ||SbAFP_F1||RlValv_S                    //        or check standby air-con fan power force on flag (0=as valve,1=force on)
         )
       &&((ParaTb01.P.Fan_Sped == Fan_High)       // check fan speed
          ||((ParaTb01.P.Fan_Sped == Fan_Auto)    // check fan auto speed
             &&(FnAuSp_S == FnAuS_Hi)
            )
         )
      ) On_FHiRl();                               // on fan high speed relay
    else Of_FHiRl();                              // off fan high speed relay
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAcnRl_E                                      //    and enable external air-con relay
            &&(ParaTb2b.P.ExAcn_RN == IAc_FHiR)             //    by fan high speed relay
           ) {
    if(ExAcRl_S) {                                // if set external air-con relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
#if(U_CSpkRl)
  } else if(CSpkRl_E                                        // if enable chime speaker relay
            &&(ParaTb2b.P.CmSpk_RN == IAc_FHiR)             // by fan high speed relay
           ) {
    if(CSpkRl_S) {                                // if set chime speaker relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
#endif
#if(U_SevSRl)
  } else if(DNDSRl_E                                        // if enable do not disturb sign relay
            &&(ParaTb2b.P.DNDSg_RN == IAc_FHiR)             // by fan high speed relay
           ) {
    if(DNDSRl_S) {                                // if set do not disturb sign relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
  } else if(MURSRl_E                                        // if enable make up room sign relay
            &&(ParaTb2b.P.MURSg_RN == IAc_FHiR)             // by fan high speed relay
           ) {
    if(MURSRl_S) {                                // if set make up room sign relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
  } else if(PWaSRl_E                                        // if enable please wait sign relay
            &&(ParaTb2b.P.PWaSg_RN == IAc_FHiR)             // by fan high speed relay
           ) {
    if(PWaSRl_S) {                                // if set please wait sign relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
#endif
#if(Us_CbnSw)
  } else if(Cbn1Sw_E                                        // if enable cabinet1 switch
            &&(ParaTb2b.P.Cbnt1_RN == IAc_FHiR)             // by fan high speed relay
           ) {
    if(Cbn1Rl_S) {                                // if set cabinet1 relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
  } else if(Cbn2Sw_E                                        // if enable cabinet2 switch
            &&(ParaTb2b.P.Cbnt2_RN == IAc_FHiR)             // by fan high speed relay
           ) {
    if(Cbn2Rl_S) {                                // if set cabinet2 relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
  #if(Ep_CbnSw)
  } else if(Cbn3Sw_E                                        // if enable cabinet3 switch
            &&(ParaTb2b.P.Cbnt3_RN == IAc_FHiR)             // by fan high speed relay
           ) {
    if(Cbn3Rl_S) {                                // if set cabinet3 relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
  } else if(Cbn4Sw_E                                        // if enable cabinet4 switch
            &&(ParaTb2b.P.Cbnt4_RN == IAc_FHiR)             // by fan high speed relay
           ) {
    if(Cbn4Rl_S) {                                // if set cabinet4 relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
  } else if(Cbn5Sw_E                                        // if enable cabinet5 switch
            &&(ParaTb2b.P.Cbnt5_RN == IAc_FHiR)             // by fan high speed relay
           ) {
    if(Cbn5Rl_S) {                                // if set cabinet5 relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
  #endif
#endif
#if(Us_Curtn)
  } else if(Curtn1_E                                        // if enable curtain1 open relay
            &&(ParaTb2b.P.Cut1O_RN == IAc_FHiR)             // by fan high speed relay
           ) {
    if(Ct1ORl_S) {                                // if set curtain1 open relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
  } else if(Curtn1_E                                        // if enable curtain1 close relay
            &&(ParaTb2b.P.Cut1C_RN == IAc_FHiR)             // by fan high speed relay
           ) {
    if(Ct1CRl_S) {                                // if set curtain1 close relay status flag
      On_FHiRl();                                 // on fan high speed relay
    } else Of_FHiRl();                            // off fan high speed relay
    //------------------------------------------------------
#endif
  } else Of_FHiRl();                                        // off fan high speed relay
  //-----------------------------------------------------------------------
  // fan medium speed relay control
#if(U_VRV2CR)
  if(AcnMdu_E                                               // if set air-con module enable flag
     &&VRV2CR_E                                             //    and enable VRV 2-dry contact relay
    ) {
    Of_FMdRl();                                   // off fan medium speed relay
    //------------------------------------------------------
#else
  if(0) {
#endif
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAc2Rl_E                                      //    and enable external air-con2 relay
            &&(ParaTb2b.P.ExAc2_RN == IAc_FMdR)             //    by fan medium speed relay
           ) {
    if(EAc2Rl_S) {                                // if set external air-con2 relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAc3Rl_E                                      //    and enable external air-con3 relay
            &&(ParaTb2b.P.ExAc3_RN == IAc_FMdR)             //    by fan medium speed relay
           ) {
    if(EAc3Rl_S) {                                // if set external air-con3 relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAc4Rl_E                                      //    and enable external air-con4 relay
            &&(ParaTb2b.P.ExAc4_RN == IAc_FMdR)             //    by fan medium speed relay
           ) {
    if(EAc4Rl_S) {                                // if set external air-con4 relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&(!EAcnRl_E)                                   //    and disable external air-con relay
           ) {
    if(FnArcn_S                                   // if air-con is on
       &&((!AcBDO0_F)                             //    and clear air-con balcony door open shut off mode status flag
          ||(!BDOFP_F0)                           //        or clear balcony door open fan power force off flag (0=maintain,1=force off)
         )
       &&(!AcNMt0_F)                              //    and clear air-con no-motion shut off mode status flag
       &&(Key_Stay                                //    and set key stay flag
          ||SbAFP_F1||RlValv_S                    //        or check standby air-con fan power force on flag (0=as valve,1=force on)
         )
       &&((ParaTb01.P.Fan_Sped == Fan__Med)       // check fan speed
          ||((ParaTb01.P.Fan_Sped == Fan_Auto)    // check fan auto speed
             &&(FnAuSp_S == FnAuS_Md)
            )
         )
      ) On_FMdRl();                               // on fan medium speed relay
    else Of_FMdRl();                              // off fan medium speed relay
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAcnRl_E                                      //    and enable external air-con relay
            &&(ParaTb2b.P.ExAcn_RN == IAc_FMdR)             //    by fan medium speed relay
           ) {
    if(ExAcRl_S) {                                // if set external air-con relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
#if(U_CSpkRl)
  } else if(CSpkRl_E                                        // if enable chime speaker relay
            &&(ParaTb2b.P.CmSpk_RN == IAc_FMdR)             // by fan medium speed relay
           ) {
    if(CSpkRl_S) {                                // if set chime speaker relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
#endif
#if(U_SevSRl)
  } else if(DNDSRl_E                                        // if enable do not disturb sign relay
            &&(ParaTb2b.P.DNDSg_RN == IAc_FMdR)             // by fan medium speed relay
           ) {
    if(DNDSRl_S) {                                // if set do not disturb sign relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
  } else if(MURSRl_E                                        // if enable make up room sign relay
            &&(ParaTb2b.P.MURSg_RN == IAc_FMdR)             // by fan medium speed relay
           ) {
    if(MURSRl_S) {                                // if set make up room sign relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
  } else if(PWaSRl_E                                        // if enable please wait sign relay
            &&(ParaTb2b.P.PWaSg_RN == IAc_FMdR)             // by fan medium speed relay
           ) {
    if(PWaSRl_S) {                                // if set please wait sign relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
#endif
#if(Us_CbnSw)
  } else if(Cbn1Sw_E                                        // if enable cabinet1 switch
            &&(ParaTb2b.P.Cbnt1_RN == IAc_FMdR)             // by fan medium speed relay
           ) {
    if(Cbn1Rl_S) {                                // if set cabinet1 relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
  } else if(Cbn2Sw_E                                        // if enable cabinet2 switch
            &&(ParaTb2b.P.Cbnt2_RN == IAc_FMdR)             // by fan medium speed relay
           ) {
    if(Cbn2Rl_S) {                                // if set cabinet2 relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
  #if(Ep_CbnSw)
  } else if(Cbn3Sw_E                                        // if enable cabinet3 switch
            &&(ParaTb2b.P.Cbnt3_RN == IAc_FMdR)             // by fan medium speed relay
           ) {
    if(Cbn3Rl_S) {                                // if set cabinet3 relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
  } else if(Cbn4Sw_E                                        // if enable cabinet4 switch
            &&(ParaTb2b.P.Cbnt4_RN == IAc_FMdR)             // by fan medium speed relay
           ) {
    if(Cbn4Rl_S) {                                // if set cabinet4 relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
  } else if(Cbn5Sw_E                                        // if enable cabinet5 switch
            &&(ParaTb2b.P.Cbnt5_RN == IAc_FMdR)             // by fan medium speed relay
           ) {
    if(Cbn5Rl_S) {                                // if set cabinet5 relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
  #endif
#endif
#if(Us_Curtn)
  } else if(Curtn1_E                                        // if enable curtain1 open relay
            &&(ParaTb2b.P.Cut1O_RN == IAc_FMdR)             // by fan medium speed relay
           ) {
    if(Ct1ORl_S) {                                // if set curtain1 open relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
  } else if(Curtn1_E                                        // if enable curtain1 close relay
            &&(ParaTb2b.P.Cut1C_RN == IAc_FMdR)             // by fan medium speed relay
           ) {
    if(Ct1CRl_S) {                                // if set curtain1 close relay status flag
      On_FMdRl();                                 // on fan medium speed relay
    } else Of_FMdRl();                            // off fan medium speed relay
    //------------------------------------------------------
#endif
  } else Of_FMdRl();                                        // off fan medium speed relay
  //-----------------------------------------------------------------------
  // fan low speed relay control
#if(U_VRV2CR)
  if(AcnMdu_E                                               // if set air-con module enable flag
     &&VRV2CR_E                                             //    and enable VRV 2-dry contact relay
    ) {
    Of_FLwRl();                                   // off fan low speed relay
    //------------------------------------------------------
#else
  if(0) {
#endif
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAc2Rl_E                                      //    and enable external air-con2 relay
            &&(ParaTb2b.P.ExAc2_RN == IAc_FLwR)             //    by fan low speed relay
           ) {
    if(EAc2Rl_S) {                                // if set external air-con2 relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAc3Rl_E                                      //    and enable external air-con3 relay
            &&(ParaTb2b.P.ExAc3_RN == IAc_FLwR)             //    by fan low speed relay
           ) {
    if(EAc3Rl_S) {                                // if set external air-con3 relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAc4Rl_E                                      //    and enable external air-con4 relay
            &&(ParaTb2b.P.ExAc4_RN == IAc_FLwR)             //    by fan low speed relay
           ) {
    if(EAc4Rl_S) {                                // if set external air-con4 relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&(!EAcnRl_E)                                   //    and disable external air-con relay
           ) {
    if(FnArcn_S                                   // if air-con is on
       &&((!AcBDO0_F)                             //    and clear air-con balcony door open shut off mode status flag
          ||(!BDOFP_F0)                           //        or clear balcony door open fan power force off flag (0=maintain,1=force off)
         )
       &&(!AcNMt0_F)                              //    and clear air-con no-motion shut off mode status flag
       &&(Key_Stay                                //    and set key stay flag
          ||SbAFP_F1||RlValv_S                    //        or check standby air-con fan power force on flag (0=as valve,1=force on)
         )
       &&((ParaTb01.P.Fan_Sped == Fan__Low)       // check fan speed
          ||((ParaTb01.P.Fan_Sped == Fan_Auto)    // check fan auto speed
             &&(FnAuSp_S == FnAuS_Lw)
            )
         )
      ) On_FLwRl();                               // on fan low speed relay
    else Of_FLwRl();                              // off fan low speed relay
    //------------------------------------------------------
  } else if(AcnMdu_E                                        // if set air-con module enable flag
            &&EAcnRl_E                                      //    and enable external air-con relay
            &&(ParaTb2b.P.ExAcn_RN == IAc_FLwR)             //    by fan low speed relay
           ) {
    if(ExAcRl_S) {                                // if set external air-con relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
#if(U_CSpkRl)
  } else if(CSpkRl_E                                        // if enable chime speaker relay
            &&(ParaTb2b.P.CmSpk_RN == IAc_FLwR)             // by fan low speed relay
           ) {
    if(CSpkRl_S) {                                // if set chime speaker relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
#endif
#if(U_SevSRl)
  } else if(DNDSRl_E                                        // if enable do not disturb sign relay
            &&(ParaTb2b.P.DNDSg_RN == IAc_FLwR)             // by fan low speed relay
           ) {
    if(DNDSRl_S) {                                // if set do not disturb sign relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
  } else if(MURSRl_E                                        // if enable make up room sign relay
            &&(ParaTb2b.P.MURSg_RN == IAc_FLwR)             // by fan low speed relay
           ) {
    if(MURSRl_S) {                                // if set make up room sign relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
  } else if(PWaSRl_E                                        // if enable please wait sign relay
            &&(ParaTb2b.P.PWaSg_RN == IAc_FLwR)             // by fan low speed relay
           ) {
    if(PWaSRl_S) {                                // if set please wait sign relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
#endif
#if(Us_CbnSw)
  } else if(Cbn1Sw_E                                        // if enable cabinet1 switch
            &&(ParaTb2b.P.Cbnt1_RN == IAc_FLwR)             // by fan low speed relay
           ) {
    if(Cbn1Rl_S) {                                // if set cabinet1 relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
  } else if(Cbn2Sw_E                                        // if enable cabinet2 switch
            &&(ParaTb2b.P.Cbnt2_RN == IAc_FLwR)             // by fan low speed relay
           ) {
    if(Cbn2Rl_S) {                                // if set cabinet2 relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
  #if(Ep_CbnSw)
  } else if(Cbn3Sw_E                                        // if enable cabinet3 switch
            &&(ParaTb2b.P.Cbnt3_RN == IAc_FLwR)             // by fan low speed relay
           ) {
    if(Cbn3Rl_S) {                                // if set cabinet3 relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
  } else if(Cbn4Sw_E                                        // if enable cabinet4 switch
            &&(ParaTb2b.P.Cbnt4_RN == IAc_FLwR)             // by fan low speed relay
           ) {
    if(Cbn4Rl_S) {                                // if set cabinet4 relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
  } else if(Cbn5Sw_E                                        // if enable cabinet5 switch
            &&(ParaTb2b.P.Cbnt5_RN == IAc_FLwR)             // by fan low speed relay
           ) {
    if(Cbn5Rl_S) {                                // if set cabinet5 relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
  #endif
#endif
#if(Us_Curtn)
  } else if(Curtn1_E                                        // if enable curtain1 open relay
            &&(ParaTb2b.P.Cut1O_RN == IAc_FLwR)             // by fan low speed relay
           ) {
    if(Ct1ORl_S) {                                // if set curtain1 open relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
  } else if(Curtn1_E                                        // if enable curtain1 close relay
            &&(ParaTb2b.P.Cut1C_RN == IAc_FLwR)             // by fan low speed relay
           ) {
    if(Ct1CRl_S) {                                // if set curtain1 close relay status flag
      On_FLwRl();                                 // on fan low speed relay
    } else Of_FLwRl();                            // off fan low speed relay
    //------------------------------------------------------
#endif
  } else Of_FLwRl();                                        // off fan low speed relay
  //-----------------------------------------------------------------------
}
//==============================================================================
// air condition control (Air_Ctrl)
void controlAir(void) {
  //---------------------------------------------------
  G_CtDfTp();                                          // get control different temperature
  ValRl_Ct();                                          // valve relay control
  FanRl_Ct();                                          // fan relay control
  //---------------------------------------------------
#if(U_AcIRTr||U_IRCBsO||(U_MdVaPI&&(RCUM_PCB == iS1K6P_M)))
  // check final air-con status is changed
  Tmp_Dat1.uint8 = ParaTb01.P.Arcn_Mod;                // mode (3-bit: b2-0)
  if(ParaTb01.P.Fan_Sped < Fan_Auto) {
    Tmp_Dat1.uint8 |= (ParaTb01.P.Fan_Sped<<3);        // air-con fan speed (2-bit: b4-3)
  } else {
    Tmp_Dat1.uint8 |= (FnAuSp_S<<3);                   // fan auto speed state (2-bit: b4-3)
  }
  if(AcBDO0_F) Tmp_Dat1.bit.b7 = 1;                    // air-con balcony door open shut off mode status flag (1-bit: b7)
  if(AcNMt0_F) Tmp_Dat1.bit.b6 = 1;                    // air-con no-motion shut off mode status flag (1-bit: b6)
  //if(RlValv_S) Tmp_Dat1.bit.b5 = 1;                    // valve (1-bit: b5)
  Tmp_Dat2.uint8 = Ct_StTmp;                           // temperature (7-bit: b6-0)
  if(FnArcn_S) Tmp_Dat2.bit.b7 = 1;                    // power (1-bit: b7)
  //------------------------------------
  if((Tmp_Dat1.uint8 != Pv_FAcSt[0])                   // if data changed
     ||(Tmp_Dat2.uint8 != Pv_FAcSt[1])
    ) {
    Pv_FAcSt[0] = Tmp_Dat1.uint8;                      // save new data
    Pv_FAcSt[1] = Tmp_Dat2.uint8;
  #if(U_MdVaPI&&(RCUM_PCB == iS1K6P_M))
    MVl_COUC = 0;                                      // reset control output update counter (s)
  #endif
  #if(U_AcIRTr||U_IRCBsO)
    if(!Hrdw_Tst) {                                    // if not during hardware testing
      IRmC_TRq = 1;                                    // set infrared remote code transmitting request flag
    }
  #endif
  }
  //---------------------------------------------------
#endif
}
//==============================================================================
// include temporary relay circuit data to general relay circuit
void Ic_TR2GR(void) {
  Genr_RlC.uintAl |= Tmp_RlCc.uintAl;        // or general relay circuit with temporary relay circuit
}
//==============================================================================
// on temporary relay circuit data of relay circuit data
void TRlC1_RD(void) {
#if(U_ShrBus&&U_ShrCct)
  MOISr_TR();                                          // mask out inactive sharing relay circuit from temporary relay circuit data
#endif
  ParaTb01.P.RlCc_Dat.uintAl |= Tmp_RlCc.uintAl;       // inclusive or temporary relay circuit data to relay circuit data
}
//---------------------------------------------------------------
// off temporary relay circuit data of relay circuit data
void TRlC0_RD(void) {
#if(U_ShrBus&&U_ShrCct)
  MOISr_TR();                                          // mask out inactive sharing relay circuit from temporary relay circuit data
#endif
  ParaTb01.P.RlCc_Dat.uintAl &= ~Tmp_RlCc.uintAl;      // mask out temporary relay circuit data from relay circuit data
}
//==============================================================================
// convert relay circuit no. to bit position
unsigned char RlC_2BtP(unsigned char CctNo) {
  switch(CctNo) {                  // check circuit no.
    case Rlc001_I: return(0);      // and return bit position
    case Rlc002_I: return(1);
    case Rlc003_I: return(2);
    case Rlc004_I: return(3);
    case Rlc005_I: return(4);
    case Rlc006_I: return(5);
    case Rlc007_I: return(6);
    case Rlc008_I: return(7);
    case Rlc009_I: return(8);
    case Rlc010_I: return(9);
    case Rlc011_I: return(10);
    case Rlc012_I: return(11);
    case Rlc013_I: return(12);
    case Rlc014_I: return(13);
    case Rlc015_I: return(14);
    case Rlc016_I: return(15);
    case RlcA01_I: return(16);
    case RlcA02_I: return(17);
    case RlcA03_I: return(18);
    case RlcA04_I: return(19);
    case RlcA05_I: return(20);
    case RlcA06_I: return(21);
    case RlcA07_I: return(22);
    case RlcA08_I: return(23);
    case RlcA09_I: return(24);
    case RlcA10_I: return(25);
    case RlcA11_I: return(26);
    case RlcA12_I: return(27);
    case RlcA13_I: return(28);
    case RlcA14_I: return(29);
    case RlcA15_I: return(30);
    case RlcA16_I: return(31);
    default: return(0);
  }
}
//==============================================================================
// convert relay circuit no. to temporary relay circuit
void CRN2_TRC(unsigned char CctNo) {
  if(CctNo <= RlCct_ID) {                    // check number of relay circuit
    Tmp_RlCc.uintAl = 1;                     // set temporary relay circuit data
    CctNo = RlC_2BtP(CctNo);                 // get bit position of no.
    while(CctNo) {
      Tmp_RlCc.uintAl <<= 1;                 // shift bit to target position
      CctNo--;
    }
  } else Tmp_RlCc.uintAl = 0;                // clear temporary relay circuit data
}
//==============================================================================
// on external air-con relay
void On_EAcnR(void) {
  ExAcRl_S = 1;                                   // set external air-con relay status flag
  CRN2_TRC(ParaTb2b.P.ExAcn_RN);                  // convert external air-con relay circuit no. to temporary relay circuit
  TRlC1_RD();                                     // on relay circuit
}
//---------------------------------------------------------------
// off external air-con relay
void Of_EAcnR(void) {
  ExAcRl_S = 0;                                   // clear external air-con relay status flag
  CRN2_TRC(ParaTb2b.P.ExAcn_RN);                  // convert external air-con relay circuit no. to temporary relay circuit
  TRlC0_RD();                                     // off relay circuit
}
//==============================================================================
// on external air-con2 relay
void On_EAc2R(void) {
  EAc2Rl_S = 1;                                   // set external air-con2 relay status flag
  CRN2_TRC(ParaTb2b.P.ExAc2_RN);                  // convert external air-con2 relay circuit no. to temporary relay circuit
  TRlC1_RD();                                     // on relay circuit
}
//---------------------------------------------------------------
// off external air-con2 relay
void Of_EAc2R(void) {
  EAc2Rl_S = 0;                                   // clear external air-con2 relay status flag
  CRN2_TRC(ParaTb2b.P.ExAc2_RN);                  // convert external air-con2 relay circuit no. to temporary relay circuit
  TRlC0_RD();                                     // off relay circuit
}
//==============================================================================
// on external air-con3 relay
void On_EAc3R(void) {
  EAc3Rl_S = 1;                                   // set external air-con3 relay status flag
  CRN2_TRC(ParaTb2b.P.ExAc3_RN);                  // convert external air-con3 relay circuit no. to temporary relay circuit
  TRlC1_RD();                                     // on relay circuit
}
//---------------------------------------------------------------
// off external air-con3 relay
void Of_EAc3R(void) {
  EAc3Rl_S = 0;                                   // clear external air-con3 relay status flag
  CRN2_TRC(ParaTb2b.P.ExAc3_RN);                  // convert external air-con3 relay circuit no. to temporary relay circuit
  TRlC0_RD();                                     // off relay circuit
}
//==============================================================================
// on external air-con4 relay
void On_EAc4R(void) {
  EAc4Rl_S = 1;                                   // set external air-con4 relay status flag
  CRN2_TRC(ParaTb2b.P.ExAc4_RN);                  // convert external air-con4 relay circuit no. to temporary relay circuit
  TRlC1_RD();                                     // on relay circuit
}
//---------------------------------------------------------------
// off external air-con4 relay
void Of_EAc4R(void) {
  EAc4Rl_S = 0;                                   // clear external air-con4 relay status flag
  CRN2_TRC(ParaTb2b.P.ExAc4_RN);                  // convert external air-con4 relay circuit no. to temporary relay circuit
  TRlC0_RD();    
}
//==============================================================================
// external air-con relay control
void EAcnR_Ct(void) {
  if(AcnMdu_E) {                   // if set air-con module enable flag
    //-----------------------------
    if(EAcnRl_E) {                 // if enable external air-con relay
      if(FnArcn_S                  // if air-con is on
         &&(!AcBDO0_F)             //    and clear air-con balcony door open shut off mode status flag
         &&(!AcNMt0_F)             //    and clear air-con no-motion shut off mode status flag
        ) On_EAcnR();              // on external air-con relay
      else Of_EAcnR();             // off external air-con relay
    }
    //-----------------------------
    if(EAc2Rl_E) {                 // if set external air-con2 relay enable flag
      if(FnArcn_S                  // if air-con is on
         &&(!A2BDO0_F)             //    and clear air-con2 balcony door open shut off mode status flag
         &&(!A2NMt0_F)             //    and clear air-con2 no-motion shut off mode status flag
        ) On_EAc2R();              // on external air-con2 relay
      else Of_EAc2R();             // off external air-con2 relay
    }
    //-----------------------------
    if(EAc3Rl_E) {                 // if set external air-con3 relay enable flag
      if(FnArcn_S                  // if air-con is on
         &&(!A3BDO0_F)             //    and clear air-con3 balcony door open shut off mode status flag
        ) On_EAc3R();              // on external air-con3 relay
      else Of_EAc3R();             // off external air-con3 relay
    }
    //-----------------------------
    if(EAc4Rl_E) {                 // if set external air-con4 relay enable flag
      if(FnArcn_S                  // if air-con is on
         &&(!A4BDO0_F)             //    and clear air-con4 balcony door open shut off mode status flag
        ) On_EAc4R();              // on external air-con4 relay
      else Of_EAc4R();             // off external air-con4 relay
    }
    //-----------------------------
#if(U_VRV2CR)
    if(VRV2CR_E) {                           // if set VRV 2-dry contact relay enable flag
      if(FnArcn_S                            // if air-con is on
         &&(!AcBDO0_F)                       //    and clear air-con balcony door open shut off mode status flag
         &&(!AcNMt0_F)                       //    and clear air-con no-motion shut off mode status flag
        ) {
        if(SbAcPw_S) {                       // if set standby air-con power status flag
          VRVDC_St = VCS_PeCl;               // VRV dry contact state = pre-cool state
          VRVC_ChD = 0;                      // clear VRV dry contact changing delay
        } else {
          if(VRVDC_St != VCS_UsCt) {
            VRVDC_St = VCS_UsCt;             // VRV dry contact state = user control state
  #if(ExAc_Mdl == MtSuBS_M)
            VRVC_ChD = 10;                   // restart VRV dry contact changing delay (10 x20ms = 200ms)
  #elif(ExAc_Mdl == ToSBCr_M)
            VRVC_ChD = 0;                    // clear VRV dry contact changing delay
  #endif
          }
        }
      } else {
        if(VRVDC_St != VCS__Off) {
          VRVDC_St = VCS__Off;               // VRV dry contact state = off state
  #if(ExAc_Mdl == MtSuBS_M)
          VRVC_ChD = 0;                      // clear VRV dry contact changing delay
  #elif(ExAc_Mdl == ToSBCr_M)
          VRVC_ChD = 1500;                   // restart VRV dry contact changing delay (1500 x20ms = 30000ms)
  #endif
        }
      }
    }
#endif
    //-----------------------------
  }
}
//==============================================================================
// get all general relay circuit
void G_GenRCc(void) {
  Genr_RlC.uintAl = ParaTb04.P.OtLt_RlC.uintAl;        // include outlet relay to general relay circuit
  //-------------------------------
  if(EAcnRl_E) {                                       // if enable external air-con relay
    CRN2_TRC(ParaTb2b.P.ExAcn_RN);                     // convert external air-con relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  //-------------------------------
  if(EAc2Rl_E) {                                       // if set external air-con2 relay enable flag
    CRN2_TRC(ParaTb2b.P.ExAc2_RN);                     // convert external air-con2 relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  //-------------------------------
  if(EAc3Rl_E) {                                       // if set external air-con3 relay enable flag
    CRN2_TRC(ParaTb2b.P.ExAc3_RN);                     // convert external air-con3 relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  //-------------------------------
  if(EAc4Rl_E) {                                       // if set external air-con4 relay enable flag
    CRN2_TRC(ParaTb2b.P.ExAc4_RN);                     // convert external air-con4 relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  //-------------------------------
#if(U_CSpkRl)
  if(CSpkRl_E) {                                       // if enable chime speaker relay
    CRN2_TRC(ParaTb2b.P.CmSpk_RN);                     // convert chime speaker relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  //-------------------------------
#endif
#if(U_SevSRl)
  if(DNDSRl_E) {                                       // if enable do not disturb sign relay
    CRN2_TRC(ParaTb2b.P.DNDSg_RN);                     // convert do not disturb sign relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  //-------------------------------
  if(MURSRl_E) {                                       // if enable make up room sign relay
    CRN2_TRC(ParaTb2b.P.MURSg_RN);                     // convert make up room sign relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  //-------------------------------
  if(PWaSRl_E) {                                       // if enable please wait sign relay
    CRN2_TRC(ParaTb2b.P.PWaSg_RN);                     // convert please wait sign relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  //-------------------------------
#endif
#if(Us_CbnSw)
  if(Cbn1Sw_E) {                                       // if enable cabinet1 switch
    CRN2_TRC(ParaTb2b.P.Cbnt1_RN);                     // convert cabinet relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  if(Cbn2Sw_E) {                                       // if enable cabinet2 switch
    CRN2_TRC(ParaTb2b.P.Cbnt2_RN);                     // convert cabinet relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  #if(Ep_CbnSw)
  if(Cbn3Sw_E) {                                       // if enable cabinet3 switch
    CRN2_TRC(ParaTb2b.P.Cbnt3_RN);                     // convert cabinet relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  if(Cbn4Sw_E) {                                       // if enable cabinet4 switch
    CRN2_TRC(ParaTb2b.P.Cbnt4_RN);                     // convert cabinet relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  if(Cbn5Sw_E) {                                       // if enable cabinet5 switch
    CRN2_TRC(ParaTb2b.P.Cbnt5_RN);                     // convert cabinet relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  #endif
  //-------------------------------
#endif
#if(Us_Curtn)
  if(Curtn1_E) {                                       // if enable curtain1
    CRN2_TRC(ParaTb2b.P.Cut1O_RN);                     // convert curtain open relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
    CRN2_TRC(ParaTb2b.P.Cut1C_RN);                     // convert curtain close relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  //-------------------------------
#endif
#if(U_485NwB&&U_CkInRl)
  Genr_RlC.uintAl |= ParaTb05.P.CkIn_RlC.uintAl;       // include check in relay to general relay circuit
  //-------------------------------
#endif
#if(U_GldDCt)
  if(!AcnMdu_E) {                                      // if clear air-con module enable flag
    CRN2_TRC(GM1OC_RN);                                // convert glydea motor1 open dry contact relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
    CRN2_TRC(GM1CC_RN);                                // convert glydea motor1 close dry contact relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
    CRN2_TRC(GM1SC_RN);                                // convert glydea motor1 stop dry contact relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
    CRN2_TRC(GM2OC_RN);                                // convert glydea motor2 open dry contact relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
    CRN2_TRC(GM2CC_RN);                                // convert glydea motor2 close dry contact relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
    CRN2_TRC(GM2SC_RN);                                // convert glydea motor2 stop dry contact relay circuit no. to temporary relay circuit
    Ic_TR2GR();                                        // include to general relay circuit
  }
  //-------------------------------
#endif
}
//==============================================================================
// emergency lighting control
void EmrLg_Ct(void) {
  if(EmrLig_E) {                   // if enable emergency lamp
    if(NmLn_Otg) {                 // if normal line outage
      // during emergency line supplied by generator
      Tmp_RlCc.uintAl = ParaTb05.P.Emer_LpC.uintAl;              // get available circuit
     // MOILp_TR();                                                // mask out inactive lamp relay from temporary relay circuit data
      //------------------------------------------
      if(SOn_EmLg) {               // if set switch on emergency light flag
        SOn_EmLg = 0;              // clear switch on emergency light flag
        FOn_EmLg = 1;              // set force on emergency light flag
        TRlC1_RD();                                              // force turn on emergency lamp
#if(U_DimBox)
        Dimm_Cmm.ChnSt.uintAl |= ParaTb05.P.Emer_DmC.uintAl;     // force turn on emergency dimmer
  #if(U_DmB2Ch&&U_DmOwPc)
        G_DmFSdC(&ParaTb05.P.Emer_DmC);                          // get dimmer force send command flag
  #endif
        Sd2_DmBx();                                              // send data to dimmer box
#endif
#if(U_ShrBus&&U_ShrCct)
        SCcSTR_1();                                              // set sharing circuit status transmitting request flag
#endif
      }
      //------------------------------------------
      if(FOn_EmLg) {               // if during force on emergency lamp
        if((!Key_Stay)                                                                                        // if clear key stay flag
           ||((ParaTb01.P.RlCc_Dat.uintAl&Tmp_RlCc.uintAl) != Tmp_RlCc.uintAl)                                //    or emergency lamp status is changed
#if(U_DimBox)
           ||((ParaTb01.P.Dimm_Sta.ChnSt.uintAl&ParaTb05.P.Emer_DmC.uintAl) != ParaTb05.P.Emer_DmC.uintAl)    //    or emergency dimmer channel status is changed
#endif
          ) {
          FOn_EmLg = 0;            // clear force on emergency light flag
        }
      }
      //------------------------------------------
    }
  }
}
//==============================================================================
// on exhaust fan relay
void On_EhFnR(void) {
#if(U_ExhFan)
  CRN2_TRC(ParaTb05.P.EhFan_RN);                  // convert exhaust fan relay circuit no. to temporary relay circuit
  MOILp_TR();                                     // mask out inactive lamp relay from temporary relay circuit data
  TRlC1_RD();                                     // on relay circuit
#endif
}
//---------------------------------------------------------------
// off exhaust fan relay
void Of_EhFnR(void) {
#if(U_ExhFan)
  CRN2_TRC(ParaTb05.P.EhFan_RN);                  // convert exhaust fan relay circuit no. to temporary relay circuit
  MOILp_TR();                                     // mask out inactive lamp relay from temporary relay circuit data
  TRlC0_RD();                                     // off relay circuit
#endif
}
//==============================================================================
// exhaust fan control
void EhFnR_Ct(void) {
#if(U_ExhFan)
  RlyCctGp rlcg1,rlcg2;
  DimChnGp dmcg1,dmcg2;
  if(ExhFan_E) {                                  // if enable exhaust fan
    //--------------------------------------------
    Tmp_RlCc.uintAl = ParaTb05.P.Bath_LpC.uintAl;                               // get available circuit of bathroom light
    MOILp_TR();                                                                 // mask out inactive lamp relay from temporary relay circuit data
  #if(U_DimBox)
    Tmp_DmDt.ChnSt.uintAl = ParaTb05.P.Bath_DmC.uintAl;                         // get available channel of bathroom light
    #if(U_ShrBus&&U_ShrCct)
    MOISr_TD();                                                                 // mask out inactive sharing dimmer channel from temporary dimmer channel
    #endif
  #endif
    if(Tmp_RlCc.uintAl                                                          // check bathroom light usage
  #if(U_DimBox)
        ||Tmp_DmDt.ChnSt.uintAl
  #endif
      ) {
      rlcg1.uintAl = ParaTb01.P.RlCc_Dat.uintAl&Tmp_RlCc.uintAl;                // check bathroom light member
      rlcg2.uintAl = rlcg1.uintAl^Bath_LCM.uintAl;                              // get changing from previous memory
  #if(U_DimBox)
      dmcg1.uintAl = ParaTb01.P.Dimm_Sta.ChnSt.uintAl&Tmp_DmDt.ChnSt.uintAl;
      dmcg2.uintAl = dmcg1.uintAl^Bath_DCM.uintAl;
  #endif
      if(rlcg2.uintAl                                                           // if some changing
  #if(U_DimBox)
         ||dmcg2.uintAl
  #endif
        ) {
        Bath_LCM.uintAl = rlcg1.uintAl;                                         // save new bathroom light memory
  #if(U_DimBox)
        Bath_DCM.uintAl = dmcg1.uintAl;
  #endif
        if(Key_Stay) {                                                          // if set key stay flag
          if((!rlcg1.uintAl)                                                    // if all bathroom light are off
  #if(U_DimBox)
             &&(!dmcg1.uintAl)
  #endif
            ) {
            Of_EhFnR();                                                         // off exhaust fan
            Sav_Mm01();                                                         // save to memory #1
          } else {
            rlcg2.uintAl &= rlcg1.uintAl;                                       // get only turn on changing
  #if(U_DimBox)
            dmcg2.uintAl &= dmcg1.uintAl;
  #endif
            if(rlcg2.uintAl                                                     // if some turn on changing
  #if(U_DimBox)
               ||dmcg2.uintAl
  #endif
              ) {
              On_EhFnR();                                                       // on exhaust fan
              Sav_Mm01();                                                       // save to memory #1
            }
          }
        }
      }
    }
    //--------------------------------------------
    if(!Key_Stay) {                                                             // if clear key stay flag
      // temperature control on duty cycle
      if(ParaTb05.P.EFn_TCDu&&(KyAw_PMn < ParaTb05.P.EFn_TCDu)) {               // check key away period timer (m) with exhaust fan temperature control duty cycle
        if(!RmTpS_Er) {                                                         // if clear room temperature sensor error flag
          if(Room_Tmp >= ParaTb05.P.EFn_SrTp) {                                 // check room temperature >= exhaust fan start temperature
            On_EhFnR();                                                         // on exhaust fan
          } else if(Room_Tmp <= ParaTb05.P.EFn_SpTp) {                          // check room temperature <= exhaust fan stop temperature
            Of_EhFnR();                                                         // off exhaust fan
          }
        }
      }
    }
    //--------------------------------------------
  }
#endif
}
//==============================================================================
// relay circuit control (RlCc_Ctl)
void controlRelayCircuit(void) {
  
   //---------------------------------------------------------------------------
   //chanatip -> for debug hidden code only, not real code
  EmrLg_Ct();                                          // emergency lighting control
  
  
  
  
  //---------------------------------------------------------------------------
  // process air-con status
  FnArcn_S = AirCon_S;                                 // copy air-con status to final air-con status
  if(SbAcS0_S                                          // if set standby air-con shut off status flag
#if(U_PwOtDt)
     ||(AcPwFS_E                                       // or if set air-con power fail saving enable flag
        &&NmLn_Otg                                     // and set normal line outage flag
       )
#endif
    ) {
    FnArcn_S = 0;                                      // off air-con
  }
  //---------------------------------------------------------------------------
  G_GenRCc();                                          // get all general relay circuit
  //---------------------------------------------------------------------------
#if(U_PwOtDt)
  EmrLg_Ct();                                          // emergency lighting control
#endif
#if(U_ExhFan)
  EhFnR_Ct();                                          // exhaust fan control
#endif
  //---------------------------------------------------------------------------
  EAcnR_Ct();                                          // external air-con relay control
#if(U_CSpkRl)
  // chime speaker relay control
  if(CSpkRl_E) {                                       // if enable chime speaker relay
    if(CmSpR_1T) On_CSpkR();                           // check chime speaker relay on timer is not expired
    else Of_CSpkR();                                   // off chime speaker relay
  }
#endif
#if(U_SevSRl)
  SevSR_Ct();                                          // service sign relay control
#endif
#if(Us_CbnSw)
  CbntR_Ct();                                          // cabinet relay control
#endif
#if(Us_Curtn)
  CutnR_Ct();                                          // curtain control
#endif
#if(U_GldDCt)
  GdMDR_Ct();                                          // glydea motor dry contact relay control
#endif
  //---------------------------------------------------------------------------
  FnRlC_St.uintAl = ParaTb01.P.RlCc_Dat.uintAl;        // get final relay circuit status
  //---------------------------------------------------------------------------
#if(U_PwOtDt)
  PwFSR_Ct();                                          // power fail saving relay circuit control
  //---------------------------------------------------------------------------
#endif
   
  EpO_Data.bit.Relay01 = FnRlC_St.bit.Rc001;                       // rc001
  EpO_Data.bit.Relay02 = FnRlC_St.bit.Rc002;                       // rc002
  EpO_Data.bit.Relay03 = FnRlC_St.bit.Rc003;                       // rc003
  EpO_Data.bit.Relay04 = FnRlC_St.bit.Rc004;                       // rc004
  EpO_Data.bit.Relay05 = FnRlC_St.bit.Rc005;                       // rc005
  EpO_Data.bit.Relay06 = FnRlC_St.bit.Rc006;                       // rc006
  EpO_Data.bit.Relay07 = FnRlC_St.bit.Rc007;                       // rc007
  EpO_Data.bit.Relay08 = FnRlC_St.bit.Rc008;                       // rc008
#if(U_2WSwRl)
  //TWSRl_Ct();                                          // 2-way switch relay control
#else
  EpO_Data.bit.Relay09 = FnRlC_St.bit.Rc009;                       // rc009
  EpO_Data.bit.Relay10 = FnRlC_St.bit.Rc010;                       // rc0010
#endif
  EpO_Data.bit.Relay11 = FnRlC_St.bit.Rc011;                       // rc011
  EpO_Data.bit.Relay12 = FnRlC_St.bit.Rc012;                       // rc012
  EpO_Data.bit.Relay13 = FnRlC_St.bit.Rc013;                       // rc013
  EpO_Data.bit.Relay14 = FnRlC_St.bit.Rc014;                       // rc014
  EpO_Data.bit.Relay22 = FnRlC_St.bit.Rc015;                       // rc015
  EpO_Data.bit.Relay21 = FnRlC_St.bit.Rc016;                       // rc016 

  //---------------------------------------------------------------------------
#if(U_GldDCt)
  if(!AcnMdu_E) {                                      // if clear air-con module enable flag
    EpO_Data.bit.RelayFanHi = FnRlC_St.bit.RcA01;                     // rcA01
    EpO_Data.bit.RelayFanMed = FnRlC_St.bit.RcA02;                     // rcA02
    EpO_Data.bit.RelayFanLow = FnRlC_St.bit.RcA03;                     // rcA03
    EpO_Data.Relay18 = FnRlC_St.bit.RcA04;                     // rcA04
  }
  //---------------------------------------------------------------------------
#endif
}

//==============================================================================
// relay control                                                    (every 20ms)
void relay_CT(void) {
  EpO259Gp tmp1;
  controlRelayCircuit();                     //RlCc_Ctl(); // relay circuit control
  controlAir();                               // Air_Ctrl(); // air-con control
  //-----------------------------------------
  if(Hrdw_Tst) {                             // if during hardware testing
    tmp1.uint8.Group3 = 0;                     // assume clear expand output port buffer
    tmp1.uint8.Group2 = 0;
    tmp1.uint8.Group1 = 0;
    if(CmMd_Lv2 == OtHiVT_M) {               // if during output high volt testing
      tmp1.uint8.Group3 = HwTs_Buf[0].uint8;   // followed by testing
      tmp1.uint8.Group2 = HwTs_Buf[1].uint8;
      tmp1.uint8.Group1 = HwTs_Buf[2].uint8;
    }
    tmp1.bit.KeyboxLED = EpO_Data.bit.KeyboxLED;            // key box led
    tmp1.bit.FrontDNTLED = EpO_Data.bit.FrontDNTLED;            // front panel do not disturb led
    tmp1.bit.FrontMURLED = EpO_Data.bit.FrontMURLED;            // front panel make up room led
    tmp1.bit.MasterPanelLED = EpO_Data.bit.MasterPanelLED;            // master panel led
    EpO_PtBf = tmp1;
  } else {
#if(U_2B3FSp)
    tmp1 = EpO_Data;                         // check expand output data
    // convert fan speed relay to 2 bit signal
    if(RlFnHi_S) {                           // if fan high speed
      tmp1.bit.RlFnMd_P = 1;                 // on fan medium speed relay
      tmp1.bit.RlFnLw_P = 1;                 // on fan low speed relay
    }
    EpO_PtBf = tmp1;                         // copy data to expand output port buffer
#else
   // EpO_PtBf = EpO_Data;                     // copy expand output data to expand output port buffer
#endif
  }
}
