/**
   @example  EIS_Test.c
   @brief test file
   Decription:
      M355 based EC Sensor project for Reference Designs

   @version  V0.1
   @author   ADI : 
   @date     Nov 2018

 *****************************************************************************

All files for ADuCM355 provided by ADI, including this file, are
provided  as is without warranty of any kind, either expressed or implied.
The user assumes any and all risk from the use of this code.
It is the responsibility of the person integrating this code into an application
to ensure that the resulting application performs as required and is safe.

**/

#include "M355_Sensor_Meas.h"
#include "ADuCM355_Peri.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "EIS_Test.h"

/*
   Modify macro below to add DC bias for biased sensor(ex. O2 sensor) Impedance measuremnt
   bias voltage is configured by SnsInit,which means (VBIAS-VZERO) will be added to excitaion sinewave as a DC offset.
   1 - EIS for biased gas sensor
   0 - EIS for none-biased gas sensor
*/
#define EIS_DCBIAS_EN   0


volatile uint8_t dftRdy = 0;
extern struct EC_Sensor Sensor;
uint32_t FCW_Val = 0;

//  User can modify frequencies of impedance measurements
ImpResult_N ImpResult[] =
{
 //low frequency measurement takes longer because of big period 
   /*{0.1,{0,0,0,0},0,0},         // 0.1Hz
   {0.15,{0,0,0,0},0,0},        // 0.15Hz
   {0.22,{0,0,0,0},0,0},        // 0.22Hz
   {0.32,{0,0,0,0},0,0},        // 0.32Hz
   {0.4,{0,0,0,0},0,0},         // 0.4Hz
   {0.5,{0,0,0,0},0,0},         // 0.5Hz
   {0.6,{0,0,0,0},0,0},         // 0.6Hz
   {0.8,{0,0,0,0},0,0},         // 0.8Hz
   {1,{0,0,0,0},0,0},
   {1.5,{0,0,0,0},0,0},
   {2.0,{0,0,0,0},0,0},
   {2.5,{0,0,0,0},0,0},
   {3.2,{0,0,0,0},0,0},
   {4,{0,0,0,0},0,0},
   {5,{0,0,0,0},0,0},
   {10,{0,0,0,0},0,0},
   {15,{0,0,0,0},0,0},
   {25,{0,0,0,0},0,0},
   {50,{0,0,0,0},0,0},
   {75,{0,0,0,0},0,0},
   {100,{0,0,0,0},0,0},
   {500,{0,0,0,0},0,0},*/
   {1000,{0,0,0,0},{0},0,0},
   {5000,{0,0,0,0},{0},0,0},
   {10000,{0,0,0,0},{0},0,0},
   {20000,{0,0,0,0},{0},0,0},
   {30000,{0,0,0,0},{0},0,0},
   {40000,{0,0,0,0},{0},0,0},
   {50000,{0,0,0,0},{0},0,0},
   {60000,{0,0,0,0},{0},0,0},
   {70000,{0,0,0,0},{0},0,0},
   {90000,{0,0,0,0},{0},0,0},
   {160000,{0,0,0,0},{0},0,0},
   {200000,{0,0,0,0},{0},0,0},
   //user can add frequency option here
};

// Small array which is used for copying into later for Tx, needs to be same size dimension size as previous array.
ImpResult_M ImpResult_min[] =
{
   // Low frequency measurement takes longer because of a longer period 
   /*{0.1,0,0},           // 0.1Hz
   {0.15,0,0},          // 0.15Hz
   {0.22,0,0},          // 0.22Hz
   {0.32,0,0},          // 0.32Hz
   {0.4,0,0},           // 0.4Hz
   {0.5,0,0},           // 0.5Hz
   {0.6,0,0},           // 0.6Hz
   {0.8,0,0},           // 0.8Hz
   {1,0,0},
   {1.5,0,0},
   {2.0,0,0},
   {2.5,0,0},
   {3.2,0,0},
   {4,0,0},
   {5,0,0},
   {10,0,0},
   {15,0,0},
   {25,0,0},
   {50,0,0},
   {75,0,0},
   {100,0,0},
   {500,0,0},*/
   {1000,0,0},
   {5000,0,0},
   {10000,0,0},
   {20000,0,0},
   {30000,0,0},
   {40000,0,0},
   {50000,0,0},
   {60000,0,0},
   {70000,0,0},
   {90000,0,0},
   {160000,0,0},
   {200000,0,0},
};

uint32_t SIZE_EIS_RESULTS = (sizeof(ImpResult_min)/sizeof(ImpResult_M)) * 12;          // 4 bytes * 3 items, Freq, Mag, Phase = 12bytes per result
uint32_t SIZE_EIS_RESULTS_FULL  = (sizeof(ImpResult)/sizeof(ImpResult_N)) * 60;          // 4 bytes * 15 items, Freq, DFT_imp x 6, DFT_mag x 4, Mag, Phase, RloadMag, Cap = 60 bytes per result
uint8_t  eis_array[sizeof(ImpResult_min)];
uint8_t  eis_array_full[sizeof(ImpResult)];
//==============================================================================
//      Convert EIS results to byte array for tx over i2C
//==============================================================================
void eis_float_to_char_array(void)
{
 uint32_t i;
 
 // Move the results into the smaller array and then copy it to a byte array for Tx
 for(i = 0; i < sizeof(ImpResult_min)/sizeof(ImpResult_M); i++)
 { 
  ImpResult_min[i].freq  = ImpResult[i].freq;
  ImpResult_min[i].Mag   = ImpResult[i].Mag ;
  ImpResult_min[i].Phase = ImpResult[i].Phase;
 }   
 memcpy(eis_array, &ImpResult_min, sizeof(ImpResult_min));
}

//==============================================================================
//      Convert EIS full results to byte array for tx over i2C
//==============================================================================
void eis_full_float_to_char_array(void)
{  
 memcpy(eis_array_full, &ImpResult, sizeof(ImpResult));
}

//==============================================================================
//   @brief uint8_t SnsACInit(uint8_t channel)
//          Initialization for AC test, setup wave generation and switches
//   @param channel :{CHAN0,CHAN1}
//      - 0 or CHAN0, Sensor channel 0
//      - 1 or CHAN1, Sensor channel 1
//==============================================================================
uint8_t SnsACInit(uint8_t channel)
{
   uint32_t ctia;

   AfeAdcIntCfg(BITM_AFE_ADCINTIEN_DFTRDYIEN);
   NVIC_EnableIRQ(AFE_ADC_IRQn);
   
   // ****** Setup exitation loop and TIA ********
   AfeHpTiaPwrUp(true);
   AfeHpTiaCon(HPTIABIAS_1V1);                                                  // Normal power mode, 1.1V biased HP TIA
   AfeSwitchFullCfg(SWITCH_GROUP_T,SWID_T9);
   ctia = BITM_HPTIA_CTIA_16PF|BITM_HPTIA_CTIA_8PF|BITM_HPTIA_CTIA_4PF| \
            BITM_HPTIA_CTIA_2PF|BITM_HPTIA_CTIA_1PF;

   AfeHpTiaSeCfg(HPTIASE_RTIA_1K,ctia, 0);                                      // reduce gain for PGA = 4
   AfeHpTiaDeCfg(CHAN0,HPTIADE_RLOAD_OPEN,HPTIADE_RTIA_OPEN);
   AfeHpTiaDeCfg(CHAN1,HPTIADE_RLOAD_OPEN,HPTIADE_RTIA_OPEN);
   // switch to RCAL, loop exitation before power up
   AfeSwitchDPNT(SWID_DR0_RCAL0,SWID_PR0_RCAL0,SWID_NR1_RCAL1,SWID_TR1_RCAL1|SWID_T9);
   
   /*********Initialize ADC and DFT********/
   //ADC initialization
   AfeAdcFiltCfg(SINC3OSR_5,SINC2OSR_178,LFPBYPEN_NOBYP,ADCSAMPLERATE_800K);    //900Hz as default
   AfeSysCfg(ENUM_AFE_PMBW_LP,ENUM_AFE_PMBW_BW50);
   AfeAdcPgaCfg(GNPGA_4,0);
   AfeAdcChan(MUXSELP_HPTIA_P,MUXSELN_HPTIA_N);
   AfeAdcChopEn(1);                                                             // Enable ADC input buffer chop for LP mode (up to 80kHz)
   
   /********sinewave generation**********/
   AfeHPDacPwrUp(true);
   // DAC attenuator = 1/5, Excitaion Amplifier Gain=1/4,DAC update rate = 320KHz,bandwidth=50KHz
   AfeHPDacCfg(HPDAC_ATTEN_DIV5,HPDAC_RATE_REG,HPDAC_INAMPGAIN_DIV4);
   AfeHPDacSineCfg(SINE_FREQ_REG,0,SINE_OFFSET_REG,SINE_AMPLITUDE_REG);
   AfeHPDacWgType(HPDAC_WGTYPE_SINE);
   return 1;
}

//==============================================================================
//   @brief uint8_t SnsACTest(uint8_t channel)
//          start AC test
//   @param channel :{CHAN0,CHAN1}
//     - 0 or CHAN0, Sensor channel 0
//      - 1 or CHAN1, Sensor channel 1
//   @param pDFTData :{}
//     - pointer to DFT result:6x word
//==============================================================================
uint8_t SnsACTest(uint8_t channel)
{
   uint32_t freqNum = sizeof(ImpResult)/sizeof(ImpResult_N);
   for(uint32_t i=0;i<freqNum;i++)
   {
      SnsACSigChainCfg(ImpResult[i].freq);
      pADI_AFE->AFECON &= ~(BITM_AFE_AFECON_WAVEGENEN|BITM_AFE_AFECON_EXBUFEN|   \
                           BITM_AFE_AFECON_INAMPEN|BITM_AFE_AFECON_TIAEN);                      // disable loop before switching 
      /*********Sensor+Rload AC measurement*************/
                                                                                                // break LP TIA connection
      AfeLpTiaSwitchCfg(channel,SWMODE_AC);                                                     // LP TIA disconnect sensor for AC test
#if EIS_DCBIAS_EN                                                                               // add bias voltage to excitation sinewave
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DACBUFEN;                                             // enable DC buffer for excitation loop
      if(channel>0)
      {
         pADI_AFE->DACDCBUFCON = ENUM_AFE_DACDCBUFCON_CHAN1;                                    //set DC offset using LP DAC1
      }
      else
      {
         pADI_AFE->DACDCBUFCON = ENUM_AFE_DACDCBUFCON_CHAN0;                                    //set DC offset using LP DAC0
      }
#endif
      // Switch to sensor+rload
      // disconnect RTIA to avoid RC filter discharge
      AfeLpTiaCon(CHAN0, Sensor.RLoad, LPTIA_RGAIN_DISCONNECT, SNS_DC_RFILTER_DEFAULT0);
      AfeSwitchDPNT(SWID_D5_CE0,SWID_P5_RE0,SWID_N5_SE0RLOAD,SWID_T5_SE0RLOAD|SWID_T9);

      pADI_AFE->AFECON |= BITM_AFE_AFECON_ADCEN|BITM_AFE_AFECON_SINC2EN|BITM_AFE_AFECON_WAVEGENEN| \
                           BITM_AFE_AFECON_EXBUFEN|BITM_AFE_AFECON_INAMPEN|BITM_AFE_AFECON_TIAEN;
      delay_10us(30);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_ADCEN;
      delay_10us(20);   //200us for switch settling
      // start ADC conversion and DFT
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN|BITM_AFE_AFECON_ADCCONVEN;
      while(!dftRdy)
      {
         PwrCfg(ENUM_PMG_PWRMOD_FLEXI,0,BITM_PMG_SRAMRET_BNK2EN);
      }
      dftRdy = 0;
      ImpResult[i].DFT_result[0] = convertDftToInt(pADI_AFE->DFTREAL);
      ImpResult[i].DFT_result[1] = convertDftToInt(pADI_AFE->DFTIMAG);
      pADI_AFE->AFECON &= ~(BITM_AFE_AFECON_WAVEGENEN|BITM_AFE_AFECON_EXBUFEN|   \
                           BITM_AFE_AFECON_INAMPEN|BITM_AFE_AFECON_TIAEN);                              //disable loop before switching
      
      // this stage measured Rload exclude PCB wires from SE to innner Rload
      AfeSwitchDPNT(SWID_D7_WE0,SWID_P7_WE0,SWID_N5_SE0RLOAD,SWID_T5_SE0RLOAD|SWID_T9);
      // Switch to rload
#ifdef EIS_DCBIAS_EN                                                                                    // add bias voltage to excitation sinewave
      pADI_AFE->AFECON &= ~BITM_AFE_AFECON_DACBUFEN;                                                    // Disable DC buffer for excitation loop
#endif
      pADI_AFE->AFECON |= BITM_AFE_AFECON_ADCEN|BITM_AFE_AFECON_SINC2EN|BITM_AFE_AFECON_WAVEGENEN| \
                           BITM_AFE_AFECON_EXBUFEN|BITM_AFE_AFECON_INAMPEN|BITM_AFE_AFECON_TIAEN;
      delay_10us(30);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_ADCEN;
      delay_10us(20);                                                                                   // 200us for switch settling
                                                                                                        // start ADC conversion and DFT    
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN|BITM_AFE_AFECON_ADCCONVEN;
      while(!dftRdy)
      {
         PwrCfg(ENUM_PMG_PWRMOD_FLEXI,0,BITM_PMG_SRAMRET_BNK2EN);
      }
      dftRdy = 0;
      ImpResult[i].DFT_result[2] = convertDftToInt(pADI_AFE->DFTREAL);
      ImpResult[i].DFT_result[3] = convertDftToInt(pADI_AFE->DFTIMAG   );
      /************RCAL AC measurement***************/
      pADI_AFE->AFECON &= ~(BITM_AFE_AFECON_WAVEGENEN|BITM_AFE_AFECON_EXBUFEN|   \
                           BITM_AFE_AFECON_INAMPEN|BITM_AFE_AFECON_TIAEN);                              // disable loop before switching
      /*switch to RCAL, loop exitation before power up*/
      AfeSwitchDPNT(SWID_DR0_RCAL0,SWID_PR0_RCAL0,SWID_NR1_RCAL1,SWID_TR1_RCAL1|SWID_T9);
      AfeLpTiaSwitchCfg(channel,SWMODE_NORM);                                                           // LP TIA normal working mode
     
      AfeLpTiaCon(CHAN0, Sensor.RLoad, (LPTIA_RGAIN_Type)Sensor.RGainIndex, SNS_DC_RFILTER_DEFAULT0);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_ADCEN|BITM_AFE_AFECON_SINC2EN|BITM_AFE_AFECON_WAVEGENEN| \
                           BITM_AFE_AFECON_EXBUFEN|BITM_AFE_AFECON_INAMPEN|BITM_AFE_AFECON_TIAEN;
      delay_10us(30);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_ADCEN;
      delay_10us(20);                                                                                   // 200us for switch settling
      
      // start ADC conversion and DFT
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN|BITM_AFE_AFECON_ADCCONVEN;
      while(!dftRdy)
      {
         PwrCfg(ENUM_PMG_PWRMOD_FLEXI,0,BITM_PMG_SRAMRET_BNK2EN);
      }
      dftRdy = 0;
      ImpResult[i].DFT_result[4] = convertDftToInt(pADI_AFE->DFTREAL);
      ImpResult[i].DFT_result[5] = convertDftToInt(pADI_AFE->DFTIMAG   );
      /**********recover LP TIA connection to maintain sensor*********/
      AfeSwitchDPNT(SWID_ALLOPEN,SWID_ALLOPEN,SWID_ALLOPEN,SWID_ALLOPEN);
      AfeWaveGenGo(false);
   }

   return 1;
}

//==============================================================================
//   @brief uint8_t SnsMagPhaseCal()
//          calculate magnitude and phase of sensor
//   @param pDFTData : {}
//      - input array which stored 6 DFT data
//   @param RMag :{}
//      - calculated Magnitude of sensor
//   @param RPhase :{}
//     - calulated Phase of sensor
//==============================================================================
uint8_t SnsMagPhaseCal(void)
{
   float Src[8];
   //float Mag[4];
   float Phase[4];
   float Var1,Var2;


   uint32_t testNum = sizeof(ImpResult)/sizeof(ImpResult_N);
   for(uint32_t i=0;i<testNum;i++)
   {
      for (uint8_t ix=0;ix<6;ix++)
      {
         Src[ix] = (float)(ImpResult[i].DFT_result[ix]); // Load DFT Real/Imag results for RCAL, RLOAD, RLOAD+RSENSE into local array for this frequency 
      }
      Src[6] = (float)(Src[2]-Src[0]);                                                  // RLoad(real)-RSensor+load(real)
      Src[7] = (float)(Src[3]-Src[1]);                                                  // RLoad(Imag)-RSensor+load(Imag)
      for (uint8_t ix=0;ix<4;ix++)
      {
         ImpResult[i].DFT_Mag[ix] = Src[ix*2]*Src[ix*2]+Src[ix*2+1]*Src[ix*2+1];
         Phase[ix] = atan2(Src[ix*2+1], Src[ix*2]);  // returns value between -pi to +pi (radians) of ATAN2(IMAG/Real)
         ImpResult[i].DFT_Mag[ix] = sqrt(ImpResult[i].DFT_Mag[ix]);
      }
      
      // Sensor Magnitude in ohms = (RCAL(ohms)*|Mag(RCAL)|*|Mag(RSensor)) 
      //                            --------------------------------------
      //                            |Mag(RSensor+Rload)|*|Mag(RLoad)) 
      Var1 = ImpResult[i].DFT_Mag[2]*ImpResult[i].DFT_Mag[3]*AFE_RCAL;                  // Mag(RCAL)*Mag(RSENSOR)*RCAL
      Var2 = ImpResult[i].DFT_Mag[0]*ImpResult[i].DFT_Mag[1];                           // Mag(RSENSE+LOAD)*Mag(RLOAD)   
      Var1 = Var1/Var2;
      ImpResult[i].Mag = Var1;
      // RSensor+Rload Magnitude in ohms =    (RCAL(ohms)*|Mag(RCAL)|*|Mag(Rload)) 
      //                                       --------------------------------------
      //                                       |Mag(RSensor+Rload)|*|Mag(RSensor+Rload)| 
      Var1 = ImpResult[i].DFT_Mag[2]*ImpResult[i].DFT_Mag[0]*AFE_RCAL;                  // Mag(Rload)*Mag(Rcal)*RCAL
      Var2 = ImpResult[i].DFT_Mag[0]*ImpResult[i].DFT_Mag[0];                           // Mag(RSENSE+LOAD)*Mag(RSENSE+LOAD)   
      Var1 = Var1/Var2;
      ImpResult[i].RloadMag = (Var1 - ImpResult[i].Mag);                                // Magnitude of Rload in ohms
      
      // Phase calculation for sensor
      Var1 = (Phase[2]+Phase[3]-Phase[1]-Phase[0]);                                     // ((RCAL+RSENSE - RLOAD-RLOADSENSE)
      Var1 = Var1*180/PI;                                                               // Convert radians to degrees.     
      if(Var1 > 180)                                                                    // shift phase back to range (-180,180]
      {
         do
         {
            Var1 -= 360;
         }
         while(Var1 > 180);
      }
      else if(Var1 < -180)
      {
         do
         {
            Var1 += 360;
         }
         while(Var1 < -180);
      }
      ImpResult[i].Phase = Var1;
   }
   eis_float_to_char_array();
   eis_full_float_to_char_array();
   return 1;
}

//------------------------------------------------------------------------------------------------------
//   @brief uint8_t SnsACSigChainCfg(float freq)
//         ======== configuration of AC signal chain depends on required excitation frequency.
//   @param freq :{}
//            - excitation AC signal frequency
//   @return 1.
//   @note settings including DAC update rate, ADC update rate and DFT samples can be adjusted for
//   different excitation frequencies to get better performance. As general guidelines,
//       - DAC update rate: make sure at least 4 points per sinewave period. Higher rate comsumes more power.
//       - ADC update rate:  at least follow Nyquist sampling rule.
//       - DFT samples should cover more than 1 sine wave period. more DFT sample reduce variation but take longer time.
//          the configuration can be optimised depending on user's applicationn
//------------------------------------------------------------------------------------------------------
uint8_t SnsACSigChainCfg(float freq)
{
   uint16_t DacCon;
   uint32_t WgFreqReg;

   DacCon = pADI_AFE->HSDACCON;
   DacCon &= (~BITM_AFE_HSDACCON_RATE);                                         // clear rate bits for later setting
   if(freq < 5)   
   {
      ClkDivCfg(1,1);                                                           // digital die to 26MHz 
      AfeHFOsc32M(0);                                                           // AFE oscillator change to 16MHz
      AfeSysClkDiv(AFE_SYSCLKDIV_1);                                            // AFE system clock remain in 16MHz
      DacCon &= 0xFE01;                                                         // Clear DACCON[8:1] bits
      DacCon |= 
        (0x1b<<BITP_AFE_HSDACCON_RATE);                                         // Set DACCLK to recommended setting for LP mode   
      AfeSysCfg(ENUM_AFE_PMBW_LP,ENUM_AFE_PMBW_BW250);
      AfeHpTiaCon(HPTIABIAS_1V1);                                               //Normal power mode, 1.1V biased HP TIA
      
      pADI_AFE->AFECON &= (~(BITM_AFE_AFECON_SINC2EN));                         // Clear the SINC2 filter
      delay_10us(50);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_SINC2EN;
   
      AfeAdcFiltCfg(SINC3OSR_4,
                    SINC2OSR_533,LFPBYPEN_BYP,
                    ADCSAMPLERATE_800K);                                        // Configure ADC update = 800KSPS/4 = 200KSPS SINC3 output. 200K/533, SINC2 O/P = 375 SPS
      // DFT source: supply filter output. 
      pADI_AFE->AFECON &=
        (~(BITM_AFE_AFECON_DFTEN));                                             // Clear DFT enable bit
      delay_10us(50);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN;                                // re-enable DFT
    
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,                                   // DFT input is from SINC2 filter. 8192 * (1/375) = 21.83 seconds to fill
                   DFTNUM_8192,
                   DFTIN_SINC2);
      FCW_Val = (uint32_t)(((freq/16000000)*1073741824)+0.5);
      WgFreqReg = FCW_Val;                     
    }
   else if(freq < 450)                                                          // frequency lower than 450 Hz
   {
      ClkDivCfg(1,1);                                                           // digital die to 26MHz 
      AfeHFOsc32M(0);                                                           // AFE oscillator change to 16MHz
      AfeSysClkDiv(AFE_SYSCLKDIV_1);                                            // AFE system clock remain in 16MHz
     
      AfeSysCfg(ENUM_AFE_PMBW_LP,ENUM_AFE_PMBW_BW250);       
      AfeHpTiaCon(HPTIABIAS_1V1);                                               // Normal power mode, 1.1V biased HP TIA
      DacCon &= 0xFE01;                                                         // Clear DACCON[8:1] bits
      DacCon |= 
        (0x1b<<BITP_AFE_HSDACCON_RATE);                                         // Set DACCLK to recommended setting for LP mode   
      // ADC 900sps update rate to DFT engine
      pADI_AFE->AFECON &= 
        (~(BITM_AFE_AFECON_SINC2EN));                                           // Clear the SINC2 filter to flush its contents
      delay_10us(50);
      pADI_AFE->AFECON |= 
        BITM_AFE_AFECON_SINC2EN;                                                // re-enable SINC2 filter
      AfeAdcFiltCfg(SINC3OSR_4,
                    SINC2OSR_178,LFPBYPEN_BYP,
                    ADCSAMPLERATE_800K);                                        // Configure ADC update = 800KSPS/4 = 200KSPS SINC3 output. 200K/178, SINC2 O/P = 1123 SPS
      pADI_AFE->AFECON &=
        (~(BITM_AFE_AFECON_DFTEN));                                             // Clear DFT enable bit
      delay_10us(50);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN;                                // re-enable DFT
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,                                   // DFT input is from SINC2 filter. 4096 * (1/1123) = 3.64 seconds to fill
                   DFTNUM_4096,
                   DFTIN_SINC2);
      FCW_Val = (uint32_t)(((freq/16000000)*1073741824)+0.5);
      WgFreqReg = FCW_Val; 
   }
   else if(freq < 80000)                                                        // 450Hz < frequency < 80KHz
   {
     ClkDivCfg(1,1);                                                            // digital die to 26MHz 
     AfeHFOsc32M(0);                                                            // AFE oscillator change to 16MHz
     AfeSysClkDiv(AFE_SYSCLKDIV_1);                                             // AFE system clock remain in 16MHz  
     // set middle DAC update rate,16MHz/18=~888KHz update rate,skew the DAC and ADC clocks with respect to each other*/
     AfeSysCfg(ENUM_AFE_PMBW_LP,ENUM_AFE_PMBW_BW250);   
     AfeHpTiaCon(HPTIABIAS_1V1);
     DacCon &= 0xFE01;                                                          // Clear DACCON[8:1] bits
     DacCon |= 
       (0x1b<<BITP_AFE_HSDACCON_RATE);                                          // Set DACCLK to recommended setting for LP mode   
     // ADC 160Ksps update rate to DFT engine
      pADI_AFE->AFECON &= 
        (~(BITM_AFE_AFECON_SINC2EN));                                           // Clear the SINC2 filter to flush its contents
      delay_10us(50);
      pADI_AFE->AFECON |= 
        BITM_AFE_AFECON_SINC2EN;                                                // re-enable SINC2 filter
      AfeAdcFiltCfg(SINC3OSR_4,SINC2OSR_178,
                    LFPBYPEN_BYP,
                    ADCSAMPLERATE_800K);                                        // bypass LPF, 200KHz ADC update rate
      pADI_AFE->AFECON &=
        (~(BITM_AFE_AFECON_DFTEN));                                             // Clear DFT enable bit
      delay_10us(50);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN;                                // re-enable DFT
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,
                   DFTNUM_16384,
                   DFTIN_SINC3);                                                // DFT source: Sinc3 result. 16384 * (1/200000) = 81.92mS
     FCW_Val = (uint32_t)(((freq/16000000)*1073741824)+0.5);
      WgFreqReg = FCW_Val; 
   }
   else                                                                         // 80KHz < frequency < 200KHz
   {
      /*****boost ADC sample rate to 1.6MHz****/
      AfeAdcChopEn(0);                                                          // Disable ADC input buffer chop for HP mode (>80kHz)
      AfeSysCfg(ENUM_AFE_PMBW_HP,ENUM_AFE_PMBW_BW250);                          // set High speed DAC and ADC in high power mode
      AfeHpTiaCon(HPTIABIAS_1V1);
      ClkDivCfg(2,2);
      AfeSysClkDiv(AFE_SYSCLKDIV_2);                                            // AFE system clock remain in 8MHz
      AfeHFOsc32M(BITM_AFE_HPOSCCON_CLK32MHZEN);                                // AFE oscillator change to 32MHz
      ClkDivCfg(1,1);
      // set High DAC update rate,16MHz/9=~1.6MHz update rate,skew the DAC and ADC clocks with respect to each other
      DacCon &= 0xFE01;                                                         // Clear DACCON[8:1] bits
      DacCon |= 
        (0x07<<BITP_AFE_HSDACCON_RATE);                                         // Set DACCLK to recommended setting for HP mode   
      // ADC 400Ksps update rate to DFT engine
      pADI_AFE->AFECON &= 
        (~(BITM_AFE_AFECON_SINC2EN));                                           // Clear the SINC2 filter to flush its contents
      delay_10us(50);
      pADI_AFE->AFECON |= 
        BITM_AFE_AFECON_SINC2EN;                                                // re-enable SINC2 filter
      AfeAdcFiltCfg(SINC3OSR_2,SINC2OSR_178,LFPBYPEN_BYP,ADCSAMPLERATE_1600K);  // 800KHz ADC update rate
      pADI_AFE->AFECON &=
        (~(BITM_AFE_AFECON_DFTEN));                                             // Clear DFT enable bit
      delay_10us(50);
      pADI_AFE->AFECON |= 
        BITM_AFE_AFECON_DFTEN;                                                  // re-enable DFT
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,
                   DFTNUM_16384,DFTIN_SINC3);                                   // DFT source: Sinc3 result 16384 * (1/800000) = 20.48mS
     FCW_Val = (uint32_t)(((freq/16000000)*1073741824)+0.5);
      WgFreqReg = FCW_Val;
   }
   pADI_AFE->HSDACCON = DacCon;
   AfeHPDacSineCfg(WgFreqReg,0,SINE_OFFSET_REG,SINE_AMPLITUDE_REG);             // set new frequency
   return 1;
}

//--------------------------------------------------------------------
//              DFTtoCurrent
//--------------------------------------------------------------------
float DFTtoCurrent (int32_t DFTres)
{
  // Take results from DFT registers and convert to current
  float result;
  float PGAgain;
  float Rtia;
  
  PGAgain = 1;
  Rtia = 5000;
  
  //Code to volts
  result = (float) convertDftToInt(DFTres);
  result = 1.8*result;                                                          // Vref = 1.8V. Changed to 1.82V in later revision.
  result = result/131072;                                                       // 2^17 because DFT is 18b result
  result = (result / PGAgain) / Rtia;                                           // divided by PGA gain, divided by RTia
                 
  return result;
}
