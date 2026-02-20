/**
   @example  M355_Sensor_Meas.c
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

#include <ADUCM355.h>
#include <UrtLib.h>
#include <DioLib.h>
#include "ClkLib.h"
#include <stdio.h>
#include <string.h>
//#include <wdtlib.h>
#include <afewdtlib.h>
#include <AfeAdclib.h>
#include <AfeDaclib.h>
#include <AfeTialib.h>
#include <I2Clib.h>
#include <Pwrlib.h>
#include "M355_Sensor_Meas.h"
#include <math.h>
#include <GPTLib.h>
#include <spilib.h>
#include <rstlib.h>
#include "MVH3000D_Prog_Functions.h"
#include "virtual_i2C_interface.h"			                // Header file for bit-bang I2C communication functions
#include "M355_calibration.h"
#include "capa_Test.h"
#include "EIS_Test.h"
#define RUN_OCV_MEASUREMENT    0x36
#define READ_OCV_RESULT        0x37
void Measure_OCV(void);

// 2D Instruction Array : Row + column
const unsigned int instruction_list[37][2] = 
                {
                {READ_AVG_LSB,              TWO_BYTES},
                {READ_RAW_LSB,              TWO_BYTES},
                {READ_AVG_PPB,              THREE_BYTES},
                {READ_RAW_PPB,              THREE_BYTES},
                {GET_STATUS,                ONE_BYTE},
                {RUN_PULSE_TEST,            THREE_BYTES},
                {READ_PULSE_TEST_RESULTS,   ADC_DATA_LENGTH_MAX},
                {SET_TEMPERATURE,           TWO_BYTES},
                {READ_TEMPERATURE,          TWO_BYTES},
                {SET_HUMIDITY,              TWO_BYTES},
                {READ_HUMIDITY,             TWO_BYTES},
                {SET_MEAS_TIME_MS,          TWO_BYTES},
                {START_MEASUREMENTS,        ONE_BYTE},
                {STOP_MEASUREMENTS,         ONE_BYTE},
                {SET_TIA_GAIN,              ONE_BYTE},
                {READ_TIA_GAIN,             ONE_BYTE},
                {RUN_EIS_TESTS,             TWO_BYTES},
                {READ_EIS_RESULTS,          ONE_BYTE},
                {READ_EIS_RESULTS_FULL,     ONE_BYTE},
                {SET_VBIAS_OFFSET,          TWO_BYTES},
                {READ_VBIAS_OFFSET,         TWO_BYTES},
                {SET_VZERO_VOLTAGE,         TWO_BYTES},
                {READ_VZERO_VOLTAGE,        TWO_BYTES},
                {SET_SENSOR_TYPE,           ONE_BYTE},
                {SET_SENSOR_SENS,           THREE_BYTES},
                {READ_SENSOR_TYPE,          ONE_BYTE},
                {READ_SENSOR_SENS,          THREE_BYTES},
                {SET_RLOAD,                 ONE_BYTE},
                {READ_RLOAD,                ONE_BYTE},
                {READ_MEAS_TIME_MS,         TWO_BYTES},
                {READ_200R_RTIA_CAL_RESULT, ONE_BYTE},
                {READ_CHIPID,               TWO_BYTES},
                {READ_UNIQUE_ID,            SIXTEEN_BYTES},
                {GOTO_SLEEP,                ONE_BYTE},
                {SET_I2C_ADDRESS,           ONE_BYTE},
                {RUN_OCV_MEASUREMENT,       ONE_BYTE},
                {READ_OCV_RESULT,           TWO_BYTES}

};

// Storage Array for response data - used for transmitting results to i2C Master
uint8_t result_array[37][3];  // BYTE array - matches i2C needs
uint16_t OCV_LSB = 0;
float   OCV_mV = 0;


// Array of all gain options on M355
unsigned int RGains[27] = {       0,    200,   1000,   2000,   3000,   4000, 
                               6000,   8000,  10000,  12000,  16000,  20000, 
                              24000,  30000,  32000,  40000,  48000,  64000, 
                              85000,  96000, 100000, 120000, 128000, 160000, 
                             196000, 256000, 512000};

// Settings for all sensors here, Ensure minumum Vbias and Vzero are not lower than 200

// Default Sensor
struct EC_Sensor DefaultSensor  = {.Sensor_Type = DEFAULT, .mV_Bias = 0, .VZero = 1100, .RGainIndex = 1, .RLoad = LPTIA_RLOAD_100, .Sensor_PPB_Offset = 0, 
                                   .Board_LSB_Offset = 0, .Sensitivity_nA = 100e-9}; 

// General Struct - used to store current Sensor settings
struct EC_Sensor Sensor;
struct SENSOR_RESULTS Sensor_Results;
struct MASTER_DATA master_data;
struct COMMAND_DATA command_data;

extern ResultData_t PulseData[];                                                                 // use up to 32K bytes max
extern pulse_PARAM_t PulseCfg;
extern uint32_t sendDataCnt;
extern uint32_t SIZE_EIS_RESULTS;
extern uint8_t eis_array[];
extern uint32_t SIZE_EIS_RESULTS_FULL;
extern uint8_t eis_array_full[];

// Storage structures for MVH temp sensor
struct mvh_vals mvh3000d;
float Temperature = 20.0, Humidity = 40.0;

signed char ucSlaveToMaster[I2C_SLAVE_TX_BYTES_MAX] = {0x11, 0x22, 0x33};
SLAVE_STATUS slave_status = NOT_READY;
unsigned short int sample_time = 500;                                                           // Time sent from the master to tell when to sample
float fVacrossRcal = 0.0;                                                                       // Voltage measured across RCAL resistor - used to generate iCal - calibration current to RTIAs.
float fVref = 1.82;                                                                             // default ADC reference voltage
float kFactor = 1.835/1.820;

// Systick Variable
volatile unsigned char fTimeBaseEvent;           		                                // Timer flag

// Boolean to indicate new i2C / SPI command data
volatile BOOLEAN new_master_data;
unsigned int rtia_results[3];
//----------------------------------------------------------
//      main - program entry point
//----------------------------------------------------------
int main (void)
{
  unsigned int time_ctr = 0;
  
  master_data.master_bytes_qty = I2C_MASTER_RX_BYTES_MAX;
  new_master_data = TRUE;                                                                       // Start the system by populating the first response correctly.
  
  AfeWdtGo(false);                                                                              // WDT on Analog die off 
  fTimeBaseEvent = 1;   
  Sys_Init(DEFAULT, ADI_CFG_I2C_DEFAULT_ADDR);                                                  // Init the System with the default sensor type and i2C address  
  
  // Command data struct init
  command_data.cur_cmd = GET_STATUS;
  command_data.cur_bytes_to_send = ONE_BYTE;
   
  // Master data struct init
  master_data.cmd = GET_STATUS;
  master_data.master_bytes_qty = ONE_BYTE;
  
  // Init the sensor again after the calibrations and run the main loop
  Sensor_Init(Sensor.Sensor_Type);
  slave_status = READY;
  while (1)
  {
   #ifdef EVAL_VERSION
    if(new_master_data)
    {
      Chk_Instruction();                                                                        // Load up the new instruction response or perform requested command
      new_master_data = FALSE;
    }
   #else
    Chk_Instruction();
   #endif
   if(fTimeBaseEvent)
   {     
    if(time_ctr % sample_time == 0)                                                             // sample time task
    {
     Sample_Sensor();
    }     
    if(time_ctr > 1000)
    {
     #ifdef MVH_SENSOR_ENABLED 
      MVH3000DInitiateMeasurement();		                                                // Tell MVH sensor to make a new measurement, collect it next time around the loop             
      Delay_ms(50);  
      MVH3000DReadMeasurement();                                                                // Get measurement data from sensor 
     #endif
     time_ctr = 1;  
    }
    else
    {
     time_ctr++;
    }
    fTimeBaseEvent = 0;                                                                         // clear the scheduler variable
   }
  }                                                                                             // end while(1)
}

//--------------------------------------------------------------------------
//              Open_All_Hp_Switches
//--------------------------------------------------------------------------
void Open_All_Hp_Switches(void)
{
  pADI_AFE->TSWFULLCON = 0;
  pADI_AFE->NSWFULLCON = 0;
  pADI_AFE->PSWFULLCON = 0;
  pADI_AFE->DSWFULLCON = 0;
  pADI_AFE->SWCON      = 0x10000;
  pADI_AFE->HSRTIACON |= 0xF;    
}

//--------------------------------------------------------------------------
// On initialization, this function is called to temporarily close SW1 in the Low Power loop.
// This results in the LPTIA output being shorted to its input.
// For a short duration, the amplifier can handle this
// This greatly speeds up the settling time of the gas sensor.
//--------------------------------------------------------------------------
void ChargeECSensor(void)
{
  pADI_AFE->LPTIASW0 |= 0x2;                                                            // Close SW1 in LP loop to shot LPTIA0 output to inverting input
  delay_10us(400000);                                                                   // delay 4secs
  pADI_AFE->LPTIASW0 &= ~(0x2);                                                         // Open SW1 in LP loop to shot LPTIA0 output to inverting input
}

//------------------------------------------------------------------------
//      Measure board offset with sensor disconnected (have it plugged out)
//------------------------------------------------------------------------
void Meas_Board_Offset(void)
{
  int32_t DC_Result1[SNS_DC_POINTS_DEFAULT];
  int32_t DC_Result1_Avg;
  uint32_t loop = 0;
  int32_t total_lsb = 0;
  
  int32_t val;
  
  val = sizeof(DC_Result1)/sizeof(DC_Result1[0]);
  val -=1;
  
  Sensor_Measure(CHAN0, DC_Result1, SNS_DC_POINTS_DEFAULT);                             // Take ADC samples and return into the array
  
  // Get average of the sample (may be 1 or many, depends on SNS_DC_POINTS value)
  for(loop = 1; loop < sizeof(DC_Result1)/sizeof(DC_Result1[0]); loop++)                // throw away first sample
  {
   DC_Result1[loop] -= 32768;
   total_lsb += DC_Result1[loop];
  }
  DC_Result1_Avg = total_lsb / val;
  Sensor.Board_LSB_Offset = DC_Result1_Avg * -1;
}

//------------------------------------------------------------------------
//      Take a sample(s) and process for average etc..
//------------------------------------------------------------------------
void Sample_Sensor(void)
{
  int32_t DC_Result1[SNS_DC_POINTS_DEFAULT];
  int32_t DC_Result1_Avg;
  uint32_t loop = 0;
  signed int *ptr_lsb = NULL;
  double *ptr_ppb = NULL;
  double total_ppb = 0; 
  int32_t total_lsb = 0;
  float current_meas; 
  int32_t val;
  
  val = sizeof(DC_Result1)/sizeof(DC_Result1[0]);
  val -=1;                                                                              // only going to avg 10 of the 11 samples
  
  Sensor_Measure(CHAN0, DC_Result1, SNS_DC_POINTS_DEFAULT);                             // Take ADC samples and return into the array
  
  // Get average of the sample (may be 1 or many, depends on SNS_DC_POINTS value)
  for(loop = 1; loop < sizeof(DC_Result1)/sizeof(DC_Result1[0]); loop++)                // throw away first sample, get average of next 10
  {
    DC_Result1[loop] -= 32768;
    total_lsb += DC_Result1[loop];
  }
  DC_Result1_Avg = total_lsb / val;
  Sensor_Results.cur_lsb = DC_Result1_Avg + Sensor.Board_LSB_Offset;
  
  current_meas = (Sensor_Results.cur_lsb * LSB_Size_16bit)/Sensor.RGainOhms;            // Result of this is in Amps
  Sensor_Results.cur_ppb = (current_meas / Sensor.Sensitivity_nA) * 1000;               // Get the relative nA/ppm from the sensor structure. Determine relationship to sensor nA/ppm, multiply by 1000 to get ppbs
  Sensor_Results.cur_ppb -= Sensor.Sensor_PPB_Offset;                                   // subtract a neg or pos ppb value to establish sensors true baseline

  // Next get the actual longer term average in terms of ppm and lsb. 
  // Also populate latest raw values                              
  ptr_lsb = &Sensor_Results.lsb_result_array[AVERAGING_ARRAY_SIZE - 1];                 // Pointer to last element of the result array, used to iterate down to get average
  ptr_ppb = &Sensor_Results.ppb_result_array[AVERAGING_ARRAY_SIZE - 1];                 // Pointer to last element of the result array, used to iterate down to get average
  
  // Store the results in the structure array, shift up and out oldest,
  // Insert newest, get the average of the array at the same time
  total_lsb = 0;
  total_ppb = 0;
  for(loop = 1; loop < AVERAGING_ARRAY_SIZE; loop++)
  {
    Sensor_Results.lsb_result_array[AVERAGING_ARRAY_SIZE - loop] = Sensor_Results.lsb_result_array[(AVERAGING_ARRAY_SIZE - loop) - 1];
    Sensor_Results.ppb_result_array[AVERAGING_ARRAY_SIZE - loop] = Sensor_Results.ppb_result_array[(AVERAGING_ARRAY_SIZE - loop) - 1];
    total_lsb = total_lsb + *ptr_lsb;
    total_ppb = total_ppb + *ptr_ppb;
    ptr_lsb--;
    ptr_ppb--;
  }
  
  // Add in the latest to average lsb values
  Sensor_Results.lsb_result_array[0] = Sensor_Results.cur_lsb;
  total_lsb += Sensor_Results.lsb_result_array[0];
  Sensor_Results.avg_lsb = total_lsb / AVERAGING_ARRAY_SIZE;
  
  // Add in the latest to average ppb values
  Sensor_Results.ppb_result_array[0] = Sensor_Results.cur_ppb;
  total_ppb += Sensor_Results.ppb_result_array[0];
  Sensor_Results.avg_ppb = total_ppb / AVERAGING_ARRAY_SIZE;
}

//------------------------------------------------------------------------
//      Check and update if needed the latest instruction running
//------------------------------------------------------------------------
void Chk_Instruction(void)
{
 unsigned long uisens_nA = 0;
 unsigned int reg;
 signed short i_dummy_var;                                                                                      // signed as negative temp and humidity
 signed char c_amplitude;
 unsigned char uc_pulse_time;   
   
 // Find the currently active instruction and prepare the slave response
   switch(command_data.cur_cmd)
     {
     case READ_AVG_LSB:
       result_array[command_data.cmd_index][0] = Sensor_Results.avg_lsb >> 8;
       result_array[command_data.cmd_index][1] = Sensor_Results.avg_lsb & 0xFF;
       break;
     case READ_RAW_LSB:
       result_array[command_data.cmd_index][0] = Sensor_Results.cur_lsb >> 8;
       result_array[command_data.cmd_index][1] = Sensor_Results.cur_lsb & 0xFF; 
       break;
     case READ_AVG_PPB:
       result_array[command_data.cmd_index][0] = (signed int)Sensor_Results.avg_ppb >> 16;
       result_array[command_data.cmd_index][1] = (signed int)Sensor_Results.avg_ppb >> 8;
       result_array[command_data.cmd_index][2] = (signed int)Sensor_Results.avg_ppb & 0xFF;
       break;
     case READ_RAW_PPB: 
       result_array[command_data.cmd_index][0] = (signed int)Sensor_Results.cur_ppb >> 16;
       result_array[command_data.cmd_index][1] = (signed int)Sensor_Results.cur_ppb >> 8;
       result_array[command_data.cmd_index][2] = (signed int)Sensor_Results.cur_ppb & 0xFF;
       break;
     case GET_STATUS:
       result_array[command_data.cmd_index][0] = slave_status;                                                   // Report the current status of device      
       break;
     case SET_MEAS_TIME_MS:
       sample_time = ((master_data.data_byte[0] << 8)
                    |(master_data.data_byte[1] & 0xFF));
       command_data.old_cmd = SET_MEAS_TIME_MS;
       command_data.cur_cmd = GET_STATUS;                                                                       // Dont want this running every time, so switch back to status 
       break;
     case READ_MEAS_TIME_MS:
       result_array[command_data.cmd_index][0] = sample_time >> 8;
       result_array[command_data.cmd_index][1] = sample_time & 0xFF; 
       break;    
     case SET_TIA_GAIN:
       if(master_data.data_byte[0] < sizeof(RGains))
       {
        Sensor.RGainIndex = master_data.data_byte[0];
        Sensor.RGainOhms = RGains[Sensor.RGainIndex];
        AfeLpTiaCon(0, Sensor.RLoad, (LPTIA_RGAIN_Type)Sensor.RGainIndex, SNS_DC_RFILTER_DEFAULT0);             // Program new TIA setting
       }
       command_data.old_cmd = SET_TIA_GAIN;
       command_data.cur_cmd = GET_STATUS;                                                                       // Dont want this running every time, so switch back to status      
       break;
     case RUN_OCV_MEASUREMENT:

       // Stop any AC excitation or waveform
       AfeWaveGenGo(false);
       AfeHPDacPwrUp(false);
       AfeHpTiaPwrUp(false);

       // Stop ADC if running
       AfeAdcGo(ADCIDLE);

       // Allow electrode settling
       Delay_ms(500);

       // Perform OCV measurement
       Measure_OCV();

       command_data.old_cmd = RUN_OCV_MEASUREMENT;
       command_data.cur_cmd = GET_STATUS;
       break;

case READ_OCV_RESULT:
    result_array[command_data.cmd_index][0] = (uint8_t)(OCV_LSB >> 8); 
    result_array[command_data.cmd_index][1] = (uint8_t)(OCV_LSB & 0xFF); 
    break;


     case READ_TIA_GAIN:
       result_array[command_data.cmd_index][0] = Sensor.RGainIndex;
       break;   
     case SET_VBIAS_OFFSET:
       Sensor.mV_Bias = (signed int)((master_data.data_byte[0] << 8)|(master_data.data_byte[1] & 0xFF));
       LPDac_Setup();
       command_data.old_cmd = SET_VBIAS_OFFSET;
       command_data.cur_cmd = GET_STATUS;                                                         // Dont want this running every time, so switch back to status
       break;
     case READ_VBIAS_OFFSET:  
       result_array[command_data.cmd_index][0] = (signed int)Sensor.mV_Bias >> 8;
       result_array[command_data.cmd_index][1] = (signed int)Sensor.mV_Bias & 0xFF;      
       break;
     case SET_SENSOR_TYPE:
       Sensor_Init((SENSOR_TYPE)master_data.data_byte[0]);                                        // Sensor data is loaded into structure, activate settings now
       command_data.old_cmd = SET_SENSOR_TYPE;
       command_data.cur_cmd = GET_STATUS;                                                         // Dont want this running every time, so switch back to status
       break;
     case SET_SENSOR_SENS:
       uisens_nA  = master_data.data_byte[0] << 16;
       uisens_nA |= master_data.data_byte[1] << 8;
       uisens_nA |= master_data.data_byte[2];
       Sensor.Sensitivity_nA = (uisens_nA / SENSOR_SCALE_FACTOR) * 1e-9;                          // Scale the received result back and set to nA
       command_data.old_cmd = SET_SENSOR_SENS;
       command_data.cur_cmd = GET_STATUS;                                                         // Dont want this running every time, so switch back to status
       break;
     case READ_SENSOR_TYPE:
       result_array[command_data.cmd_index][0] = Sensor.Sensor_Type;
       break; 
     case READ_SENSOR_SENS:
       uisens_nA = (unsigned long) round(Sensor.Sensitivity_nA * SENSOR_SCALE_FACTOR  * 1e9);      // remove nA and multiply by scale factor, ensure rounded up or down correctly
       result_array[command_data.cmd_index][0] = (unsigned long) uisens_nA >> 16;
       result_array[command_data.cmd_index][1] = (unsigned long) uisens_nA >> 8;
       result_array[command_data.cmd_index][2] = (unsigned long) uisens_nA & 0xFF;
       break;
     case SET_RLOAD:
       if(master_data.data_byte[0] < 8)                                                           // 8 RL options in LPTIACON0 register
       {
         Sensor.RLoad = (LPTIA_RLOAD_Type)master_data.data_byte[0];
         reg = pADI_AFE->LPTIACON0;                                                               // Read
         reg &= ~(0x1C00);                                                                        // Clear bits 10-12 as these are Rload bits
         reg |= ((Sensor.RLoad << 10));                                                           // Modify
         pADI_AFE->LPTIACON0 = reg;                                                               // Write
       }
       command_data.old_cmd = SET_RLOAD;
       command_data.cur_cmd = GET_STATUS;                                                         // Dont want this running every time, so switch back to status
       break; 
     case READ_RLOAD:
       result_array[command_data.cmd_index][0] = Sensor.RLoad;
       break;
     case RUN_EIS_TESTS:
       i_dummy_var = master_data.data_byte[0];
       if(i_dummy_var == 1)
       {
         NVIC_DisableIRQ(I2C_SLV_EVT_IRQn);                                                       // Disable i2C Slave interrupt, dont want interrupts during EIS tests
         //printf("Impedance Test:\r\n");
         SnsACInit(CHAN0);
         SnsACTest(CHAN0);
         SnsMagPhaseCal();                                                                        // Calculate impedance
         // Power off high power exitation loop if required
         AfeAdcIntCfg(NOINT);                                                                     // Disable all ADC interrupts
         NVIC_DisableIRQ(AFE_ADC_IRQn);
         AfeWaveGenGo(false);
         AfeHPDacPwrUp(false);
         AfeHpTiaPwrUp(false);
         NVIC_EnableIRQ(I2C_SLV_EVT_IRQn);                                                        // Re-Enable i2C Slave interrupt 
         command_data.old_cmd = RUN_EIS_TESTS;
         command_data.cur_cmd = GET_STATUS;                                                       // Dont want this running every time, so switch to status
       }
       break;  
     case SET_TEMPERATURE:  
       i_dummy_var   = master_data.data_byte[0] << 8;
       i_dummy_var  |= master_data.data_byte[1];     
       mvh3000d.TempDegC = i_dummy_var / 100.0;                                                   // Temp is scaled to get rid of decimal points during transmission, divide back
       command_data.old_cmd = SET_TEMPERATURE;
       command_data.cur_cmd = GET_STATUS;                                                         // Dont want this running every time, so switch to status
       break;
     case SET_HUMIDITY:
       i_dummy_var   = master_data.data_byte[0] << 8;
       i_dummy_var  |= master_data.data_byte[1];     
       Humidity = i_dummy_var / 100.0;                                                            // Hum is scaled to get rid of decimal points during transmission, divide back
       command_data.old_cmd = SET_HUMIDITY;
       command_data.cur_cmd = GET_STATUS;                                                         // Dont want this running every time, so switch to status
       break;  
     case GOTO_SLEEP:                                                                             // Goto Hibernate mode, keep sensor biased, ensure wakeup enable on i2C interrupt
       Stop_GPTimer0();                                                                           // Stop 1ms scheduler
       NVIC_EnableIRQ(XINT_EVT1_IRQn);                                                            // Enable Sys_Wake Interrupt in the NVIC
       EnterHibernateMode();                                                                      // Goto Hibernate, Sys_Wake ExtInt1 pin when toggled will wake
       NVIC_DisableIRQ(XINT_EVT1_IRQn);                                                           // Disable Sys_Wake Interrupt in the NVIC
       Setup_GPTimer0();                                                                          // Start 1ms scheduler       
       Clock_Init();                                                                              // Rewrite clk
       command_data.old_cmd = GOTO_SLEEP;
       command_data.cur_cmd = GET_STATUS;                                                         // Dont want this running every time, so switch to status
       break;
     case READ_TEMPERATURE:  
    #ifdef MVH_SENSOR_ENABLED 
       i_dummy_var = (signed int)(mvh3000d.TempDegC  * 100);
    #else
       i_dummy_var = (signed int)(Temperature * 100);
    #endif
       result_array[command_data.cmd_index][0] = i_dummy_var >> 8;
       result_array[command_data.cmd_index][1] = i_dummy_var & 0xFF;
       break;
     case READ_HUMIDITY:
    #ifdef MVH_SENSOR_ENABLED 
       i_dummy_var = (signed int)(mvh3000d.Hum * 100);
    #else
       i_dummy_var = (signed int)(Humidity * 100);
    #endif
       result_array[command_data.cmd_index][0] = i_dummy_var >> 8;
       result_array[command_data.cmd_index][1] = i_dummy_var & 0xFF;      
       break;
     case RUN_PULSE_TEST:
       c_amplitude  = master_data.data_byte[0];
       uc_pulse_time = master_data.data_byte[1];
       PulseTestInit(c_amplitude, uc_pulse_time);
       pADI_AFE->LPTIASW0 = 0x302C;                                                               // Set 
       PulseTestRun();                                                                            // Execute the pulse and store the results
       // Sensor_Init(Sensor.Sensor_Type);                                                           // Return the sensor to previous settings for normal measurements    
       command_data.old_cmd = RUN_PULSE_TEST;
       command_data.cur_cmd = GET_STATUS;                                                         // Dont want this running every time, so switch to status
       break;
     case READ_PULSE_TEST_RESULTS:  
       ucSlaveToMaster[0] = 0x00;                                                                 // 0x00 Indicates start of pulse Test results, need to do this as if load first result here, lower byte is lost later..       
       break;
     case READ_EIS_RESULTS:
       ucSlaveToMaster[0] = 0x00;                                                                 // 0x00 Indicates start of EIS Test results, need to do this as if load first result here, lower byte is lost later..
       break;
     case READ_EIS_RESULTS_FULL:
       ucSlaveToMaster[0] = 0x00;                                                                 // 0x00 Indicates start of EIS Test full results, need to do this as if load first result here, lower byte is lost later..
       break;
     case READ_200R_RTIA_CAL_RESULT:  
       result_array[command_data.cmd_index][0] = PulseCfg.actual_Rtia;                            // Indicates start of pulse Test results
       break;
     case READ_CHIPID:
       result_array[command_data.cmd_index][0] = pADI_AFECON->CHIPID >> 8;                        // Read 16 bit Chipid from AFE. Top 8bits
       result_array[command_data.cmd_index][1] = pADI_AFECON->CHIPID & 0xFF;                      // Read 16 bit Chipid from AFE. bottlom 8bits
       break;
     case READ_UNIQUE_ID:        
       ucSlaveToMaster[0]  = UNIQUE_ID_1 >> 24;
       ucSlaveToMaster[1]  = UNIQUE_ID_1 >> 16;
       ucSlaveToMaster[2]  = UNIQUE_ID_1 >> 8;
       ucSlaveToMaster[3]  = UNIQUE_ID_1 & 0xFF;
       ucSlaveToMaster[4]  = UNIQUE_ID_2 >> 24;
       ucSlaveToMaster[5]  = UNIQUE_ID_2 >> 16;
       ucSlaveToMaster[6]  = UNIQUE_ID_2 >> 8;
       ucSlaveToMaster[7]  = UNIQUE_ID_2 & 0xFF;
       ucSlaveToMaster[8]  = UNIQUE_ID_3 >> 24;
       ucSlaveToMaster[9]  = UNIQUE_ID_3 >> 16;
       ucSlaveToMaster[10] = UNIQUE_ID_3 >> 8;
       ucSlaveToMaster[11] = UNIQUE_ID_3 & 0xFF;
       ucSlaveToMaster[12] = UNIQUE_ID_4 >> 24;
       ucSlaveToMaster[13] = UNIQUE_ID_4 >> 16;
       ucSlaveToMaster[14] = UNIQUE_ID_4 >> 8;
       ucSlaveToMaster[15] = UNIQUE_ID_4 & 0xFF; 
       break;
     case SET_I2C_ADDRESS:
       SetI2CAddr(master_data.data_byte[0]);
       break;
     default:
       break;
     }
   /*
   SpiFifoFlush(pADI_SPI0, BITM_SPI_CTL_TFLUSH, BITM_SPI_CTL_RFLUSH);                           // Flush FIFO, ensure it's empty
   uiTxCnt = 0;
   do{
     if(uiTxCnt < command_data.cur_bytes_to_send)
     {
       SpiTx(pADI_SPI0, ucSlaveToMaster[uiTxCnt]);                                              // Prepare SPI byte for next transfer
     }
     else
     {
       SpiTx(pADI_SPI0, 0xFF);                                                                  // In case more bytes are requested than needed
     }
     uiTxCnt++;
   }while(uiTxCnt < SPI_SLAVE_TX_BYTES_MAX);
   
   if((command_data.cur_cmd == master_data.cmd) || (command_data.cur_cmd == SET_TEMPERATURE))   // Before leaving, ensure no new data has arrived in the meantime, also don't want to set for passive commmands like temperature
   {
    new_master_data = FALSE;                                                                    // Command is processed, clear flag
   }
   */
  #ifdef EVAL_VERSION
   NVIC_DisableIRQ(I2C_SLV_EVT_IRQn);                                                       // Disable i2C Slave interrupt, dont want interrupts
   I2cFifoFlush(pADI_I2C0,SLAVE,ENABLE);                                                        // Enable flush of Slave FIFOs
   I2cFifoFlush(pADI_I2C0,SLAVE,DISABLE);
   if((command_data.cur_cmd == READ_PULSE_TEST_RESULTS) || (command_data.cur_cmd == READ_EIS_RESULTS) || (command_data.cur_cmd == READ_EIS_RESULTS_FULL) ||(command_data.cur_cmd == READ_UNIQUE_ID))
   {
    pADI_I2C0->STX = ucSlaveToMaster[0];
   }
   else
   {
    pADI_I2C0->STX = result_array[command_data.cmd_index][0];                                    // N.B. !! Pre-populate the first i2C slave response byte, i2C int handler fills the others, this way solves clock stretching issues
   }
   NVIC_EnableIRQ(I2C_SLV_EVT_IRQn);                                                       // Disable i2C Slave interrupt, dont want interrupts during EIS tests
  #endif
}

//------------------------------------------------------------------------
//      Sleep Mode
//------------------------------------------------------------------------
void EnterHibernateMode(void)
{
  
  pADI_AFE->TSWFULLCON =0x0000;                                                 // open all T switches
  pADI_AFE->DSWFULLCON =0x0000;                                                 // open all D switches
  pADI_AFE->HSRTIACON=0xF;                                                      // open switch at HPTIA near RTIA
  pADI_AFE->DE1RESCON=0xFF;                                                     // open switch at HPTIA near RTIA05
  pADI_AFE->DE0RESCON=0xFF;                                                     // open switch at HPTIA near RTIA03  
  
  // Add before Hibernate to minimise idd when asleep 
  // i2C Channel re-config
  DioCfgPin(pADI_GPIO0, PIN4, 0);                                               // Config as normal GPIO
  DioCfgPin(pADI_GPIO0, PIN5, 0);                                               // Config as normal GPIO
  
  // SPI channel re-config
  DioCfgPin(pADI_GPIO0, PIN0, 0);                                               // Config as normal GPIO
  DioCfgPin(pADI_GPIO0, PIN1, 0);                                               // Config as normal GPIO
  DioCfgPin(pADI_GPIO0, PIN2, 0);                                               // Config as normal GPIO
  DioCfgPin(pADI_GPIO0, PIN3, 0);                                               // Config as normal GPIO
  
  // virtual i2C channel re-config
  DioIenPin(pADI_GPIO1, PIN4, 0);                                               // Disable as input GPIO1.4
  DioIenPin(pADI_GPIO1, PIN5, 0);                                               // Disable as input GPIO1.5
   
  delay(1000);
  AfePwrCfg(AFE_HIBERNATE);
  delay(1000);                                                                  // Wait for AFE to enter Hibernate mode

  PwrCfg(ENUM_PMG_PWRMOD_HIBERNATE,                                             // Place digital die in Hibernate
         BITM_PMG_PWRMOD_MONVBATN,
         BITM_PMG_SRAMRET_BNK2EN);
  
  // Continue here after Hibernate Mode
  //i_dummy_var = pADI_AFE->LPDACCON0;                                          // Read any AFE die register to wake-up AFE die  
  AfePwrCfg(AFE_ACTIVE);
  Delay_us(100);
  
  // i2C Channel re-config
  DioCfgPin(pADI_GPIO0, PIN4, 1);                                               // Config as SCL
  DioCfgPin(pADI_GPIO0, PIN5, 1);                                               // Config as SDA
  
  //SPI channel re-config
  DioCfgPin(pADI_GPIO0, PIN0, 1);                                               // Config as SCLK
  DioCfgPin(pADI_GPIO0, PIN1, 1);                                               // Config as MOSI
  DioCfgPin(pADI_GPIO0, PIN2, 1);                                               // Config as MISO
  DioCfgPin(pADI_GPIO0, PIN3, 1);                                               // Config as CS
  
  // Virtual i2C channel re-config
  DioIenPin(pADI_GPIO1, PIN4, 0);                                               // Disable as SDA
  DioIenPin(pADI_GPIO1, PIN5, 0);                                               // Disable as SCL
}

//----------------------------------------------------------
//      System Initialise
//----------------------------------------------------------
uint8_t Sys_Init(SENSOR_TYPE Sens_Type, unsigned char i2C_Addr)
{
  unsigned int loop;
  #ifdef MVH_SENSOR_ENABLED   
    unsigned long ID;
    volatile unsigned long temp_res;
    volatile unsigned long hum_res;
  #endif
    
  CALDATLOCK = 0xDE87A5AF;
  EXBUFTRMLP = 0x103F;
  EXBUFTRMHP = 0x203C;
  CALDATLOCK = 0xDE87A5B0;
  
  Clock_Init();
  Uart_Init();
  GPIO_Init();
  
  // MVH needs to be configured within 10ms of POR
  #ifdef MVH_SENSOR_ENABLED                
      enterProgrammingMode();                                                                   // Put the MVH3000D Sensor into programming mode, This command must be executed within 10ms of applying power to the sensor  
      MVH3000D_ResetRegisters();
      ID = MVH3000DReadID();                                                                    // Read the sensor ID 
      ID = ID;
      MVH3000DSetResolution(RH_RES, RES_10);                                                    // Set the resolution of the relative humidity measurements to 14 bits 
      MVH3000DSetResolution(T_RES, RES_10);                                                     // Set the resolution of the temperature measurements to 14 bits  
      exitProgrammingMode();                                                                    // Exit programming mode so relative humidity and tempreature measurements can be made
      Delay_us(100);     
      temp_res = MVH3000DReadRegister(MVH3000D_T_RES_READ);
      hum_res = MVH3000DReadRegister(MVH3000D_RH_RES_READ);        
      MVH3000DInitiateMeasurement();		                                                // Tell MVH sensor to make a measurement
  #endif
  #ifndef MVH_SENSOR_ENABLED     
      mvh3000d.TempDegC = 21;                                                                   // Just put a value in
  #endif
  
  I2C_Init(i2C_Addr);                                                                           // Setup i2C with address
  SPI_Init();
  Init_NVIC();
  SwitchSetup();                                                                                // Configure the T, P, N and D switches for HS DAC calibration
  InitVBIAS0();                                                                                 // Init LPDAC to turn on VZERO0 output - sets common mode voltage of HSDAC
  InitAfeDie(); 
  Setup_GPTimer0();                                                                             // For 1ms Tick scheduler
  Sensor_Init(Sens_Type);                                                                       // Sensor data is loaded into structure, activate settings now
  
  //
  SwitchSetup();                                                                                // Configure the T, P, N and D switches for HS DAC calibration
  InitVBIAS0();                                                                                 // Init LPDAC to turn on VZERO0 output - sets common mode voltage of HSDAC
  InitAfeDie();                                                                                 // Init HSDAC, Excitation amplifier and associated switch setting
  AdcCalibrate(GNPGA_1, 0);                                                                     // Routine that calls functions to calibrate ADC    
  DacCalibrate(0);                                                                              // Internal HSDAC Calibration
  AfeAdcPgaCfg(GNPGA_1_5, 0);                                                                   // Enable PGA G = 1.5
  Open_All_Hp_Switches(); 
  
  // LPTIA Calibrate Gain Resistor
  for(loop = 0; loop < sizeof(rtia_results)/sizeof(rtia_results[0]); loop++)
  {
    rtia_results[loop]= LPTiaCalibrateGainResistor();                                           // Measure RTIA a few times, take the final one as seems to take a few tries to reach optimum 
  }
  PulseCfg.actual_Rtia = rtia_results[sizeof(rtia_results)/sizeof(rtia_results[0]) - 1];        // Save the last result
  Open_All_Hp_Switches();
  pADI_AFE->LPTIASW0 = 0x302C;                                                                  // Set switches
  ChargeECSensor();                                                                             // Takes 4secs but improves sensor stabilisation time
  // Meas_Board_Offset();                                                                       // Enable with sensor removed to get electronic offset in LSBs, remove for now as should be part of sensor baseline correction
  return 1;
}

//----------------------------------------------------------
//      SPI Init
//----------------------------------------------------------
void SPI_Init(void)
{
  SpiCfg(pADI_SPI0, BITM_SPI_CTL_CSRST, 0, 0);                                  // Start with Disabled
  delay(5);
  SpiCfg(pADI_SPI0, BITM_SPI_CTL_CSRST, 0, BITM_SPI_CTL_CON                     // Cont mode
                                         | BITM_SPI_CTL_OEN                     // MOSI pin normal
                                         | BITM_SPI_CTL_ZEN                     // Send zeroes if empty
                                         | BITM_SPI_CTL_CPOL                            
                                         | BITM_SPI_CTL_SPIEN);                 // Enable
  
  SpiIenEn(pADI_SPI0, (SPI_SLAVE_TX_BYTES_MAX -1 ));                            // Interrupt on X bytes
  SpiFifoFlush(pADI_SPI0, BITM_SPI_CTL_TFLUSH, BITM_SPI_CTL_RFLUSH);	        // Flush FIFO 
}

//==============================================================================
//                      SPI Slave Int Handler
//==============================================================================
void SPI0_Int_Handler(void)
{
  unsigned int lcl_SPISTA;
  static unsigned char ucRxCnt = 0;
 // static unsigned char ucTxCnt = 3;
  
  lcl_SPISTA = SpiSta(pADI_SPI0);
   
  if((lcl_SPISTA & BITM_SPI_STAT_CSERR) == BITM_SPI_STAT_CSERR)
  {
   SPI_Init();
  }
   /*
  // Transmit
  if((lcl_SPISTA & BITM_SPI_STAT_TXIRQ) == BITM_SPI_STAT_TXIRQ)                 // Slave Receive IRQ - no of bytes is Tx
  {
   if (ucTxCnt < command_data.cur_bytes_to_send)                                // In case more bytes are requested than needed
   {
    SpiTx(pADI_SPI0, ucSlaveToMaster[ucTxCnt++]);                               // Prepare SPI byte for next transfer
    SpiTx(pADI_SPI0, ucSlaveToMaster[ucTxCnt++]);                               // 
    SpiTx(pADI_SPI0, ucSlaveToMaster[ucTxCnt]);                                 // 
   }
   else
   {                                         
    SpiTx(pADI_SPI0, 0xFF);                                                     // Send 0xFF if any more bytes are requested
   }
  }
  */
  // Receive
  if((lcl_SPISTA & BITM_SPI_STAT_RXIRQ) == BITM_SPI_STAT_RXIRQ)                 // Slave Receive IRQ - no of bytes expected is received
  {
   ucRxCnt = 0;
   master_data.cmd = SpiRx(pADI_SPI0);                                          // read first byte as the command  
   do{
     ucRxCnt++;
     master_data.data_byte[ucRxCnt - 1] = SpiRx(pADI_SPI0);                     // -1 as starting array fill from 0 , read data from SPI0 FIFO
   }while(ucRxCnt < SPI_MASTER_RX_BYTES_MAX);                                   //
   if(ucRxCnt >= SPI_SLAVE_TX_BYTES_MAX)                                        // 3 bytes and CS is high again
   {
    ucRxCnt = 0;
    new_master_data = TRUE;
   }
   pADI_SPI0->STAT |=BITM_SPI_STAT_RXIRQ;                                       // Clear the flag                                   
  }
  /*
  if((lcl_SPISTA & BITM_SPI_STAT_CSRISE) == BITM_SPI_STAT_CSRISE)               // Detect CS rising edge
  {
   ucTxCnt = 3;
  }*/
  // Underflow
  if((lcl_SPISTA & BITM_SPI_STAT_TXUNDR) == BITM_SPI_STAT_TXUNDR)               // Slave Tx FIFO underflow IRQ
  {
    pADI_SPI0->STAT |=BITM_SPI_STAT_TXUNDR;                                     // Clear the Tx underflow interrupt
  } 
}

//==============================================================================
//                      Setup the timer for Scheduler
//==============================================================================
void Setup_GPTimer0(void)
{
  GptLd(pADI_TMR0, 26000/4);                                                    // 1ms
  GptCfg(pADI_TMR0, TCTL_CLK_HFOSC,
         TCTL_PRE_DIV1, BITM_TMR_CTL_MODE |BITM_TMR_CTL_EN);
}

//==============================================================================
//                      Stop the timer for Scheduler
//==============================================================================
void Stop_GPTimer0(void)
{
  GptCfg(pADI_TMR0, TCTL_CLK_HFOSC,
         TCTL_PRE_DIV1, 0);
}

//==============================================================================
//                      GPTimer Int Handler
//==============================================================================
void GP_Tmr0_Int_Handler(void)
{
  GptClrInt(pADI_TMR0, 1);
  fTimeBaseEvent = 1;
  __DSB();
}

//==============================================================================
//                      delay routine
//==============================================================================     
void delay(volatile unsigned int del)
{
  while (del-- > 0);  
}

//==============================================================================
//                      i2C Slave Int Handler
//==============================================================================
void I2C0_Slave_Int_Handler(void)
{
  unsigned int uiI2CStatus, uidummy, uiloop;
  static unsigned char ucRxCnt = 0;
  
  #ifdef EVAL_VERSION
    static unsigned int uiTxCnt = 1; 
  #else
    static unsigned int uiTxCnt = 0; 
  #endif
    
  static bool_t byte_toggle = FALSE;
  const unsigned int val = (sizeof(instruction_list)/sizeof(instruction_list[0][0])) / 2;        // Divide by 2 as 2D array
  
  uiI2CStatus = pADI_I2C0->SSTAT;                                                                // Read the status flag to determine read/write 
  
  // Master Read Request
  if ((uiI2CStatus & 0x4) == 0x4)                                                                // If I2C Master Tx IRQ - read Request
  {
    switch(command_data.cur_cmd)
    {
     case READ_PULSE_TEST_RESULTS:
      if (uiTxCnt < sendDataCnt)
      {
       switch(byte_toggle)
       {
        case FALSE:
          pADI_I2C0->STX = PulseData[uiTxCnt].ILpTia0 >> 8;                                     // Tx Upper byte
          byte_toggle = TRUE;
         break;
        case TRUE:
          pADI_I2C0->STX = PulseData[uiTxCnt].ILpTia0 & 0xFF;                                   // Tx lower byte
          uiTxCnt++;                                                                            // only increment the tx count on every second time as transmitting 16bit results in 8byte i2C bytes
          byte_toggle = FALSE;
         break;
       }
      }
      else
      {                    
       pADI_I2C0->STX = 0xFF;                                                                   // Send 0xFF if any more bytes are requested
      }
      break;
    case READ_EIS_RESULTS:
      if (uiTxCnt < SIZE_EIS_RESULTS)
      {
       pADI_I2C0->STX = eis_array[uiTxCnt];                                                     // Tx byte
       uiTxCnt++;    
      }
      else
      {
       pADI_I2C0->STX = 0xFF;                                                                   // Send 0xFF if any more bytes are requested
      }
      break;
    case READ_EIS_RESULTS_FULL:
      if (uiTxCnt < SIZE_EIS_RESULTS_FULL)
      {
       pADI_I2C0->STX = eis_array_full[uiTxCnt];                                                // Tx byte
       uiTxCnt++;    
      }
      else
      {
       pADI_I2C0->STX = 0xFF;                                                                   // Send 0xFF if any more bytes are requested
      }
      break; 
    case READ_UNIQUE_ID:
      if (uiTxCnt < command_data.cur_bytes_to_send)                                             // In case more bytes are requested than needed
      {
       pADI_I2C0->STX = ucSlaveToMaster[uiTxCnt];
       uiTxCnt++;
      }
      else
      {
       pADI_I2C0->STX = 0xFF;                                                                   // Send 0xFF if any more bytes are requested
      }
      break;    
    default:
      if (uiTxCnt < command_data.cur_bytes_to_send)                                             // In case more bytes are requested than needed
      {
       pADI_I2C0->STX = result_array[command_data.cmd_index][uiTxCnt];
       uiTxCnt++;
      }
      else
      {
       pADI_I2C0->STX = 0xFF;                                                                   // Send 0xFF if any more bytes are requested
      }
      break;
    }
  }
  // Master Write Request
  if ((uiI2CStatus & 0x8) == 0x8)                                                               // If I2C Slave Rx IRQ
  {
   // Received bytes hold instruction and any additional data from the Master 
   if(ucRxCnt == 0)
   {
    master_data.cmd = I2cRx(pADI_I2C0, SLAVE);                                                  // First byte is the command
   }
   else
   {
    if (ucRxCnt < I2C_MASTER_RX_BYTES_MAX)                                                      // As long as we havent received more than max
    {
     master_data.data_byte[ucRxCnt - 1] = I2cRx(pADI_I2C0, SLAVE);                              // Store, -1 as already stored the cmd
    }
    else
    {
     uidummy = I2cRx(pADI_I2C0, SLAVE);                                                         // Throw away
     uidummy = uidummy;                                                                         // Lose compiler warning
    }   
   }
   ucRxCnt++;
  }
  // I2C Stop detected IRQ - Means end of read or write
  if ((uiI2CStatus & 0x400)== 0x400)                                                            
  {
   I2cFifoFlush(pADI_I2C0,SLAVE,ENABLE);                                                        // Enable flush of Slave FIFOs
   I2cFifoFlush(pADI_I2C0,SLAVE,DISABLE);
   ucRxCnt = 0;
   #ifdef EVAL_VERSION
    uiTxCnt = 1;
   #else
    uiTxCnt = 0;
   #endif
   new_master_data = TRUE;
   byte_toggle = FALSE;
  
   // Now search for the instruction as a write command may have been received
   for(uiloop = 0; uiloop < val; uiloop++)
   {
    if(instruction_list[uiloop][0] == master_data.cmd)                                          // Match to our instruction list ?
    {
     command_data.old_cmd = command_data.cur_cmd;                                               // Store this for later in case we want it, maybe for temperature command in future
     command_data.cur_cmd = (COMMAND)instruction_list[uiloop][0];                               // Yes, load into current working structure
     command_data.cur_bytes_to_send = (QTY_BYTES_TO_SEND)instruction_list[uiloop][1];           // How many bytes expected to be transmitted back   
     command_data.cmd_index = uiloop;                                                           // Get the index and use for i2C transmissions 
     break;                                                                                     // exit, can add more later if needed
    }   
   }
  }
}

//==============================================================================
//                      Initialise the NVIC
//==============================================================================
void Init_NVIC(void)
{  
  NVIC_EnableIRQ(TMR0_EVT_IRQn);                                                // Enable GPTimer Interrupt in the NVIC
  NVIC_EnableIRQ(I2C_SLV_EVT_IRQn);                                             // Enable i2C Slave interrupt  
  //NVIC_EnableIRQ(UART_EVT_IRQn);                                                // Enable UART interrupt source in NVIC
  NVIC_EnableIRQ(SPI0_EVT_IRQn);                                                // Enable SPI interrupt source in NVIC
  
  // External interrupts config for Sys_Wake pin P1.0
  pADI_GPIO1->OEN &= 0xFFFE;                                                    // Disable P1.0 output.
  pADI_GPIO1->IEN |= 0x0001;                                                    // Enable input path for P1.0 input.
  pADI_XINT0->CFG0 |=0x90;                                                      // External IRQ1 enabled, falling edge
  NVIC_EnableIRQ(XINT_EVT1_IRQn);                                               // Enable Sys_Wake Interrupt in the NVIC
}

//----------------------------------------------
//     Init the sensor with selected type
//----------------------------------------------
uint8_t Sensor_Init(SENSOR_TYPE Sens_Type)
{
  // Electrochemical gas sensor setup, Load the sensor struct from target type
  switch (Sens_Type)
  {
   case DEFAULT:
    Sensor = DefaultSensor;
    break;
   default:
    break;
  } 
 LPDac_Setup();
 return 1;
}

//----------------------------------------------
//     Init the sensor with selected type
//----------------------------------------------
void LPDac_Setup(void)
{
 unsigned int channel = 0;                                                            // Only using Channel 0 
 uint16_t Cbias, Czero; 
 float Actual_Vzero;
                                                                                      // Power up DAC
 LPDacPwrCtrl(CHAN0, PWR_UP);
 Czero = (uint16_t)round((Sensor.VZero - 200) * 63.0/2166 + 0.5);  
 Actual_Vzero = Czero * LSB_Size_6Bit;                                                // As Vzero is 6bit, actual could be 34mV away from expected
 Actual_Vzero = Actual_Vzero * 1000;
 Cbias = (uint16_t)((Actual_Vzero - Sensor.mV_Bias) * 4095.0/2200 + 0.5);
   
 if(Cbias == Czero)
 {
  LPDacCfg(channel,LPDACSWNOR,VBIAS12BIT_VZERO12BIT_SHORT,LPDACREF2P5);               // For a Zero bias sensor, short vzero to vbias
 }
 else
 {
  LPDacCfg(channel,LPDACSWNOR,VBIAS12BIT_VZERO6BIT,LPDACREF2P5);
 }
 LPDacWr(0,Czero,Cbias);

 // ULP TIA/PA setup
 AfeLpTiaPwrDown(channel,0);
 AfeLpTiaSwitchCfg(channel,SWMODE_SHORT);                                             // Short TIA feedback with diode for Sensor setup
 Sensor.RGainOhms = RGains[Sensor.RGainIndex];
 AfeLpTiaCon(channel, Sensor.RLoad, (LPTIA_RGAIN_Type)Sensor.RGainIndex, SNS_DC_RFILTER_DEFAULT0);// Program TIA settings

 // Place a delay here if sensor requires long time settling
 AfeLpTiaSwitchCfg(channel, SWMODE_NORM);                                             // TIA switch to normal mode for measurement
}

//-------------------------------------------------------------
//              InitAfeLPDac
//-------------------------------------------------------------
void InitAfeLPDac(void)
{ 
  pADI_AFE->LPDACDAT0 = 0x0F813;
  pADI_AFE->LPDACCON0 &= 0xFFFD;                                                        // Power on ULP DAC0
  LPDacCfg(0, LPDACSWNOR,
           VBIAS12BIT_VZERO6BIT, LPDACREF2P5);                                          // Configure LPDAC0 - direct writes to DACDAT0, normal mode for switches, VZERO 6bit, VBIAS 12bit, 2.5V reference
  delay_10us(100);                                                                      // Delay for sensor setup
}

//-------------------------------------------------------------
//   @brief uint8_t SnsMeasure(uint8_t channel, int16_t *pResult, uint16_t iNumber)
//          ======== start DC measurement
//   @param channel :{0,1}
//         - 0 Sensor channel 0
//         - 1 Sensor channel 1
//   @param pResult: the pointer to array which stored measured data.
//  @param iNumber: {0-65535}
//         -  Data number will be measured
//-------------------------------------------------------------
uint8_t Sensor_Measure(uint8_t channel, int32_t *pResult, uint16_t iNumber)
{
   AfeAdcChan(MUXSELP_LPTIA0_LPF, MUXSELN_LPTIA0_N);  
   AfeAdcPgaCfg(GNPGA_1, 0); 
   AfeSysCfg(ENUM_AFE_PMBW_LP, ENUM_AFE_PMBW_BW50);
   AfeAdcPwrUp(BITM_AFE_AFECON_ADCEN);                                                  // turn on ADC
   pADI_AFE->AFECON &= ~BITM_AFE_AFECON_SINC2EN;                                        // Turn off LPF filter
   AfeAdcFiltCfg(SINC3OSR_4, 
                 SINC2OSR_22, 
                 LFPBYPEN_BYP, 
                 ADCSAMPLERATE_800K);
                                                                     
   pADI_AFE->AFECON &= ~BITM_AFE_AFECON_SINC2EN;
delay_10us(10);
pADI_AFE->AFECON |= BITM_AFE_AFECON_SINC2EN;

   AfeAdcGo(BITM_AFE_AFECON_ADCCONVEN);                                                 // Start ADC conversion

   for(uint16_t ix=0; ix<iNumber; ix++)
   {
      while(!(pADI_AFE->ADCINTSTA&BITM_AFE_ADCINTSTA_SINC2RDY));                        // wait till ready ADC data
      pADI_AFE->ADCINTSTA = BITM_AFE_ADCINTSTA_SINC2RDY;                                // clear the bit
      pResult[ix] = pADI_AFE->SINC2DAT;                                                 // store the result
   }                                                                  
   AfeAdcGo(ADCIDLE);                                                                   // Power down ADC

   return 1;
}
//-------------------------------------------------------------
//              Measure Open Circuit Voltage (OCV)
//-------------------------------------------------------------
void Measure_OCV(void)
{
    int32_t adcBuf[16];
    int32_t total = 0;
    uint8_t i;

    /* -------------------------------------------------- */
    /* STEP 1 — STOP EVERYTHING                          */
    /* -------------------------------------------------- */

    AfeWaveGenGo(false);
    AfeHPDacPwrUp(false);
    AfeHpTiaPwrUp(false);
    AfeAdcGo(ADCIDLE);

    /* -------------------------------------------------- */
    /* STEP 2 — REMOVE ALL BIAS COMPLETELY               */
    /* -------------------------------------------------- */

    LPDacPwrCtrl(CHAN0, PWR_DOWN);     //  IMPORTANT

    /* -------------------------------------------------- */
    /* OPEN ALL SWITCHES                        */
    /* -------------------------------------------------- */

    Open_All_Hp_Switches();            // REMOVE ALL LOADING

    /* -------------------------------------------------- */
    /* DISCONNECT LP TIA PATH                   */
    /* -------------------------------------------------- */

    AfeLpTiaSwitchCfg(CHAN0, SWMODE_AC);

    /* -------------------------------------------------- */
    /* ROUTE ELECTRODES DIRECTLY                */
    /* -------------------------------------------------- */

    AfeSwitchDPNT(
        SWID_ALLOPEN,
        SWID_P5_RE0,        // RE  P node
        SWID_N5_SE0RLOAD,   // WE  N node
        SWID_ALLOPEN
    );

    AfeAdcChan(MUXSELP_P_NODE, MUXSELN_N_NODE);
    AfeAdcPgaCfg(GNPGA_1, 0);
    AfeAdcPwrUp(BITM_AFE_AFECON_ADCEN);

    AfeAdcFiltCfg(SINC3OSR_5,
                  SINC2OSR_1333,
                  LFPBYPEN_NOBYP,
                  ADCSAMPLERATE_800K);

    /* -------------------------------------------------- */
    /* LET SENSOR FLOAT NATURALLY               */
    /* -------------------------------------------------- */

    Delay_ms(2000);     

    /* -------------------------------------------------- */
    /* STARTING CONVERSION                         */
    /* -------------------------------------------------- */

    AfeAdcGo(BITM_AFE_AFECON_ADCCONVEN);

    for(i = 0; i < 16; i++)
    {
        while(!(pADI_AFE->ADCINTSTA & BITM_AFE_ADCINTSTA_SINC2RDY));
        pADI_AFE->ADCINTSTA = BITM_AFE_ADCINTSTA_SINC2RDY;
        adcBuf[i] = pADI_AFE->SINC2DAT;
    }

    AfeAdcGo(ADCIDLE);

    for(i = 1; i < 16; i++)
        total += adcBuf[i];

    OCV_LSB = total / 15;

    /* -------------------------------------------------- */
    /*  (TEMP TEST)      */
    /* -------------------------------------------------- */

    
    AfeSwitchDPNT(SWID_ALLOPEN,SWID_ALLOPEN,SWID_ALLOPEN,SWID_ALLOPEN);
    AfeLpTiaSwitchCfg(CHAN0, SWMODE_NORM);
}





//---------------------------------------------------------------------------------------
//              Function to convert samples and return back via pointer
//---------------------------------------------------------------------------------------
void ADC_Conversion(signed int *avg_samples)
{
  uint32_t pResult[16];
  uint32_t total_lsb;
  uint16_t loopVal, val; 
 
  val = sizeof(pResult)/sizeof(pResult[0]);
  
  // Start
  val -=1;                                                                              // Won't be using the first sample as gets removed  
  pADI_AFE->AFECON &=(~BITM_AFE_AFECON_SINC2EN);
  delay_10us(1);
  pADI_AFE->AFECON |= BITM_AFE_AFECON_SINC2EN;
  delay_10us(1);
  AfeAdcGo(BITM_AFE_AFECON_ADCCONVEN);                                                  // Start ADC conversion
  for(loopVal = 0; loopVal < sizeof(pResult)/sizeof(pResult[0]); loopVal++)
  {
    while(!(pADI_AFE->ADCINTSTA & BITM_AFE_ADCINTSTA_SINC2RDY));                        // Wait till ready ADC data
    pADI_AFE->ADCINTSTA = BITM_AFE_ADCINTSTA_SINC2RDY;                                  // Clear the bit
    pResult[loopVal] = pADI_AFE->SINC2DAT;                                              // Store the result
  }
  AfeAdcGo(ADCIDLE);
  total_lsb = 0;
  
  // Get average of the samples
  for(loopVal = 1; loopVal < sizeof(pResult)/sizeof(pResult[0]); loopVal++)             // Throw away first sample
  {   
    total_lsb += pResult[loopVal];
  }
  *avg_samples = total_lsb / val;
}

//-------------------------------------------------------------
//              Pend Handler - may be needed in future
//-------------------------------------------------------------
void PendSV_Handler(void)
{
}

//-------------------------------------------------------------
//     XInt1 Handler - SYS_Wake, wake from Hibernate
//-------------------------------------------------------------
void Ext_Int1_Handler(void)
{
 volatile unsigned int lcl_stat;
 
 lcl_stat = pADI_XINT0->EXT_STAT;                // Read the status register
 pADI_XINT0->CLR = 0x2;                          // Clear the interrupt
 __DSB();
}

//-------------------------------------------------------------
//              GPIO Initialise
//-------------------------------------------------------------
void GPIO_Init(void)
{
  DioCfgPin(pADI_GPIO0, PIN0, 1);                                               // Config as SCLK
  DioCfgPin(pADI_GPIO0, PIN1, 1);                                               // Config as MOSI
  DioCfgPin(pADI_GPIO0, PIN2, 1);                                               // Config as MISO
  DioCfgPin(pADI_GPIO0, PIN3, 1);                                               // Config as CS
  
  DioCfgPin(pADI_GPIO0, PIN10, 0);                                              // Config as GPIO
  DioCfgPin(pADI_GPIO0, PIN11, 0);                                              // Config as GPIO
  
  //DioCfgPin(pADI_GPIO0, PIN10, 1);                                            // Config as UART for testing
  //DioCfgPin(pADI_GPIO0, PIN11, 1);                                            // Config as UART for testing
  
  DioPulPin(pADI_GPIO0, PIN0, 0);                                               // Disable pullup for SPI as per User Guide instruction
  DioPulPin(pADI_GPIO0, PIN1, 0);                                               // Disable pullup for SPI as per User Guide instruction
  DioPulPin(pADI_GPIO0, PIN2, 0);                                               // Disable pullup for SPI 
  //DioPulPin(pADI_GPIO0, PIN2, 1);                                             // Enable MISO  pullup for SPI
  
#ifdef SPICOMMS
  DioPulPin(pADI_GPIO0, PIN3, 0);                                               // Disable pullup for SPI as per User Guide instruction
#endif
#ifdef I2CCOMMS
  /*SPI_/CS0 configuration as GPIO - Read with GP0IN register*/
  DioCfgPin(pADI_GPIO0,PIN3,0);                                                 //confige as gpio
  DioIenPin(pADI_GPIO0,PIN3,1);                                                 //enable input
  DioPulPin(pADI_GPIO0,PIN3,1);                                                 //enable pull-up
#endif
  
  DioPulPin(pADI_GPIO0, PIN4, 0);                                               // Disable pullup for i2C as per User Guide instruction
  DioPulPin(pADI_GPIO0, PIN5, 0);                                               // Disable pullup for i2C as per User Guide instruction
  
  DioCfgPin(pADI_GPIO0, PIN4, 1);                                               // Config as SCL
  DioCfgPin(pADI_GPIO0, PIN5, 1);                                               // Config as SDA

  DioDsPin(pADI_GPIO0,PIN4,1);                                                  // Set Drive strength high
  DioDsPin(pADI_GPIO0,PIN5,1);                                                  // Set Drive strength high
   
  // Port 1
  DioCfgPin(pADI_GPIO1, PIN0, 0);                                               // Set GPIO1.0 as normal gpio
  DioIenPin(pADI_GPIO1, PIN0, 1);                                               // Set GPIO1.0 as input as used as a wakeup pin
  
  DioCfgPin(pADI_GPIO1, PIN2, 0);                                               // Set GPIO1.2 as normal GPIO
  DioOenPin(pADI_GPIO1, PIN2, 1);                                               // Configure GPIO1.2 as o/p
  
  DioCfgPin(pADI_GPIO1, PIN4, 0);                                               // Set GPIO1.4 as normal GPIO
  //DioOenPin(pADI_GPIO1, PIN4, 1);                                             // Configure GPIO1.4 as o/p
  DioIenPin(pADI_GPIO1, PIN4, 1);                                               // Configure GPIO1.4 as i/p
  DioSetPin(pADI_GPIO1, PIN4);
    
  DioCfgPin(pADI_GPIO1, PIN5, 0);                                               // Set GPIO1.5 as normal GPIO
  //DioOenPin(pADI_GPIO1, PIN5, 1);                                             // Configure GPIO1.5 as o/p
  DioIenPin(pADI_GPIO1, PIN5, 1);                                               // Configure GPIO1.5 as i/p
  DioSetPin(pADI_GPIO1, PIN5);
  
  DioDsPin(pADI_GPIO1, PIN4, 1);                                                // Set Drive strength high
  DioDsPin(pADI_GPIO1, PIN5, 1);                                                // Set Drive strength high
  
  DioOenPin(pADI_GPIO0, PIN10, 0);                                              // Configure GPIO0.10, disable o/p
  DioOenPin(pADI_GPIO0, PIN11, 0);                                              // Configure GPIO0.11, disable o/p
  
  DioIenPin(pADI_GPIO0, PIN10, 0);                                              // Configure GPIO0.10, disable i/p
  DioIenPin(pADI_GPIO0, PIN11, 0);                                              // Configure GPIO0.11, disable i/p
}

//-------------------------------------------------------------
//              i2C Initialise
//-------------------------------------------------------------
uint8_t I2C_Init(uint8_t address)
{
  unsigned int result;
  
  I2cAutoStretch(pADI_I2C0, SLAVE, STRETCH_EN, 0xF); 
  I2C0SIDCfg(address << 1, 0xFC, 0xFD, 0xFE);                                   // Shift the address to fill in the ID registers
  I2cFifoFlush(pADI_I2C0,SLAVE, ENABLE);                                        // Enable flush of Slave FIFOs
  I2cFifoFlush(pADI_I2C0,SLAVE, DISABLE);
  I2cSCfg(pADI_I2C0,
          0,                                                                    // I2C  - no DMA
          BITM_I2C_SCTL_IENREPST |                                              // Enable repeated Start detect interrupt
          BITM_I2C_SCTL_IENSTOP |                                               // Enable Stop detect interrupt
          BITM_I2C_SCTL_IENSTX |                                                // Enable transmit interrupt
          BITM_I2C_SCTL_IENSRX,                                                 // Enable receive interrupt
          BITM_I2C_SCTL_EARLYTXR |                                              // Enable early transmit request - enable to support higher baud rates
          BITM_I2C_SCTL_SLVEN);                                                 // Enable I2C Slave
  
  result = I2cSta(pADI_I2C0, SLAVE);
  result = result;                                                              // stop compiler warning
  
  return 1;
}

#ifdef I2CCOMMS
//-------------------------------------------------------------
//                    Set I2C address
//-------------------------------------------------------------
void SetI2CAddr(uint8_t newaddr)
{
	if((DioRd(pADI_GPIO0)&PIN3)==0)                                         //CS is low, this device is selected for address change
	{
		NVIC_DisableIRQ(I2C_SLV_EVT_IRQn);
		pADI_I2C0->SCTL &= ~(0x1);                                      //Clear I2C_SCTL[0] to disable the slave
		pADI_I2C0->SHCTL |= 0x1;                                        //resets start/stop detection circuits of I2C block
		I2C_Init(newaddr);                                              //re-enables the slave and the interrupt
	}
}
#endif /*I2Ccomms*/

//-------------------------------------------------------------
//              Clock Initialise
//-------------------------------------------------------------
void Clock_Init(void)
{
   DigClkSel(DIGCLK_SOURCE_HFOSC);
   ClkDivCfg(1,1);
   AfeClkSel(AFECLK_SOURCE_HFOSC);
   AfeSysClkDiv(AFE_SYSCLKDIV_1);
}

//-------------------------------------------------------------
//              UART Initialise
//-------------------------------------------------------------
void Uart_Init(void)
{
   DioCfgPin(pADI_GPIO0,PIN10,1);                                               // Setup P0.10 as UART pin
   DioCfgPin(pADI_GPIO0,PIN11,1);                                               // Setup P0.11 as UART pin
   UrtCfg(pADI_UART0,B57600,
          (BITM_UART_COMLCR_WLS|3),0);                                          // Configure UART for 57600 baud rate
   UrtFifoCfg(pADI_UART0, RX_FIFO_1BYTE,                                        // Configure the UART FIFOs for 8 bytes deep
              BITM_UART_COMFCR_FIFOEN);
   UrtFifoClr(pADI_UART0, BITM_UART_COMFCR_RFCLR                                // Clear the Rx/TX FIFOs
              |BITM_UART_COMFCR_TFCLR);

   UrtIntCfg(pADI_UART0,BITM_UART_COMIEN_ERBFI|BITM_UART_COMIEN_ELSI);
}

//==============================================================================
// General delay code. Delays approx. in uSec.
//==============================================================================
void Delay_us(volatile unsigned int delay)
{
  delay *= 4;
  while(delay-- > 0){}
}

//==============================================================================
// General delay code. Delays approx. in mSec.
//==============================================================================
void Delay_ms(volatile unsigned int mSec)
{
  int d1, d2;
  for ( d1 = 0; d1 < mSec; d1++ )
  {
   for ( d2 = 0; d2 < 3715; d2++ )
   {
   }
  }
}

//==============================================================================
// Function to average the temperature and humidity readings
// Inputs: temp and hum raw values from sensor
// Outputs: populates the mvh structure
//==============================================================================
void MVH3000D_AVG_Readings(unsigned short int temp, unsigned short int hum)
{
  unsigned int loop = 0;
  unsigned short int *ptr = NULL;
  unsigned int total;
 
  // First do the temperature averaging
  total = 0;
  ptr = &mvh3000d.temp_result_array[MVH_ARRAY_AVG_SIZE - 1];                    // Pointer to last element of the result array, used to iterate down to get average
  for(loop = 1; loop < MVH_ARRAY_AVG_SIZE; loop++)
  {
    mvh3000d.temp_result_array[MVH_ARRAY_AVG_SIZE - loop] = mvh3000d.temp_result_array[(MVH_ARRAY_AVG_SIZE - loop)-1];		// move up the previous one
    total = total + *ptr;
    ptr--;
  }
  mvh3000d.temp_result_array[0] = temp;                                         // Load the latest result into the array[0] position
  total += temp;                                                                // add it to the running total
  mvh3000d.temp_running_avg = total / MVH_ARRAY_AVG_SIZE;                       // get the average

  // Next do the humidity
  total = 0;
  ptr = &mvh3000d.rh_result_array[MVH_ARRAY_AVG_SIZE - 1];                      // Pointer to last element of the result array, used to iterate down to get average
  for(loop = 1; loop < MVH_ARRAY_AVG_SIZE; loop++)
  {
    mvh3000d.rh_result_array[MVH_ARRAY_AVG_SIZE - loop] = mvh3000d.rh_result_array[(MVH_ARRAY_AVG_SIZE - loop)-1];		// move up the previous one
    total = total + *ptr;
    ptr--;
  }
  mvh3000d.rh_result_array[0] = hum;                                            // Load the latest result into the array[1] position
  total += hum;                                                                 // add it to the running total
  mvh3000d.rh_running_avg = total / MVH_ARRAY_AVG_SIZE;                         // get the average
  
  // Temp measurement convertion to degrees, used for sensor temp comp 
  mvh3000d.TempDegC = (mvh3000d.temp_running_avg / 16383.0) * 165 - 40;
  mvh3000d.Hum = (mvh3000d.rh_running_avg / 16383.0) * 100;
}
