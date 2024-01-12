// lp_ADC.c
#include "lp_ADC.h"
#include "WVariant.h"
#include "limits.h"
#include "utility\app_util_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NRF52_SERIES

#include "nrf_error.h"

extern int ADC_readResolution; // set in wiring_analog_nRF52.c  by analogReadResolution( ) default 10bit

// these constants set the ADC parameters
// see the nRF52832_PS_v1.3-1117956.pdf for the details

// set in wiring_analog_nRF52.c by call to analogReference(... )
// choices are  AR_DEFAULT or AR_INTERNAL:  => 0.6V gain 1/5 for 3V -> 0.6V fullscale
// and  AR_VDD4  => VDD / 4 and gain 1/5  for VDD -> fulscale

extern uint32_t saadcReference; // default SAADC_CH_CONFIG_REFSEL_Internal; // internal 0.6V ref
extern uint32_t saadcGain; // default SAADC_CH_CONFIG_GAIN_Gain1_5; // gain 1/5 => 3V -> 0.6V fullscale

extern unsigned long adc_settling_time; //  set in wiring_analog_nRF52.c
// default SAADC_CH_CONFIG_TACQ_10us; // < 10 us good for <100K source resisance.
// i.e. a  less than 200K + less than 200K  voltage divider connected to the ADC pin
// The calculated source resistance == the value of two resistors if they were connected in parallel.

/** choice depends on the source resistance and are
  SAADC_CH_CONFIG_TACQ_3us (0UL) < 3 us   for < 10K
  SAADC_CH_CONFIG_TACQ_5us (1UL) < 5 us   for < 40K
  SAADC_CH_CONFIG_TACQ_10us (2UL) < 10 us  for <100K
  SAADC_CH_CONFIG_TACQ_15us (3UL) < 15 us  for <200K
  SAADC_CH_CONFIG_TACQ_20us (4UL) < 20 us  for <400K
  SAADC_CH_CONFIG_TACQ_40us (5UL) < 40 us  for <800K
*/

static uint32_t resolution;
static bool calibrateOffsetFlag = false; // set by lp_ADC_calibrate();
static volatile bool calibrateOffset = false; // picked up from calibrateOffsetFlag, when lp_ADC_start( ..) runs
static uint32_t saadcResolution;
static uint32_t adcPin = 0; // input arg to lp_ADC_start(
static void (*handler_callback)(int) = NULL;

#define LP_ADC_CONFIG_IRQ_PRIORITY 3

static inline uint32_t mapResolution( uint32_t value, uint32_t from, uint32_t to )
{
  if ( from == to )
  {
    return value ;
  }

  if ( from > to )
  {
    return value >> (from - to) ;
  }
  else
  {
    return value << (to - from) ;
  }
}

/**
   @brief Function for enabling interrupts from lp_ADC.

   @param[in] lp_ADC_int_mask Mask of interrupts to be enabled.

*/
__STATIC_INLINE void nrf_lp_ADC_int_enable(uint32_t lp_ADC_int_mask) {
  NRF_SAADC->INTENSET = lp_ADC_int_mask;
}

__STATIC_INLINE void nrf_lp_ADC_int_clear(uint32_t lp_ADC_int_mask) {
  NRF_SAADC->INTENCLR = lp_ADC_int_mask;
}

// clear all interrupts
__STATIC_INLINE void nrf_lp_ADC_int_clear_all() {
  NRF_SAADC->INTEN = 0;
}

/**
   @brief Function for retrieving the state of a specific interrupt.

   @param[in]  int_mask         Interrupt.

   @retval     true                   If the interrupt is enabled.
   @retval     false                  If the interrupt is not enabled.
*/
__STATIC_INLINE bool nrf_lp_ADC_int_enable_check(uint32_t lp_ADC_int_mask) {
  return (NRF_SAADC->INTENSET & lp_ADC_int_mask);
}

static volatile uint32_t adcResult = UINT32_MAX;

static volatile bool adcStarted = false; // set true when started and false when result picked up
static int16_t value; // adc result before scaling


/**
  TASKS_START 0x000 Start the ADC and prepare the result buffer in RAM
  TASKS_SAMPLE 0x004 Take one ADC sample, if scan is enabled all channels are sampled
  TASKS_STOP 0x008 Stop the ADC and terminate any on-going conversion
  TASKS_CALIBRATEOFFSET0x00C Starts offset auto-calibration
  EVENTS_STARTED 0x100 The ADC has started
  EVENTS_END 0x104 The ADC has filled up the Result buffer
  EVENTS_DONE 0x108 A conversion task has been completed. Depending on the mode, multiple conversions might be
  needed for a result to be transferred to RAM.
  EVENTS_RESULTDONE 0x10C A result is ready to get transferred to RAM.
  EVENTS_CALIBRATEDONE0x110 Calibration is complete
  EVENTS_STOPPED 0x114 The ADC has stopped
  EVENTS_CH[0].LIMITH 0x118 Last results is equal or above CH[0]
**/

// this is the interrupt handler for the ADC module
// Must clear the interrupt being handled or will just call this again, and again.
void SAADC_IRQHandler(void) { // handle ADC interrupt
  // calibrate
  if (nrf_lp_ADC_int_enable_check(SAADC_INTENSET_CALIBRATEDONE_Msk)) {
    //nrf_lp_ADC_int_clear(SAADC_INTENSET_CALIBRATEDONE_Msk);
    nrf_lp_ADC_int_clear_all();
    NRF_SAADC->EVENTS_CALIBRATEDONE = 0x00UL;
    // finished calibration now start sample task
    nrf_lp_ADC_int_enable(SAADC_INTENSET_STOPPED_Msk);
    NRF_SAADC->TASKS_STOP = 0x01UL;

    // start
  } else if (nrf_lp_ADC_int_enable_check(SAADC_INTENSET_STARTED_Msk)) {
    //nrf_lp_ADC_int_clear(SAADC_INTENSET_STARTED_Msk);
    nrf_lp_ADC_int_clear_all();
    NRF_SAADC->EVENTS_STARTED = 0x00UL;
    nrf_lp_ADC_int_enable(SAADC_INTENSET_END_Msk);
    NRF_SAADC->TASKS_SAMPLE = 0x01UL;

    // sample
  } else if (nrf_lp_ADC_int_enable_check(SAADC_INTENSET_END_Msk)) {
    // nrf_lp_ADC_int_clear(SAADC_INTENSET_END_Msk);
    nrf_lp_ADC_int_clear_all();
    NRF_SAADC->EVENTS_END = 0x00UL;
    nrf_lp_ADC_int_enable(SAADC_INTENSET_STOPPED_Msk);
    NRF_SAADC->TASKS_STOP = 0x01UL;

    // stop
  } else if (nrf_lp_ADC_int_enable_check(SAADC_INTENSET_STOPPED_Msk)) {
    //nrf_lp_ADC_int_clear(SAADC_INTENSET_STOPPED_Msk);
    nrf_lp_ADC_int_clear_all();
    NRF_SAADC->EVENTS_STOPPED = 0x00UL;
    if (calibrateOffset) {
      value = 0;
    } else { // not calibrate so return result
      // pick up result
      if (value < 0) {
        value = 0;
      }
    }
    NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);
    uint32_t result = mapResolution(value, resolution, ADC_readResolution);
    CRITICAL_REGION_ENTER();
    adcResult = result;
    CRITICAL_REGION_EXIT();
  } else { // unexpected ADC interrupt, just clear all
    nrf_lp_ADC_int_clear_all();
  }
}

// this is called after sleep wakes up
// BEFORE any other callback
// this is called on the loop() thread so no need to use volatiles or synchronization
void postADCResult() {
  // critical to check result
  uint32_t adcPostResult = UINT32_MAX;
  CRITICAL_REGION_ENTER();
  if (adcResult != UINT32_MAX) {
    adcPostResult = adcResult;
    adcResult = UINT32_MAX;
    adcStarted = false;
  }
  CRITICAL_REGION_EXIT();
  if ((adcPostResult != UINT32_MAX) && (handler_callback != NULL)) {
    if (!calibrateOffset) {
      // call callback
      if (adcPostResult > INT_MAX) {
        adcPostResult = INT_MAX;
      }
      handler_callback((int)adcPostResult);
    } else {
      // do the sample now
      lp_ADC_start(adcPin, handler_callback);
    }
  }
}


// Uses internal reference and 1/5 gain for 0 to 3V range (max input must be less than VDD, the supply voltage)
// lp_ADC_start(A0, handler)
// You can use ADC and lp_compator at the same time BUT not on the same pin
// can use getChipTemperature() while lp_comparator is active
// this call starts the SAADC, samples the pin, saves the result and stops the SAADC to save the residual current
// the handler is call with the ADC count when the SAADC has stopped.
// NOTE ulPin is A0 to A7 mapped to chip pin from g_ADigitalPinMap
// for NanoV2 and Skylab NanoV2 replacement only A0 to A5 available

/**
  A0  = AIN5 = pin 4
  A1  = AIN6 = pin 5
  A2  = AIN7 = pin 6
  A3  = AIN5 = pin 7
  A4  = AIN6 = pin 28
  A5  = AIN7 = pin 29
  A6  = AIN6 = pin 30
  A7  = AIN7 = pin 31
**/

// set calibrate flag to perform offset calibration, just once, before next sample
// need to call lp_ADC_start(..) after call lp_ADC_calibrate()
// only calibrates once on next lp_ADC_start(..)
// calibrate setting is cleared after next successful lp_ADC_start(..)
// NOTE: if lp_ADC_start(..) returns an error OR NRF_ERROR_BUSY
// the calibrate flag is not cleared.
void lp_ADC_calibrate() {
  calibrateOffsetFlag = true;
}

// handler_fun is the method called when ADC completes arg is the ADC count
// returns 0 if arguments OK, else error no.
// returns NRF_ERROR_BUSY if conversion already in progress.
uint32_t lp_ADC_start(uint32_t ulPin, void (*adc_handler_fun)(int) ) {
  //  flag = false; // testing only
  handler_callback = adc_handler_fun;
  adcPin = ulPin;

  // do checks first before starting
  if (adc_handler_fun == NULL) {
    return NRF_ERROR_INVALID_PARAM;
  }
  if (ulPin >= PINS_COUNT) {
    return NRF_ERROR_INVALID_PARAM;
  }

  uint32_t pin = SAADC_CH_PSELP_PSELP_NC;

  ulPin = g_ADigitalPinMap[ulPin];

  switch ( ulPin ) {
    case 2:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput0;
      break;

    case 3:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput1;
      break;

    case 4:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput2;
      break;

    case 5:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput3;
      break;

    case 28:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput4;
      break;

    case 29:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput5;
      break;

    case 30:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput6;
      break;

    case 31:
      pin = SAADC_CH_PSELP_PSELP_AnalogInput7;
      break;

    default:
      return NRF_ERROR_INVALID_PARAM;
  }

  bool alreadyStarted = false;
  CRITICAL_REGION_ENTER();
  if (adcStarted) { // started and not complete yet
    alreadyStarted = true;
  } else {
    adcResult = UINT32_MAX;
    adcStarted = true;
  }
  CRITICAL_REGION_EXIT();
  if (alreadyStarted) {
    return NRF_ERROR_BUSY; // still running so ignore this start
  }

  //_sasdc_handler = (pfod_sasdc_handler_fun)handler_fun;
  uint32_t saadcResolution;


  if (ADC_readResolution <= 8) {
    resolution = 8;
    saadcResolution = SAADC_RESOLUTION_VAL_8bit;
  } else if (ADC_readResolution <= 10) {
    resolution = 10;
    saadcResolution = SAADC_RESOLUTION_VAL_10bit;
  } else if (ADC_readResolution <= 12) {
    resolution = 12;
    saadcResolution = SAADC_RESOLUTION_VAL_12bit;
  } else {
    resolution = 14;
    saadcResolution = SAADC_RESOLUTION_VAL_14bit;
  }

  NRF_SAADC->RESOLUTION = saadcResolution;

  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);
  for (int i = 0; i < 8; i++) {
    NRF_SAADC->CH[i].PSELN = SAADC_CH_PSELP_PSELP_NC;
    NRF_SAADC->CH[i].PSELP = SAADC_CH_PSELP_PSELP_NC;
  }
  NRF_SAADC->CH[0].CONFIG =   ((SAADC_CH_CONFIG_RESP_Bypass   << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
                              | ((SAADC_CH_CONFIG_RESP_Bypass   << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
                              | ((saadcGain                     << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
                              | ((saadcReference                << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
                              | ((adc_settling_time      << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
                              | ((SAADC_CH_CONFIG_MODE_SE       << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk);
  NRF_SAADC->CH[0].PSELN = pin;
  NRF_SAADC->CH[0].PSELP = pin;


  NRF_SAADC->RESULT.PTR = (uint32_t)&value;
  NRF_SAADC->RESULT.MAXCNT = 1; // One sample

  nrf_drv_common_irq_enable(SAADC_IRQn, LP_ADC_CONFIG_IRQ_PRIORITY);
  // this is called first for calibration and then to do the actual sample.
  if (calibrateOffsetFlag) { // setting this flag starts calibration
  	if (!calibrateOffset) { // pickup calibrate setting and clear it.
  	  calibrateOffset = true;  // this discards any result and start actual sample after calibration finishes
  	  calibrateOffsetFlag = false;
      nrf_lp_ADC_int_enable(SAADC_INTENSET_CALIBRATEDONE_Msk);
      NRF_SAADC->TASKS_CALIBRATEOFFSET = 0x01UL; 
    } else { // calibrateOffset set so have finished calibration normal ADC BUT lp_ADC_calibrate has been called since calibration started, so leave it set for next sample call
      calibrateOffset = false;  // this is the actual sample, result will be returned.
      nrf_lp_ADC_int_enable(SAADC_INTENSET_STARTED_Msk);
      NRF_SAADC->TASKS_START = 0x01UL;   // will call SAADC_IRQHandler when started
    }
  } else { // calibrateOffsetFlag not set  OR was cleared above when picked up  AND NOT called since lp_ADC_start( ) called to start calibration
  	       // so just a normal ADC sample perhaps after a calibration so clear calibrationOffset flag in any case
     calibrateOffset = false;
     nrf_lp_ADC_int_enable(SAADC_INTENSET_STARTED_Msk);
     NRF_SAADC->TASKS_START = 0x01UL;   // will call SAADC_IRQHandler when started  	  
  }  

  return 0; // OK started
}
#ifdef __cplusplus
}
#endif
