// lp_ADC.h
#ifndef LP_ADC_H
#define LP_ADC_H

#include "utility/nrf_lpcomp.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


// Uses internal reference and 1/5 gain for 0 to 3V range (max input must be less than VDD, the supply voltage)
// lp_ADC_start(A0, adc_handler)
// NOTE: the handler is called on the loop() thread so no need for volatiles or synchronization.
//
// You can use ADC and lp_compator at the same time BUT not on the same pin
// can use getChipTemperature() while lp_comparator is active
// this call starts the SAADC, samples the pin, saves the result and stops the SAADC to save the residual current
// the handler is call with the ADC count when the SAADC has stopped.
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
// non-blocking ~42us (without lp_ADC_calibrate() called)
// non-blocking ~140us if lp_ADC_calibrate() called first.
uint32_t lp_ADC_start(uint32_t ulPin, void (*adc_handler_fun)(int) );

// Can call lp_ADC_calibrate() from loop() at any time and the next
// lp_ADC_start( .. ) will do an offset calibration first
// takes about ~140us (non-blocking) from lp_ADC_start(..) to value returned
// with no calibration takes ~45us (non-blocking)
void lp_ADC_calibrate();


void postADCResult();

#ifdef __cplusplus
}
#endif

#endif // LP_ADC_H
