// lp_comparator.h
#ifndef LP_COMPARATOR_H
#define LP_COMPARATOR_H

#include "utility/nrf_lpcomp.h"

typedef enum {
  DOWN,
  UP
} pfod_lpcomp_event;


#ifdef __cplusplus
extern "C" {
#endif


enum lp_ref_enum {
	REF_1_16Vdd = 8,
	REF_2_16Vdd = 0,
	REF_3_16Vdd = 9,
	REF_4_16Vdd = 1,
	REF_5_16Vdd = 10,
	REF_6_16Vdd = 2,
	REF_7_16Vdd = 11,
	REF_8_16Vdd = 3,
	REF_9_16Vdd = 12,
	REF_10_16Vdd = 4,
	REF_11_16Vdd = 13,
	REF_12_16Vdd = 5,
	REF_13_16Vdd = 14,
	REF_14_16Vdd = 6,
	REF_15_16Vdd = 15,
	REF_ext_1 = 7,
	REF_ext_2 = 65543
};

typedef enum lp_ref_enum lp_ref;

// returns true if lp_comparator is currently enabled
// retarting the lp_comparator while already running will just stop it.
bool lp_comparator_isRunning();

// to used 1/2 VDD as ref
// lp_comparator_start(A0, REF_8_16Vdd, handler)
// You can use ADC and lp_compator at the same time BUT not on the same pin
// can use getChipTemperature() while lp_comparator is active
// NOTE: lp_comparator_start returns an error if already initialized and stops the current lp_comparator
// so check  lp_comparator_isRunning() first if restarting comparator
uint32_t lp_comparator_start(uint32_t ulPin, lp_ref refVolts, void (*handler_fun)(int) );

// call this is you want to use the ADC on the same pin
// you will need to start lpcomp again afterwards
void lp_comparator_stop();


#ifdef __cplusplus
}
#endif

#endif // LP_COMPARATOR_H
