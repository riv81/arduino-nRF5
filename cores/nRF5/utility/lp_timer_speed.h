#ifndef LP_TIMER_SPEED
#define LP_TIMER_SPEED
// this file sets the lp_timer speed/resolution
// uncomment the LP_TIMER_US define for lp_timers specified in us
// else if commented out, the timers are in ms
// see notes in lp_timer.h

#define LP_TIMER_US

// the lp_timer.h  has this code
//#ifdef LP_TIMER_US  
//  const static uint32_t MAX_TIMEOUT = 511000000;  // i.e. 511sec
// minimum timeout 156us if argument <156  // i.e. 5*31.5us if argument <156us
//#else  
//  const static uint32_t MAX_TIMEOUT = 4095000;  // i.e. 4095sec
// minimum timeout 1220us (1.22ms) if argument <2  // i.e. 5*244us if argument <2ms
//#endif  

#endif

