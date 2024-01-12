#ifndef LP_TIMER_H
#define LP_TIMER_H
//lp_timer.h

// 0.7.11  Rev 11  25th May 2023, added LP_TIMER_US for us timings
// LP_TIMER_US is defined in utility/lp_timer_speed.h.h
// with LP_TIMER_US defined (the default), the maximum timeout is reduced from 4069sec to 511sec
// and startTimer_us, startDelay_us and getTimeout_us methods are available.
// use them in preference to startTimer, startDelay and getTimeout which include ms to us multipliers and dividers.
// To revert to the previous lp_timer code with the longer maximum timeout
// comment out the #define LP_TIMER_US in utility/lp_timer_speed.h.h

#include <stdint.h>

#include "utility/app_timer.h"
#include "utility/app_scheduler.h"
#include "Print.h"
#include "utility/lp_timer_speed.h"


class lp_timer {
  public:
    lp_timer();
    // if LP_TIMER_US is defined then timeout is in us in multiples of 30.5us, maximum timer is 511sec  (511000000 us)
    // min is 5 ticks => 152.5us
    // else timeout is in ms in multiples of 244us, i.e. 1ms => 41 ticks of 244us each =>10.11ms,  maximum timer is 4095sec (4095000 ms)
    // min is 5 ticks => 1.222ms
    // NOTE: if the timeout (rounded) is less than 5 RTC timer ticks the delay is set to 5 tick1s.
    uint32_t startTimer(uint32_t timeout_ms, void (*handler)(void)); // if LP_TIMER_US defined this method has a ms to us multiplier startTimer_us(..) instead
    uint32_t startDelay(uint32_t timeout_ms, void (*handler)(void)); // if LP_TIMER_US defined this method has a ms to us multiplier startDelay_us(..) instead
    uint32_t getTimeout(); // if LP_TIMER_US defined this method has a us to ms divider  use getTimeout_us(..) instead

#ifdef LP_TIMER_US
    uint32_t startTimer_us(uint32_t timeout_us, void (*handler)(void));
    uint32_t startDelay_us(uint32_t timeout_us, void (*handler)(void));
    uint32_t getTimeout_us();
#endif

    void setDebugStream(Print* debugOut);
    bool isRunning();
    bool isRepeating(); // true if running and repeating else false if stopped OR single shot
    uint32_t stop();
#ifdef LP_TIMER_US
    // in us
    const static uint32_t MAX_TIMEOUT = 511000;
    const static uint32_t MAX_TIMEOUT_US = 511000000;
    // minimum timeout 156us if argument <156
#else
    // in ms
    const static uint32_t MAX_TIMEOUT = 4095000;
    // minimum timeout 1220us (1.22ms) if argument <2
#endif
    void (*_timeoutHandler)(void);

  protected:
    Print* debugOut;
    // options for mode are defined in utility/app_timer.h
    //    APP_TIMER_MODE_SINGLE_SHOT,                 /**< The timer will expire only once. */
    //    APP_TIMER_MODE_REPEATED                     /**< The timer will restart each time it expires. */
    uint32_t start(uint32_t timeout, void (*handler)(void), app_timer_mode_t mode);

  private:
    uint32_t init(void (*timeout_handler)(void) , app_timer_mode_t mode);
    bool repeating;
    bool created;
    uint32_t timeout;
    app_timer_t timer_data;
    app_timer_t* p_timer_data;

};

#endif // #ifndef LP_TIMER_H
