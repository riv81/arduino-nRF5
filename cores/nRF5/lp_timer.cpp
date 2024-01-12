
#include "lp_timer_init.h"
// lp_timer_init.h has LP_TIMER_US define if defined
#include "utility/app_scheduler.h"
// include collection of nordic macros
#include "utility/nordic_common.h"
// include nrf error defines, only NRF_SUCCESS used here but you can use these to check error returns
// which in general are not checked in this code/library
#include "nrf_error.h"
// include scheduled timer, uses LF clock and RTC1  millis(), micros(), delay() moved to RTC2 delay()
// uses SWI0_IRQn and SWI0_IRQHandler
#include "utility/app_timer_appsh.h"
// included LF clock handling. Note: this shares an interrupt with nrf_drv_power but that is not used in these examples/library so no special handling needed
#include "utility/nrf_drv_clock.h"
#include "lp_timer.h"

//For nRF52, the application must ensure calibration at least once
//every 8 seconds to ensure +/-500 ppm clock stability. The
//recommended configuration for ::NRF_CLOCK_LF_SRC_RC on nRF52 is
//rc_ctiv=16 and rc_temp_ctiv=2. This will ensure calibration at
//least once every 8 seconds and for temperature changes of 0.5
//degrees Celsius every 4 seconds. See the Product Specification
//for the nRF52 device being used for more information.

//typedef struct
//{
//  uint8_t source;        /**< LF oscillator clock source, see @ref NRF_CLOCK_LF_SRC. */
//  uint8_t rc_ctiv;      /**< Only for NRF_CLOCK_LF_SRC_RC: Calibration timer interval in 1/4 second
//                              units (nRF51: 1-64, nRF52: 1-32).
//                              @note To avoid excessive clock drift, 0.5 degrees Celsius is the
//                                    maximum temperature change allowed in one calibration timer
//                                    interval. The interval should be selected to ensure this.
//
//                              @note Must be 0 if source is not NRF_CLOCK_LF_SRC_RC.  */
//  uint8_t rc_temp_ctiv; /**<  Only for NRF_CLOCK_LF_SRC_RC: How often (in number of calibration
//                              intervals) the RC oscillator shall be calibrated if the temperature
//                              hasn't changed.
//                                  0: Always calibrate even if the temperature hasn't changed.
//                                  1: Only calibrate if the temperature has changed (nRF51 only).
//                                  2-33: Check the temperature and only calibrate if it has changed,
//                                        however calibration will take place every rc_temp_ctiv
//                                        intervals in any case.
//
//                              @note Must be 0 if source is not NRF_CLOCK_LF_SRC_RC.
//
//                              @note For nRF52, the application must ensure calibration at least once
//                                    every 8 seconds to ensure +/-250ppm clock stability. The
//                                    recommended configuration for NRF_CLOCK_LF_SRC_RC on nRF52 is
//                                    rc_ctiv=16 and rc_temp_ctiv=2. This will ensure calibration at
//                                    least once every 8 seconds and for temperature changes of 0.5
//                                    degrees Celsius every 4 seconds. See the Product Specification
//                                    for the nRF52 device being used for more information.*/
//  uint8_t xtal_accuracy; /**< External crystal clock accuracy used in the LL to compute timing windows.
//
//                              @note For the NRF_CLOCK_LF_SRC_RC clock source this parameter is ignored. */
//} nrf_clock_lf_cfg_t;


//#define DEBUG
void lp_timer::setDebugStream(Print* _debugOut) {
  debugOut = _debugOut;
}

#ifdef LP_TIMER_US
// in us
const static uint32_t LP_TIMER_SCALING = 1000;
#endif

typedef struct {
  uint32_t                    ticks_to_expire;                            /**< Number of ticks from previous timer interrupt to timer expiry. */
  uint32_t                    ticks_at_start;                             /**< Current RTC counter value when the timer was started. */
  uint32_t                    ticks_first_interval;                       /**< Number of ticks in the first timer interval. */
  uint32_t                    ticks_periodic_interval;                    /**< Timer period (for repeating timers). */
  bool                        is_running;                                 /**< True if timer is running, False otherwise. */
  app_timer_mode_t            mode;                                       /**< Timer mode. */
  app_timer_timeout_handler_t p_timeout_handler;                          /**< Pointer to function to be executed when the timer expires. */
  void *                      p_context;                                  /**< General purpose pointer. Will be passed to the timeout handler when the timer expires. */
  void *                      next;                                       /**< Pointer to the next node. */
} timer_node_t;


extern "C" {
  static void internalTimeoutHandler(lp_timer* timerPtr) {
    if (timerPtr == NULL) { // should not happen
      return;
    }
    if (timerPtr->_timeoutHandler != NULL) {
      timerPtr->_timeoutHandler();
    }
  }
}

lp_timer::lp_timer() {
  timer_data = {0};
  p_timer_data = &timer_data;
  created = false;
  debugOut = NULL;
}

uint32_t lp_timer::init(void (*timeout_handler)(void), app_timer_mode_t mode) {
  uint32_t err_code;
  // Create timers
  _timeoutHandler = timeout_handler;
  err_code = app_timer_create(p_timer_data, mode, (app_timer_timeout_handler_t) internalTimeoutHandler);
  repeating = (mode == APP_TIMER_MODE_REPEATED);
#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->print("init return:"); debugOut->println(err_code);
  }
#endif
  return err_code;
}

bool lp_timer::isRepeating() {
  if (!isRunning()) {
    return false;
  } else {
    return repeating;
  }
}

bool lp_timer::isRunning() {
  if (!created) {
    return false;
  } // else
  timer_node_t* node  = (timer_node_t*)(p_timer_data);
  return node->is_running;
}

uint32_t lp_timer::stop() {
  if (!isRunning()) {
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->print("stop() not running return:"); debugOut->println(0);
    }
#endif
    return 0; // not running
  }
  uint32_t err_code;
  err_code = app_timer_stop(p_timer_data);
#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->print("stop() return:"); debugOut->println(err_code);
  }
#endif
  return err_code;
}

// if LP_TIMER_US defined in utility/lp_timer_speed.h.h
// this method returns ms
uint32_t lp_timer::getTimeout() {
  if (!created) {
    return 0;
  } // else
#ifdef LP_TIMER_US
  return timeout / LP_TIMER_SCALING;
#else
  return timeout;
#endif
}


// if LP_TIMER_US defined in utility/lp_timer_speed.h.h
#ifdef LP_TIMER_US
uint32_t lp_timer::startDelay(uint32_t timeout_ms, void (*handler)(void)) {
  if (timeout_ms > MAX_TIMEOUT) {
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->print("startDelay timeout:"); debugOut->print(timeout_ms); debugOut->println("ms");
      debugOut->print("startDelay timeout limited to "); debugOut->print(MAX_TIMEOUT); debugOut->println("ms");
    }
#endif
    timeout_ms = MAX_TIMEOUT;
  }
  return start(timeout_ms * LP_TIMER_SCALING, handler , APP_TIMER_MODE_SINGLE_SHOT);
}

uint32_t lp_timer::startTimer(uint32_t timeout_ms, void (*handler)(void)) {
  if (timeout_ms > MAX_TIMEOUT) {
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->print("startTimer timeout:"); debugOut->print(timeout_ms); debugOut->println("ms");
      debugOut->print("startTimer timeout limited to "); debugOut->print(MAX_TIMEOUT); debugOut->println("ms");
    }
#endif
    timeout_ms = MAX_TIMEOUT;
  }
  return start(timeout_ms * LP_TIMER_SCALING, handler , APP_TIMER_MODE_REPEATED);
}

uint32_t lp_timer::startTimer_us(uint32_t timeout_us, void (*handler)(void)) {
  if (timeout_us > MAX_TIMEOUT_US) {
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->print("startTimer_us timeout:"); debugOut->print(timeout_us); debugOut->println("us");
      debugOut->print("startTimer_us timeout limited to "); debugOut->print(MAX_TIMEOUT_US); debugOut->println("us");
    }
#endif
    timeout_us = MAX_TIMEOUT_US;
  }
  return start(timeout_us, handler , APP_TIMER_MODE_REPEATED);
}

uint32_t lp_timer::startDelay_us(uint32_t timeout_us, void (*handler)(void)) {
  if (timeout_us > MAX_TIMEOUT_US) {
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->print("startDelay_us timeout:"); debugOut->print(timeout_us); debugOut->println("us");
      debugOut->print("startDelay_us timeout limited to "); debugOut->print(MAX_TIMEOUT_US); debugOut->println("us");
    }
#endif
    timeout_us = MAX_TIMEOUT_US;
  }
  return start(timeout_us, handler , APP_TIMER_MODE_SINGLE_SHOT);
}

uint32_t lp_timer::getTimeout_us() {
  if (!created) {
    return 0;
  } // else
  return timeout;
}

#else
// previous version
uint32_t lp_timer::startTimer(uint32_t timeout_ms, void (*handler)(void)) {
  return start(timeout_ms, handler , APP_TIMER_MODE_REPEATED);
}

uint32_t lp_timer::startDelay(uint32_t timeout_ms, void (*handler)(void)) {
  return start(timeout_ms, handler , APP_TIMER_MODE_SINGLE_SHOT);
}
#endif

// if LP_TIMER_US defined in utility/lp_timer_speed.h.h
// this method takes us, else ms
// returns 0 if OK
uint32_t lp_timer::start(uint32_t timeout_in, void (*handler)(void), app_timer_mode_t mode) {
  if (handler == NULL) {
    return NRF_ERROR_INVALID_PARAM;// null handler
  }
  uint32_t err_code = 0;
  if (created) {
    if (isRunning()) {
      err_code = NRF_ERROR_INVALID_STATE;
#ifdef DEBUG
      if (debugOut != NULL) {
        debugOut->print("lp_timer.start() still running return:"); debugOut->println(err_code);
      }
#endif
      return err_code;// error still running
    } else {
      // stopped re-initialize
#ifdef DEBUG
      if (debugOut != NULL) {
        debugOut->print("lp_timer.start() created re-init ");
      }
#endif
      err_code = init(handler, mode);
      if (err_code != 0) {
#ifdef DEBUG
        if (debugOut != NULL) {
          debugOut->print("lp_timer.start() init return: ");    debugOut->println(err_code);
        }
#endif
        return err_code;
      }
    }
  } else {
    //  not created yet
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->print("lp_timer.start() not yet created ");
    }
#endif
    err_code = init(handler, mode);
    if (err_code != 0) {
#ifdef DEBUG
      if (debugOut != NULL) {
        debugOut->print("lp_timer.start() init return: ");    debugOut->println(err_code);
      }
#endif
      return err_code;
    }
    created = true;
  }
#ifdef LP_TIMER_US
  if (timeout_in > MAX_TIMEOUT_US) {
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->print("lp_timer timeout:"); debugOut->print(timeout_in); debugOut->println("us");
      debugOut->print("lp_timer timeout limited to "); debugOut->print(MAX_TIMEOUT_US); debugOut->println("us");
    }
#endif
    timeout_in = MAX_TIMEOUT_US; // limit to max RTC can handle
  }
#else
  if (timeout_in > MAX_TIMEOUT) {
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->print("lp_timer timeout:"); debugOut->print(timeout_in); debugOut->println("ms");
      debugOut->print("lp_timer timeout limited to "); debugOut->print(MAX_TIMEOUT); debugOut->println("ms");
    }
#endif
    timeout_in = MAX_TIMEOUT; // limit to max RTC can handle
  }
#endif

  uint32_t ticks = APP_TIMER_TICKS(timeout_in, APP_TIMER_PRESCALER);
#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->print("lp_timer.start() timeout_in == "); debugOut->print(timeout_in);
#ifdef LP_TIMER_US    
    debugOut->println("us");
#else
    debugOut->println("ms");
#endif
    debugOut->print("lp_timer.start() ticks == "); debugOut->println(ticks);
  }
#endif
  if (ticks < APP_TIMER_MIN_TIMEOUT_TICKS) {
    ticks = APP_TIMER_MIN_TIMEOUT_TICKS;
#ifdef DEBUG
    if (debugOut != NULL) {
      debugOut->println("lp_timer.start() ticks < 5 set to 5 ticks");
    }
#endif
  }
  timeout = timeout_in;
  err_code = app_timer_start(p_timer_data, ticks, this);
#ifdef DEBUG
  if (debugOut != NULL) {
    debugOut->print("lp_timer.start() return:"); debugOut->println(err_code);
  }
#endif
  return err_code;
}
