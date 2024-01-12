// bleConstants.h

#ifndef BLE_CONSTANTS_H
#define BLE_CONSTANTS_H

#include <stdint.h>
/** NOTE
Connection Interval Min ≥ 7.5 ms (multiples of 1.25 ms) 
   suggested min 15ms and then in multiples of 15ms
Connection Interval Max < 4sec
   suggested Interval Min + 15 ms ≤ Interval Max (Interval Max == 15 ms is allowed)
Interval Max * (Slave Latency + 1) ≤ 2 seconds
Interval Max * (Slave Latency + 1) * 3 < connSupervisionTimeout
Slave Latency ≤ 30
2 seconds ≤ connSupervisionTimeout ≤ 6 seconds
****/

// default min/max connection interval in ms
// internally converted to multiples of 1.25ms i.e. 100ms => converted internally to 80
// use setConnectionInterval( , ) to override these settings
const unsigned short DEFAULT_MIN_CONNECTION_INTERVAL_ms = 100;
const unsigned short DEFAULT_MAX_CONNECTION_INTERVAL_ms = 150;

// default connection slave latency
// use setSlaveLatency() to override. Non-zero lets device skip responding to connection events if no data to send
// must be in the range 
// 0 to ((connectionSupervisionTimeout / connectionInterval) 1). 
// i.e. must respond within supervision timeout even if no data
const unsigned short DEFAULT_SLAVE_LATENCY = 0; // must respond to every connection event

// Connection Supervision Timeout in ms
// internally converted to multiples of 10ms i.e. 4000ms => converted internally to 400
// must be in the range 100ms to 32000ms (32sec) 
// no setter for this value, edit default if you need to
const unsigned short DEFAULT_CONNECTION_SUPERVISION_TIMEOUT_ms = 4000;

// for nRF52 tx power 
// -40, -30, -20, -16, -12, -8, -4, 0, 4
// use setTxPower( ) to override this setting
const uint8_t DEFAULT_BLE_TX_POWER = 4;

// advertising in ms
// internally converted to multiples of 0.625ms i.e. 100ms => converted internally to 160
// use setAdvertisingInterval( ) to override this in the range 20ms to 10240ms (10.24sec)
const unsigned short DEFAULT_ADVERTISING_INTERVAL_ms = 500;

// default connectable setting
// use setConnectable( ) to override this
const bool DEFAULT_CONNECTABLE = true;

#endif //  #ifndef BLE_CONSTANTS_H
