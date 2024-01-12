/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2022 Forward Computing and Control Pty. Ltd. All right reserved.
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_nRF52832_BARE_MODULE_
#define _VARIANT_nRF52832_BARE_MODULE_

/** Master clock frequency */
#define VARIANT_MCK       (64000000ul)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// _VARIANT_nRF52832_BARE_MODULE_
// this pin mapping is for generic nRF52832 bare modules
// see nRF53832_pins.pdf

// not all the pins listed here may accessible at the pcb contacts
// NFC pins are re-mapped as general GPIO pin P0.09, P0.10
// NFC pins are re-mapped as general GPIO pin P0.09, P0.10 via -DCONFIG_NFCT_PINS_AS_GPIOS compile setting

// Number of pins defined in PinDescription array
#define PINS_COUNT           (32u)
#define NUM_DIGITAL_PINS     (32u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (0u)

// LEDs  NOTE no built in led on bare boards
// so LED_BUILTIN is not defined
//#define LED_BUILTIN           13

/*
 * Analog pins
 */
#define PIN_A0               (4)
#define PIN_A1               (5)
#define PIN_A2               (6)
#define PIN_A3               (7)
#define PIN_A4               (28)
#define PIN_A5               (29)
#define PIN_A6               (30)
#define PIN_A7               (31)

static const uint8_t A0  = PIN_A0 ; // AIN5
static const uint8_t A1  = PIN_A1 ; // AIN6
static const uint8_t A2  = PIN_A2 ; // AIN7
static const uint8_t A3  = PIN_A3 ; // AIN5
static const uint8_t A4  = PIN_A4 ; // AIN6
static const uint8_t A5  = PIN_A5 ; // AIN7
static const uint8_t A6  = PIN_A6 ; // AIN6
static const uint8_t A7  = PIN_A7 ; // AIN7
#define ADC_RESOLUTION    14

// Other pins 
// no AREF

/*
 * Serial interfaces
 */
// default Tx = P0.30,  Rx = P0.29
// these pins can be changed with Serial.setPins(rx_pin,tx_pin);
#define PIN_SERIAL_TX       (29)
#define PIN_SERIAL_RX       (30)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1
// P0.06, P0.07, P0.08    in master mode 
// these pins can be changed with SPI.setPins(miso_pin,clk_pin,mosi_pin);
#define PIN_SPI_MISO         (6u)
#define PIN_SPI_MOSI         (7u)
#define PIN_SPI_SCK          (8u)

//static const uint8_t SS   = 31u ; // Chip select output not specified 
// on the nRF52 you do not need to set the Chip select pin as an Output to run the SPI in master mode
// the nRF52 SPI init only uses the MOSI,MISO,SCK values
static const uint8_t MOSI = PIN_SPI_MOSI ; // P0.06
static const uint8_t MISO = PIN_SPI_MISO ; // P0.07
static const uint8_t SCK  = PIN_SPI_SCK ;  // P0.08

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1
// P0.09, P0.10
// use the NFC pins for Wire as these pins are generally exposed
// need to choose other pins if NFC enabled
// these default pins can be changed with Wire.setPins(sda_pin,scl_pin);
#define PIN_WIRE_SDA         (9u)
#define PIN_WIRE_SCL         (10u)

static const uint8_t SDA = PIN_WIRE_SDA; // P0.09
static const uint8_t SCL = PIN_WIRE_SCL; //  P0.10

/*
 * P0.21 as Reset may or may not be avaiable
 */

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
