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

#include "variant.h"

/*   nRF52832 bare module
 see nRF53832_pins.pdf

*/

// _VARIANT_nRF52832_BARE_MODULE_
// this pin mapping is for generic nRF52832 bare modules
// not all the pins listed here may accessible at the pcb contacts
// NFC pins are re-mapped as general GPIO pin P0.09, P0.10 via -DCONFIG_NFCT_PINS_AS_GPIOS compile setting
const uint32_t g_ADigitalPinMap[] = {
  0, // P0.00
  1, // P0.01
  2, // P0.02
  3, // P0.03
  4, // P0.04
  5, // P0.05
  6, // P0.06
  7, // P0.07
  8, // P0.08
  9, // P0.09
  10, // P0.10
  11, // P0.11
  12, // P0.12
  13, // P0.13
  14, // P0.14
  15, // P0.15
  16, // P0.16
  17, // P0.17
  18, // P0.18
  19, // P0.19
  20, // P0.20
  21, // P0.21
  22, // P0.22
  23, // P0.23
  24, // P0.24
  25, // P0.25
  26, // P0.26
  27, // P0.27
  28, // P0.28
  29, // P0.29
  30, // P0.30
  31 // P0.31
};
