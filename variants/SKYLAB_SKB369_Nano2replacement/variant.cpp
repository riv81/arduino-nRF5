/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.

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
// _VARIANT_SKYLAB_SKB369_NANO2_REPLACEMENT_
// this pin mapping is for the Temp/RH board used as a Redbear NanoV2 replacement
// NanoV2 has pins D0/A0/RX, D1/A1/TX, D2/A2/CTS/SDAO, D3/A3/RTS/SCL0
// D6, D7, D8, D9, D10/MRST
// D4/A4, D5/A5
// there is NO led (D13) on this board
const uint32_t g_ADigitalPinMap[] = {
  // A0/D0 - A5/D5
  30, // D0
  29, // D1
  28, // D2
  31, // D3
  2,  // D4/A4  
  3,  // D5/A5

  // D6 - D10
  27,
  6,
  7,
  8,
  21,

  // D11 - D13
  (uint32_t)-1,
  (uint32_t)-1,
  (uint32_t)-1  // no D13, no BUILT-IN LED

};
