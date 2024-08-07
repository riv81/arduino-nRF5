// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "Arduino.h"
#include "Stream.h"
#include "BLEUtil.h"

void BLEUtil::addressToString(const unsigned char *in, char* out) {
  String address = "";
  String hex;

  for (int i = 5; i >= 0; i--) {
    if (in[i] < 0x10) {
      address += "0";
    }

    hex = String(in[i], 16);
    address += hex;

    if (i > 0) {
      address += ":";
    }
  }

  address.toCharArray(out, 18);
}

void BLEUtil::printBuffer(Stream* stream, const unsigned char* buffer, unsigned char length) {
  if (stream == NULL) {
    return;
  }
  for (int i = 0; i < length; i++) {
    if ((buffer[i] & 0xf0) == 00) {
      stream->print("0");
    }

    stream->print(buffer[i], HEX);
    stream->print(" ");
  }
  stream->println();
}

