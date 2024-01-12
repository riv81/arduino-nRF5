// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "Arduino.h"

#include "BLEDevice.h"
#include "bleConstants.h"


BLEDevice::BLEDevice() :
  _advertisingInterval(DEFAULT_ADVERTISING_INTERVAL_ms),
  _minimumConnectionInterval(DEFAULT_MIN_CONNECTION_INTERVAL_ms),
  _maximumConnectionInterval(DEFAULT_MAX_CONNECTION_INTERVAL_ms),
  _slaveLatency(DEFAULT_SLAVE_LATENCY),
  _connectable(DEFAULT_CONNECTABLE),
  _bondStore(NULL),
  _eventListener(NULL),
  _scanResponseHandler(NULL)
{
}

BLEDevice::~BLEDevice() {
}

unsigned short BLEDevice::getAdvertisingInterval() {
  return _advertisingInterval; // in ms
}
    
unsigned short BLEDevice::getMaximumConnectionInterval() {
  return _maximumConnectionInterval; // in ms
}

bool BLEDevice::getConnectable() {
  return _connectable;
}


void BLEDevice::setEventListener(BLEDeviceEventListener* eventListener) {
  this->_eventListener = eventListener;
}

void BLEDevice::setAdvertisingTimeout(uint16_t timeout_in_secs) {
  this->_advertisingTimeout = timeout_in_secs;
}

void BLEDevice::setAdvertisingInterval(unsigned short advertisingInterval) {
  this->_advertisingInterval = advertisingInterval;
}

void BLEDevice::setConnectionInterval(unsigned short minimumConnectionInterval, unsigned short maximumConnectionInterval) {
  if (maximumConnectionInterval < minimumConnectionInterval) {
    maximumConnectionInterval = minimumConnectionInterval;
  }

  this->_minimumConnectionInterval = minimumConnectionInterval;
  this->_maximumConnectionInterval = maximumConnectionInterval;
}

void BLEDevice::setSlaveLatency(unsigned short slaveLatency) {
  this->_slaveLatency = slaveLatency;
}

void BLEDevice::setConnectable(bool connectable) {
  this->_connectable = connectable;
}

void BLEDevice::setBondStore(BLEBondStore& bondStore) {
  this->_bondStore = &bondStore;
}
