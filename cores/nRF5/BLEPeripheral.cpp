// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "BLEUuid.h"

#include "BLEDeviceLimits.h"
#include "BLEUtil.h"

#include "BLEPeripheral.h"

//#define BLE_PERIPHERAL_DEBUG

#define DEFAULT_DEVICE_NAME "Arduino"
#define DEFAULT_APPEARANCE  0x0000


#ifdef __cplusplus
extern "C" void BLEPeripheralInstancePoll() {
  if (BLEPeripheral::pollInstance != NULL) {
  	BLEPeripheral::pollInstance->poll();
  }
}	
#endif // #ifdef __cplusplus

BLEPeripheral* BLEPeripheral::pollInstance = NULL; // allocate space for the static

/*------------------------------------------------------------------*/
/* Paser helper from modified from Adafruit BLEScanner.cpp code
  NOTE: the return and args differ from Adafruit's method
  ------------------------------------------------------------------*/
/**************************************************************************/
/*!
    @file     BLEScanner.cpp
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
/**
  parseScanDataByType
  NOTE: the return and args differ from Adafruit's method
   @param scandata
   @param scanlen
   @param type  a type from ble_gap.h BLE_GAP_AD_TYPE_xxx defines
   @param buf     Output buffer
   @param lenWritten pointer to uint8_t returns the number of bytes written to result
   @return pointer to static reused buffer which is overwritted on each call and so needs to be copied if to be kept
           returns NULL if not found
*/
uint8_t* BLEPeripheral::parseScanDataByType(const uint8_t* scandata, uint8_t scanlen, uint8_t type, uint8_t& lenWritten) {

  static uint8_t buf[BLE_ADV_LEN+1]; // +1 for '\0'

  memset(buf, 0, sizeof(buf)); // clear return ALWAYS
  lenWritten = 0;
  uint8_t bufsize = BLE_ADV_LEN;
  uint8_t len = 0;
  uint8_t const* ptr = NULL;

  if ( scanlen < 2 ) {
    return 0;
  }

  // len (1+data), type, data
  while ( scanlen ) {
    if ( scandata[1] == type ) {
      len = (scandata[0] - 1);
      ptr = (scandata + 2);
      break;
    } else {
      scanlen  -= (scandata[0] + 1);
      scandata += (scandata[0] + 1);
    }
  }

  // not found return 0
  if (ptr == NULL) {
    return NULL;
  }

  // Only check len if bufsize is input
  if (len > bufsize) {
    len = bufsize;
  }

  memcpy(buf, ptr, len);
  // note if pass in bufsize max adv len then should always have len < bufsize
  lenWritten = len;
  return buf;
}

uint8_t* BLEPeripheral::parseScanDataByType(const ble_gap_evt_adv_report_t* report, uint8_t type, uint8_t& lenWritten) {
  return parseScanDataByType(report->data, report->dlen, type, lenWritten);
}

bool BLEPeripheral::isScanResponse(ble_gap_evt_adv_report_t* report) {
  uint8_t scan_response_type = report->scan_rsp;
  return (scan_response_type != 0);
}
bool BLEPeripheral::isScanAdvertise(ble_gap_evt_adv_report_t* report) {
  uint8_t scan_response_type = report->scan_rsp;
  return (scan_response_type == 0);
}
uint8_t* BLEPeripheral::getScanData(ble_gap_evt_adv_report_t* report) {
  return report->data;
}
uint8_t BLEPeripheral::getScanDataLen(ble_gap_evt_adv_report_t* report) {
  return report->dlen;
}

bool BLEPeripheral::isConnectable(ble_gap_evt_adv_report_t* report) {
  return (report->type == BLE_GAP_ADV_TYPE_ADV_IND || report->type == BLE_GAP_ADV_TYPE_ADV_DIRECT_IND);
}
int BLEPeripheral::getScanRSSI(ble_gap_evt_adv_report_t* report) {
  return report->rssi;
}

const uint8_t* BLEPeripheral::getScanAddress(ble_gap_evt_adv_report_t* report) {
  return (report->peer_addr.addr);
}


BLEPeripheral::BLEPeripheral(unsigned char req, unsigned char rdy, unsigned char rst) :
#if defined(NRF51) || defined(NRF52) || defined(__RFduino__)
  _nRF51822(),
#else
  _nRF8001(req, rdy, rst),
#endif

  _advertisedServiceUuid(NULL),
  _serviceSolicitationUuid(NULL),
  _manufacturerData(NULL),
  _manufacturerDataLength(0),
  _localAdvertName(NULL), // if this set local name sent in advert
  _localName(NULL), // if this sent local name sent in scan response

  _localAttributes(NULL),
  _numLocalAttributes(0),
  _remoteAttributes(NULL),
  _numRemoteAttributes(0),

  _genericAccessService("1800"),
  _deviceNameCharacteristic("2a00", BLERead, 19),
  _appearanceCharacteristic("2a01", BLERead, 2),
  _genericAttributeService("1801"),
  _servicesChangedCharacteristic("2a05", BLEIndicate, 4),

  _remoteGenericAttributeService("1801"),
  _remoteServicesChangedCharacteristic("2a05", BLEIndicate),

  _central(this)
{
#if defined(NRF51) || defined(NRF52) || defined(__RFduino__)
  this->_device = &this->_nRF51822;
#else
  this->_device = &this->_nRF8001;
#endif

  memset(this->_eventHandlers, 0x00, sizeof(this->_eventHandlers));

  this->setDeviceName(DEFAULT_DEVICE_NAME);
  this->setAppearance(DEFAULT_APPEARANCE);

  this->_device->setEventListener(this);
  pollInstance = this;
}

BLEPeripheral::~BLEPeripheral() {
  this->end();

  if (this->_remoteAttributes) {
    free(this->_remoteAttributes);
  }

  if (this->_localAttributes) {
    free(this->_localAttributes);
  }
  pollInstance = NULL;
}

void BLEPeripheral::setConnectedHandler(void(*connectHandler)(BLECentral& central)) {
	setEventHandler(BLEConnected, connectHandler);
}
void BLEPeripheral::setDisconnectedHandler(void(*disconnectHandler)(BLECentral& central)){
  setEventHandler(BLEDisconnected, disconnectHandler);
}
  
// returns false if failed
bool BLEPeripheral::startScanning(ble_scan_response_handler_t scan_response_handler) {
	return this->_device->startScanning(scan_response_handler);
}	

bool BLEPeripheral::stopScanning() {
	return this->_device->stopScanning(); // does not clear handler
}

void BLEPeripheral::setScanInterval(uint16_t interval, uint16_t window) {
	this->_device->setScanInterval(interval, window);       // in units of 0.625 ms,  default 160,40  i.e. 100ms,25ms
}
void BLEPeripheral::setActiveScan(bool enable) {
	// Request scan response data, default is false
	this->_device->setActiveScan(enable);       // in units of 0.625 ms,  default 160,40  i.e. 100ms,25ms
}
void BLEPeripheral::setScanTimeout(uint16_t timeout) {
	// 0 = Don't stop scanning after n seconds
	this->_device->setScanTimeout(timeout);       // in units of 0.625 ms,  default 160,40  i.e. 100ms,25ms
}

void BLEPeripheral::begin() {	
  unsigned char advertisementDataSize = 0;

  BLEEirData advertisementData[3];
  BLEEirData scanData;

  scanData.length = 0;

  unsigned char remainingAdvertisementDataLength = BLE_ADVERTISEMENT_DATA_MAX_VALUE_LENGTH + 2;
  if (this->_serviceSolicitationUuid){
    BLEUuid serviceSolicitationUuid = BLEUuid(this->_serviceSolicitationUuid);

    unsigned char uuidLength = serviceSolicitationUuid.length();
    advertisementData[advertisementDataSize].length = uuidLength;
    advertisementData[advertisementDataSize].type = (uuidLength > 2) ? 0x15 : 0x14;

    memcpy(advertisementData[advertisementDataSize].data, serviceSolicitationUuid.data(), uuidLength);
    advertisementDataSize += 1;
    remainingAdvertisementDataLength -= uuidLength + 2;
  }
  if (this->_advertisedServiceUuid){
    BLEUuid advertisedServiceUuid = BLEUuid(this->_advertisedServiceUuid);

    unsigned char uuidLength = advertisedServiceUuid.length();
    if (uuidLength + 2 <= remainingAdvertisementDataLength) {
      advertisementData[advertisementDataSize].length = uuidLength;
      advertisementData[advertisementDataSize].type = (uuidLength > 2) ? 0x06 : 0x02;

      memcpy(advertisementData[advertisementDataSize].data, advertisedServiceUuid.data(), uuidLength);
      advertisementDataSize += 1;
      remainingAdvertisementDataLength -= uuidLength + 2;
    }
  }
  if (this->_manufacturerData && this->_manufacturerDataLength > 0) {
    if (remainingAdvertisementDataLength >= 3) {
      unsigned char dataLength = this->_manufacturerDataLength;

      if (dataLength + 2 > remainingAdvertisementDataLength) {
        dataLength = remainingAdvertisementDataLength - 2;
      }

      advertisementData[advertisementDataSize].length = dataLength;
      advertisementData[advertisementDataSize].type = 0xff;

      memcpy(advertisementData[advertisementDataSize].data, this->_manufacturerData, dataLength);
      advertisementDataSize += 1;
      remainingAdvertisementDataLength -= dataLength + 2;
    }
  }
// THIS ADDS LOCAL NAME TO ADVERT PACKET 
  if (this->_localAdvertName){
    if (remainingAdvertisementDataLength >= 3) {
      unsigned char dataLength = strlen(this->_localAdvertName);

      if (dataLength + 2 > remainingAdvertisementDataLength) {
        dataLength = remainingAdvertisementDataLength - 2;
      }

      advertisementData[advertisementDataSize].length = dataLength;
      advertisementData[advertisementDataSize].type = (dataLength > dataLength) ? 0x08 : 0x09;

      memcpy(advertisementData[advertisementDataSize].data, this->_localAdvertName, dataLength);
      advertisementDataSize += 1;
      remainingAdvertisementDataLength -= dataLength + 2;
    }
  }
  
 // this adds LOCAL name to scan data  
  if (this->_localName){
    unsigned char localNameLength = strlen(this->_localName);
    scanData.length = localNameLength;

    if (scanData.length > BLE_SCAN_DATA_MAX_VALUE_LENGTH) {
      scanData.length = BLE_SCAN_DATA_MAX_VALUE_LENGTH;
    }

    scanData.type = (localNameLength > scanData.length) ? 0x08 : 0x09;

    memcpy(scanData.data, this->_localName, scanData.length);
  }
  
  if (this->_localAttributes == NULL) {
    this->initLocalAttributes();
  }

  for (int i = 0; i < this->_numLocalAttributes; i++) {
    BLELocalAttribute* localAttribute = this->_localAttributes[i];
    if (localAttribute->type() == BLETypeCharacteristic) {
      BLECharacteristic* characteristic = (BLECharacteristic*)localAttribute;

      characteristic->setValueChangeListener(*this);
    }
  }

  for (int i = 0; i < this->_numRemoteAttributes; i++) {
    BLERemoteAttribute* remoteAttribute = this->_remoteAttributes[i];
    if (remoteAttribute->type() == BLETypeCharacteristic) {
      BLERemoteCharacteristic* remoteCharacteristic = (BLERemoteCharacteristic*)remoteAttribute;

      remoteCharacteristic->setValueChangeListener(*this);
    }
  }

  if (this->_numRemoteAttributes) {
    this->addRemoteAttribute(this->_remoteGenericAttributeService);
    this->addRemoteAttribute(this->_remoteServicesChangedCharacteristic);
  }

  this->_device->begin(advertisementDataSize, advertisementData,
                        scanData.length > 0 ? 1 : 0, &scanData,
                        this->_localAttributes, this->_numLocalAttributes,
                        this->_remoteAttributes, this->_numRemoteAttributes);

  this->_device->requestAddress();
}

bool BLEPeripheral::poll() {
  return this->_device->poll();
}

void BLEPeripheral::end() {
  this->_device->end();
}

void BLEPeripheral::setAdvertisedServiceUuid(const char* advertisedServiceUuid) {
  this->_advertisedServiceUuid = advertisedServiceUuid;
}

void BLEPeripheral::setServiceSolicitationUuid(const char* serviceSolicitationUuid) {
  this->_serviceSolicitationUuid = serviceSolicitationUuid;
}

void BLEPeripheral::setManufacturerData(const unsigned char manufacturerData[], unsigned char manufacturerDataLength) {
  this->_manufacturerData = manufacturerData;
  this->_manufacturerDataLength = manufacturerDataLength;
}

void BLEPeripheral::setName(const char* localScanName) {
	setLocalName(localScanName);
}

void BLEPeripheral::setLocalName(const char* localScanName) {
  this->_localName = localScanName;
}

void BLEPeripheral::setAdvertisedName(const char *localAdvertName) { // set name in advert packet
  this->_localAdvertName = localAdvertName;
}

void BLEPeripheral::setConnectable(bool connectable) {
  this->_device->setConnectable(connectable);
}

bool  BLEPeripheral::setTxPower(int txPower) {
  return this->_device->setTxPower(txPower);
}

void BLEPeripheral::setBondStore(BLEBondStore& bondStore) {
  this->_device->setBondStore(bondStore);
}

void BLEPeripheral::setDeviceName(const char* deviceName) {
  this->_deviceNameCharacteristic.setValue(deviceName);
}

void BLEPeripheral::setAppearance(unsigned short appearance) {
  this->_appearanceCharacteristic.setValue((unsigned char *)&appearance, sizeof(appearance));
}

void BLEPeripheral::addAttribute(BLELocalAttribute& attribute) {
  this->addLocalAttribute(attribute);
}

void BLEPeripheral::addLocalAttribute(BLELocalAttribute& localAttribute) {
  if (this->_localAttributes == NULL) {
    this->initLocalAttributes();
  }

  this->_localAttributes[this->_numLocalAttributes] = &localAttribute;
  this->_numLocalAttributes++;
}

void BLEPeripheral::addRemoteAttribute(BLERemoteAttribute& remoteAttribute) {
  if (this->_remoteAttributes == NULL) {
    this->_remoteAttributes = (BLERemoteAttribute**)malloc(BLERemoteAttribute::numAttributes() * sizeof(BLERemoteAttribute*));
  }

  this->_remoteAttributes[this->_numRemoteAttributes] = &remoteAttribute;
  this->_numRemoteAttributes++;
}

void BLEPeripheral::setAdvertisingInterval(unsigned short advertisingInterval) {
  this->_device->setAdvertisingInterval(advertisingInterval);
}

void BLEPeripheral::setAdvertisingTimeout(uint16_t advertisingTimeout_secs) {
  this->_device->setAdvertisingTimeout(advertisingTimeout_secs);
}

bool BLEPeripheral::setAdvertising(bool on) {
	bool rtn = false;
	if (isConnected()) {
		return false;
	}
	if (on) {
		rtn = _device->startAdvertising();
	} else {
		rtn = (sd_ble_gap_adv_stop() == NRF_SUCCESS);
	}
	return rtn;
}

void BLEPeripheral::setConnectionInterval(unsigned short minimumConnectionInterval, unsigned short maximumConnectionInterval) {
  this->_device->setConnectionInterval(minimumConnectionInterval, maximumConnectionInterval);
}

void BLEPeripheral::setSlaveLatency(unsigned short slaveLatency) {
  this->_device->setSlaveLatency(slaveLatency);
}

void BLEPeripheral::disconnect() {
  this->_device->disconnect();
}

BLECentral BLEPeripheral::central() {
  this->poll();

  return this->_central;
}

bool BLEPeripheral::isConnected() {
	return connected();
}

bool BLEPeripheral::connected() {
  this->poll();

  return this->_central;
}

void BLEPeripheral::setEventHandler(BLEPeripheralEvent event, BLEPeripheralEventHandler eventHandler) {
  if (event < sizeof(this->_eventHandlers)) {
    this->_eventHandlers[event] = eventHandler;
  }
}

bool BLEPeripheral::characteristicValueChanged(BLECharacteristic& characteristic) {
  return this->_device->updateCharacteristicValue(characteristic);
}

bool BLEPeripheral::broadcastCharacteristic(BLECharacteristic& characteristic) {
  return this->_device->broadcastCharacteristic(characteristic);
}

bool BLEPeripheral::canNotifyCharacteristic(BLECharacteristic& characteristic) {
  return this->_device->canNotifyCharacteristic(characteristic);
}

bool BLEPeripheral::canIndicateCharacteristic(BLECharacteristic& characteristic) {
  return this->_device->canIndicateCharacteristic(characteristic);
}

bool BLEPeripheral::canReadRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  return this->_device->canReadRemoteCharacteristic(characteristic);
}

bool BLEPeripheral::readRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  return this->_device->readRemoteCharacteristic(characteristic);
}

bool BLEPeripheral::canWriteRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  return this->_device->canWriteRemoteCharacteristic(characteristic);
}

bool BLEPeripheral::writeRemoteCharacteristic(BLERemoteCharacteristic& characteristic, const unsigned char value[], unsigned char length) {
  return this->_device->writeRemoteCharacteristic(characteristic, value, length);
}

bool BLEPeripheral::canSubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  return this->_device->canSubscribeRemoteCharacteristic(characteristic);
}

bool BLEPeripheral::subscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  return this->_device->subscribeRemoteCharacteristic(characteristic);
}

bool BLEPeripheral::canUnsubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  return this->_device->canUnsubscribeRemoteCharacteristic(characteristic);
}

bool BLEPeripheral::unsubcribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic) {
  return this->_device->unsubcribeRemoteCharacteristic(characteristic);
}

void BLEPeripheral::BLEDeviceConnected(BLEDevice& /*device*/, const unsigned char* address) {
  this->_central.setAddress(address);

#ifdef BLE_PERIPHERAL_DEBUG
  Serial.print(F("Peripheral connected to central: "));
  Serial.println(this->_central.address());
#endif

  BLEPeripheralEventHandler eventHandler = this->_eventHandlers[BLEConnected];
  if (eventHandler) {
    eventHandler(this->_central);
  }
}

void BLEPeripheral::BLEDeviceDisconnected(BLEDevice& /*device*/) {
#ifdef BLE_PERIPHERAL_DEBUG
  Serial.print(F("Peripheral disconnected from central: "));
  Serial.println(this->_central.address());
#endif

  BLEPeripheralEventHandler eventHandler = this->_eventHandlers[BLEDisconnected];
  if (eventHandler) {
    eventHandler(this->_central);
  }

  this->_central.clearAddress();
}

void BLEPeripheral::BLEDeviceBonded(BLEDevice& /*device*/) {
#ifdef BLE_PERIPHERAL_DEBUG
  Serial.print(F("Peripheral bonded: "));
  Serial.println(this->_central.address());
#endif

  BLEPeripheralEventHandler eventHandler = this->_eventHandlers[BLEBonded];
  if (eventHandler) {
    eventHandler(this->_central);
  }
}

void BLEPeripheral::BLEDeviceRemoteServicesDiscovered(BLEDevice& /*device*/) {
#ifdef BLE_PERIPHERAL_DEBUG
  Serial.print(F("Peripheral discovered central remote services: "));
  Serial.println(this->_central.address());
#endif

  BLEPeripheralEventHandler eventHandler = this->_eventHandlers[BLERemoteServicesDiscovered];
  if (eventHandler) {
    eventHandler(this->_central);
  }
}

void BLEPeripheral::BLEDeviceCharacteristicValueChanged(BLEDevice& /*device*/, BLECharacteristic& characteristic, const unsigned char* value, unsigned char valueLength) {
  characteristic.setValue(this->_central, value, valueLength);
}

void BLEPeripheral::BLEDeviceCharacteristicSubscribedChanged(BLEDevice& /*device*/, BLECharacteristic& characteristic, bool subscribed) {
  characteristic.setSubscribed(this->_central, subscribed);
}

void BLEPeripheral::BLEDeviceRemoteCharacteristicValueChanged(BLEDevice& /*device*/, BLERemoteCharacteristic& remoteCharacteristic, const unsigned char* value, unsigned char valueLength) {
  remoteCharacteristic.setValue(this->_central, value, valueLength);
}

void BLEPeripheral::BLEDeviceAddressReceived(BLEDevice& /*device*/, const unsigned char* /*address*/) {
#ifdef BLE_PERIPHERAL_DEBUG
  char addressStr[18];

  BLEUtil::addressToString(address, addressStr);

  Serial.print(F("Peripheral address: "));
  Serial.println(addressStr);
#endif
}

void BLEPeripheral::BLEDeviceTemperatureReceived(BLEDevice& /*device*/, float /*temperature*/) {
}

void BLEPeripheral::BLEDeviceBatteryLevelReceived(BLEDevice& /*device*/, float /*batteryLevel*/) {
}

void BLEPeripheral::initLocalAttributes() {
  this->_localAttributes = (BLELocalAttribute**)malloc(BLELocalAttribute::numAttributes() * sizeof(BLELocalAttribute*));

  this->_localAttributes[0] = &this->_genericAccessService;
  this->_localAttributes[1] = &this->_deviceNameCharacteristic;
  this->_localAttributes[2] = &this->_appearanceCharacteristic;

  this->_localAttributes[3] = &this->_genericAttributeService;
  this->_localAttributes[4] = &this->_servicesChangedCharacteristic;

  this->_numLocalAttributes = 5;
}
