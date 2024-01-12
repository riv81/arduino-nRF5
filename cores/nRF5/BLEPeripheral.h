// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef _BLE_PERIPHERAL_H_
#define _BLE_PERIPHERAL_H_

#include "bleConstants.h"

#include "Arduino.h"

#include "BLEBondStore.h"
#include "BLECentral.h"
#include "BLEConstantCharacteristic.h"
#include "BLEDescriptor.h"
#include "BLEDevice.h"
#include "BLEFixedLengthCharacteristic.h"
#include "BLELocalAttribute.h"
#include "BLEProgmemConstantCharacteristic.h"
#include "BLERemoteAttribute.h"
#include "BLERemoteCharacteristic.h"
#include "BLERemoteService.h"
#include "BLEService.h"
#include "BLETypedCharacteristics.h"

#if defined(NRF51) || defined(NRF52) || defined(__RFduino__)
  #include "nRF51822.h"
#else
  #include "nRF8001.h"
#endif

#if defined(NRF51) || defined(NRF52) || defined(__RFduino__)
  #define BLE_DEFAULT_REQ   -1
  #define BLE_DEFAULT_RDY   -1
  #define BLE_DEFAULT_RST   -1
#elif defined(BLEND_MICRO)
  #define BLE_DEFAULT_REQ   6
  #define BLE_DEFAULT_RDY   7
  #define BLE_DEFAULT_RST   4
#elif defined(BLEND)
  #define BLE_DEFAULT_REQ   9
  #define BLE_DEFAULT_RDY   8
  #define BLE_DEFAULT_RST   4
#else
  #define BLE_DEFAULT_REQ   10
  #define BLE_DEFAULT_RDY   2
  #define BLE_DEFAULT_RST   9
#endif


enum BLEPeripheralEvent {
  BLEConnected = 0,
  BLEDisconnected = 1,
  BLEBonded = 2,
  BLERemoteServicesDiscovered = 3
};

typedef void (*BLEPeripheralEventHandler)(BLECentral& central);

class BLEPeripheral : public BLEDeviceEventListener,
                        public BLECharacteristicValueChangeListener,
                        public BLERemoteCharacteristicValueChangeListener
{
  public:
    BLEPeripheral(unsigned char req = BLE_DEFAULT_REQ, unsigned char rdy = BLE_DEFAULT_RDY, unsigned char rst = BLE_DEFAULT_RST);
    virtual ~BLEPeripheral();
    
    static BLEPeripheral* pollInstance; // used for polling if not NULL
    
    static const size_t BLE_ADDR_LEN = 6;
    static const size_t BLE_ADV_LEN = 31;
    static uint8_t* parseScanDataByType(const uint8_t* scandata, uint8_t scanlen, uint8_t type, uint8_t& lenWritten);
    static uint8_t* parseScanDataByType(const ble_gap_evt_adv_report_t* report, uint8_t type, uint8_t& lenWritten);
    static bool isScanResponse(ble_gap_evt_adv_report_t* report);
    static bool isScanAdvertise(ble_gap_evt_adv_report_t* report);
    static uint8_t* getScanData(ble_gap_evt_adv_report_t* report);
    static uint8_t getScanDataLen(ble_gap_evt_adv_report_t* report);
    static bool isConnectable(ble_gap_evt_adv_report_t* report);
    static int getScanRSSI(ble_gap_evt_adv_report_t* report);
    static const uint8_t* getScanAddress(ble_gap_evt_adv_report_t* report);


    void begin();  // this starts advertising so need to call stopAdvertising() if needed
    virtual bool poll(); // use sub-class implementation if any
    void end();
    
    virtual void setConnectedHandler(void(*connectHandler)(BLECentral& central));
    virtual void setDisconnectedHandler(void(*disconnectHandler)(BLECentral& central));
    virtual bool isConnected();
    
    // NOTE: scanning is a high current process, while running the nRF52832 uses ~3mA
    virtual bool startScanning(ble_scan_response_handler_t scan_response_handler); // returns false if failed
    /* Function for stopping the scanning.
    * Note This functions returns immediately, but the scanning is actually stopped after the next radio slot.
    */
    virtual bool stopScanning(); // does not clear handler
    
    // update these while not scanning and then call startScanning() to apply them 
    virtual void setScanInterval(uint16_t interval, uint16_t window);       // in units of 0.625 ms,  default 160,40  i.e. 100ms,25ms
    virtual void setActiveScan(bool enable);        // Request scan response data, default is false
    virtual void setScanTimeout(uint16_t timeout_sec);   //NOTE: this timeout is in seconds!! // 0 = Don't stop scanning after n seconds

    void setAdvertisedServiceUuid(const char* advertisedServiceUuid);
    void setServiceSolicitationUuid(const char* serviceSolicitationUuid);
    void setManufacturerData(const unsigned char manufacturerData[], unsigned char manufacturerDataLength);
    void setAdvertisedName(const char *localAdvertName); // set name in advert packet
    virtual void setLocalName(const char *localScanName); // this sets scan packet
    virtual void setName(const char *localScanName); // this sets scan packet

    // setAdvertisingTimeout in sec
    // e.g. to set 2s advertising interval use
    // setAdvertisingTimeout(2);
    // and then setAdvertising(true) to start for 2sec then stop
    // setAdvertisingTimeout(0); // never turns off advertising
    void setAdvertisingTimeout(uint16_t advertisingTimeout_secs);

    // call setAdvertising(true) to start advertising with current advertisingTimeout
    // call setAdvertising(false) to stop advertising regardless
    bool setAdvertising(bool turnOn);

    bool stopAdvertising() { return setAdvertising(false); }
    bool startAdvertising() { return setAdvertising(true); }

    // setAdvertisingInterval in ms
    // e.g. to set 100ms advertising interval use
    // setAdvertisingInterval(100);
    // must be in the range 20ms to 10240ms (10.24sec)
    // default is 500ms
    void setAdvertisingInterval(unsigned short advertisingInterval);

    // must be between 8ms and 4000ms
    // internally stored in 1.25 ms increments, but this method expects ms arguments
    // e.g. to set 50ms min, 100ms max connection intervals use
    // setConnectionInterval(50,100);
    // must be between 8ms and 4000ms
    // defaults are 100ms min and 150ms max
    void setConnectionInterval(unsigned short minimumConnectionInterval, unsigned short maximumConnectionInterval);
    
    // must be in the range 
    // 0 to ((4000 / connectionInterval_ms) ). 
    // i.e. must respond within supervision timeout even if no data
    // default connectionSupervisionTimeout is 4sec see bleConstants.h
    // default is 0
    void setSlaveLatency(unsigned short slaveLatency);
    
    // default txPower is +4 (maximum)
    // available settings -40, -30, -20, -16, -12, -8, -4, 0, 4
    bool setTxPower(int txPower);
    
    // default true (connectable)
    void setConnectable(bool connectable);
    void setBondStore(BLEBondStore& bondStore);


    // default deviceName "Arduino"
    void setDeviceName(const char* deviceName);
    void setAppearance(unsigned short appearance);

    void addAttribute(BLELocalAttribute& attribute);
    void addLocalAttribute(BLELocalAttribute& localAttribute);
    void addRemoteAttribute(BLERemoteAttribute& remoteAttribute);

    void disconnect();
    
    BLECentral central();
    bool connected();

    void setEventHandler(BLEPeripheralEvent event, BLEPeripheralEventHandler eventHandler);

  protected:
    bool characteristicValueChanged(BLECharacteristic& characteristic);
    bool broadcastCharacteristic(BLECharacteristic& characteristic);
    bool canNotifyCharacteristic(BLECharacteristic& characteristic);
    bool canIndicateCharacteristic(BLECharacteristic& characteristic);

    bool canReadRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    bool readRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    bool canWriteRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    bool writeRemoteCharacteristic(BLERemoteCharacteristic& characteristic, const unsigned char value[], unsigned char length);
    bool canSubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    bool subscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    bool canUnsubscribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);
    bool unsubcribeRemoteCharacteristic(BLERemoteCharacteristic& characteristic);

    virtual void BLEDeviceConnected(BLEDevice& device, const unsigned char* address);
    virtual void BLEDeviceDisconnected(BLEDevice& device);
    virtual void BLEDeviceBonded(BLEDevice& device);
    virtual void BLEDeviceRemoteServicesDiscovered(BLEDevice& device);

    virtual void BLEDeviceCharacteristicValueChanged(BLEDevice& device, BLECharacteristic& characteristic, const unsigned char* value, unsigned char valueLength);
    virtual void BLEDeviceCharacteristicSubscribedChanged(BLEDevice& device, BLECharacteristic& characteristic, bool subscribed);

    virtual void BLEDeviceRemoteCharacteristicValueChanged(BLEDevice& device, BLERemoteCharacteristic& remoteCharacteristic, const unsigned char* value, unsigned char valueLength);

    virtual void BLEDeviceAddressReceived(BLEDevice& device, const unsigned char* address);
    virtual void BLEDeviceTemperatureReceived(BLEDevice& device, float temperature);
    virtual void BLEDeviceBatteryLevelReceived(BLEDevice& device, float batteryLevel);

  private:
    void initLocalAttributes();

  protected:
    BLEDevice*                     _device;
    
  private:

#if defined(NRF51) || defined(NRF52) || defined(__RFduino__)
    nRF51822                       _nRF51822;
#else
    nRF8001                        _nRF8001;
#endif

    const char*                    _advertisedServiceUuid;
    const char*                    _serviceSolicitationUuid;
    const unsigned char*           _manufacturerData;
    unsigned char                  _manufacturerDataLength;
    const char*                    _localName; // if this set localname sent in scan response
    const char*                    _localAdvertName; // if this set local name sent in advert

    BLELocalAttribute**            _localAttributes;
    unsigned char                  _numLocalAttributes;
    BLERemoteAttribute**           _remoteAttributes;
    unsigned char                  _numRemoteAttributes;

    BLEService                     _genericAccessService;
    BLECharacteristic              _deviceNameCharacteristic;
    BLECharacteristic              _appearanceCharacteristic;
    BLEService                     _genericAttributeService;
    BLECharacteristic              _servicesChangedCharacteristic;

    BLERemoteService               _remoteGenericAttributeService;
    BLERemoteCharacteristic        _remoteServicesChangedCharacteristic;

    BLECentral                     _central;
    BLEPeripheralEventHandler      _eventHandlers[4];
};

#endif
