// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef _BLE_DEVICE_H_
#define _BLE_DEVICE_H_

#include "BLEBondStore.h"
#include "BLECharacteristic.h"
#include "BLELocalAttribute.h"
#include "BLERemoteAttribute.h"
#include "BLERemoteCharacteristic.h"
#include "BLERemoteService.h"

#include "SDK\components\softdevice\s132\headers\ble_gap.h"
/** Scanner response handler. 
*/
typedef void (*ble_scan_response_handler_t) (ble_gap_evt_adv_report_t* report);

struct BLEEirData
{
  unsigned char length;
  unsigned char type;
  unsigned char data[BLE_EIR_DATA_MAX_VALUE_LENGTH];
};

class BLEDevice;

class BLEDeviceEventListener
{
  public:
    virtual void BLEDeviceConnected(BLEDevice& /*device*/, const unsigned char* /*address*/) { }
    virtual void BLEDeviceDisconnected(BLEDevice& /*device*/) { }
    virtual void BLEDeviceBonded(BLEDevice& /*device*/) { }
    virtual void BLEDeviceRemoteServicesDiscovered(BLEDevice& /*device*/) { }

    virtual void BLEDeviceCharacteristicValueChanged(BLEDevice& /*device*/, BLECharacteristic& /*characteristic*/, const unsigned char* /*value*/, unsigned char /*valueLength*/) { }
    virtual void BLEDeviceCharacteristicSubscribedChanged(BLEDevice& /*device*/, BLECharacteristic& /*characteristic*/, bool /*subscribed*/) { }

    virtual void BLEDeviceRemoteCharacteristicValueChanged(BLEDevice& /*device*/, BLERemoteCharacteristic& /*characteristic*/, const unsigned char* /*value*/, unsigned char /*valueLength*/) { }


    virtual void BLEDeviceAddressReceived(BLEDevice& /*device*/, const unsigned char* /*address*/) { }
    virtual void BLEDeviceTemperatureReceived(BLEDevice& /*device*/, float /*temperature*/) { }
    virtual void BLEDeviceBatteryLevelReceived(BLEDevice& /*device*/, float /*batteryLevel*/) { }
};


class BLEDevice
{

  friend class BLEPeripheral;

  protected:
    BLEDevice();


    virtual ~BLEDevice();

    virtual bool startScanning(ble_scan_response_handler_t scanResponseHandler) {return false;} // not supported
    virtual bool stopScanning() {return false;} // not supported
    // update these while not scanning and then call startScanning() to apply them 
    virtual void setScanInterval(uint16_t interval, uint16_t window) {}       // in units of 0.625 ms,  default 160,40  i.e. 100ms,25ms
    virtual void setActiveScan(bool enable) {}        // Request scan response data, default is false
    virtual void setScanTimeout(uint16_t timeout) {};   // 0 = Don't stop scanning after n seconds

    void setEventListener(BLEDeviceEventListener* eventListener);

    void setAdvertisingTimeout(uint16_t timeout_in_secs); // 0 means never timeout
    void setAdvertisingInterval(unsigned short advertisingInterval);
    void setConnectionInterval(unsigned short minimumConnectionInterval, unsigned short maximumConnectionInterval);
    void setSlaveLatency(unsigned short slaveLatency);
    void setConnectable(bool connectable);
    void setBondStore(BLEBondStore& bondStore);

    virtual void begin(unsigned char /*advertisementDataSize*/,
                BLEEirData * /*advertisementData*/,
                unsigned char /*scanDataSize*/,
                BLEEirData * /*scanData*/,
                BLELocalAttribute** /*localAttributes*/,
                unsigned char /*numLocalAttributes*/,
                BLERemoteAttribute** /*remoteAttributes*/,
                unsigned char /*numRemoteAttributes*/) { }
    
    virtual bool poll() {return true;}

    virtual void end() { }

    virtual bool setTxPower(int /*txPower*/) { return false; }

    virtual void disconnect() { }

    virtual bool updateCharacteristicValue(BLECharacteristic& /*characteristic*/) { return false; }
    virtual bool broadcastCharacteristic(BLECharacteristic& /*characteristic*/) { return false; }
    virtual bool canNotifyCharacteristic(BLECharacteristic& /*characteristic*/) { return false; }
    virtual bool canIndicateCharacteristic(BLECharacteristic& /*characteristic*/) { return false; }

    virtual bool canReadRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }
    virtual bool readRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }
    virtual bool canWriteRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }
    virtual bool writeRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/, const unsigned char /*value*/[], unsigned char /*length*/) { return false; }
    virtual bool canSubscribeRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }
    virtual bool subscribeRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }
    virtual bool canUnsubscribeRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }
    virtual bool unsubcribeRemoteCharacteristic(BLERemoteCharacteristic& /*characteristic*/) { return false; }

    virtual void requestAddress() { }
    virtual void requestTemperature() { }
    virtual void requestBatteryLevel() { }

  public:
   virtual bool startAdvertising() {return true;}
	unsigned short getAdvertisingInterval(); // in ms
    unsigned short getMaximumConnectionInterval(); // in ms
    bool getConnectable();
    
  protected:
  	ble_scan_response_handler_t    _scanResponseHandler;
    unsigned short                _advertisingInterval; // in ms
    uint16_t                      _advertisingTimeout; // in seconds 0 to 65535
    unsigned short                _minimumConnectionInterval; // in ms
    unsigned short                _maximumConnectionInterval; // in ms
    unsigned short                _slaveLatency;
    bool                          _connectable;

    BLEBondStore*                 _bondStore;
    BLEDeviceEventListener*       _eventListener;
};

#endif
