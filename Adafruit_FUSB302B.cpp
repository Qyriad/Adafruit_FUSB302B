#include "Adafruit_FUSB302B.h"

#define REG_DEVICE_ID (0x01)

const uint8_t DEVICE_ADDR = 0x22;

FUSB302B_DeviceId::FUSB302B_DeviceId() {
  deviceId = 0;
  versionId = 0;
  productId = 0;
  revisionId = 0;
}

bool Adafruit_FUSB302B::begin() {
  _i2c = &Wire;
  _i2cDev = new Adafruit_I2CDevice(
    DEVICE_ADDR, // addr
    _i2c
  );

  if (!_i2cDev->begin()) {
    return false;
  }


  _deviceId = getDeviceId();
  // The FUSB302B has a bunch of valid device IDs that could be returned here.
  // To check if this one is valid, we'll check that it's not 0xFF or 0x00.
  if (_deviceId.deviceId == 0 || _deviceId.deviceId == 0xFF) {
    return false;
  }


  return true;
}

FUSB302B_DeviceId Adafruit_FUSB302B::getDeviceId() {

  Adafruit_BusIO_Register idReg = Adafruit_BusIO_Register(
    _i2cDev,
    0x01 // register address
  );

  FUSB302B_DeviceId deviceId = FUSB302B_DeviceId();
  if (!idReg.read(&deviceId.deviceId)) {
    Serial.print("Failed with 0x");
    Serial.println(deviceId.deviceId);
    return deviceId;
  }

  Adafruit_BusIO_RegisterBits versionId = Adafruit_BusIO_RegisterBits(&idReg, 4, 4);
  Adafruit_BusIO_RegisterBits productId = Adafruit_BusIO_RegisterBits(&idReg, 2, 2);
  Adafruit_BusIO_RegisterBits revisionId = Adafruit_BusIO_RegisterBits(&idReg, 2, 0);

  deviceId.versionId = versionId.read();
  deviceId.productId = productId.read();
  deviceId.revisionId = revisionId.read();

  return deviceId;
}
