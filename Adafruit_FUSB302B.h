/**
 * @file Adafruit_FUSB302B.h
 *
 * @mainpage Onsemi FUSB302B USB-C PowerDelivery Controller Library
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>

enum FUSB302B_VersionId {
  FUSB302B_A = 0b1000,
  FUSB302B_B = 0b1001,
  FUSB302B_C = 0b1010,
};

class FUSB302B_DeviceId {

public:

  FUSB302B_DeviceId();

  /**
   * @brief The full I2C device ID byte, unmodified.
  */
  uint8_t deviceId;

  /**
   * @brief The part of the I2C device ID that indicates this device's version.
   *
   * According to the datasheet, the following values correspond to the following device version:
   * 0b1000: A
   * 0b1001: B
   * 0b1010: C
   */
  uint8_t versionId;

  /**
   * @brief The part of the I2C device ID that indicates which exact product this is.
   *
   * According to the datasheet, the following values correspond to the following products:
   * 0b00: FUSB302BMPX, FUSB302BVMPX, or FUSB302BUCX.
   * 0b01: FUSB302B01MPX
   * 0b10: FUSB302B10MPX
   * 0b11: FUSB302B11MPX
   */
  uint8_t productId;

  /**
   * @brief The part of the I2C device ID that indicates what revision of the device version specified in
   * @ref deviceId.
   *
   * According to the datasheet, the following values correspond to the following revisions:
   * 0b00: revA
   * 0b01: revB
   * 0b10: revC
   * 0b11: revD
   */
  uint8_t revisionId;
};


class Adafruit_FUSB302B {
public:
  bool begin();

  FUSB302B_DeviceId getDeviceId();

private:
  Adafruit_I2CDevice *_i2cDev;
  TwoWire *_i2c;

  FUSB302B_DeviceId _deviceId;

};
