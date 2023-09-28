#include <assert.h>

#include "Adafruit_FUSB302B.h"

#define REG_DEVICE_ID (0x01)

// Used to manipulate currents and resistors on the CC lines.
#define REG_SWITCHES0 (0x02)

#define REG_SWITCHES1 (0x03)

// This register is 8 bits wide.
#define REG_FIFO      (0x43)

#define REG_RESET     (0x0C)

#define TOKEN_TXON    (0xA1)
#define TOKEN_SOP1    (0x12)
#define TOKEN_SOP2    (0x13)
#define TOKEN_SOP3    (0x1B)
#define TOKEN_RESET1  (0x15)
#define TOKEN_RESET2  (0x16)
#define TOKEN_PACKSYM (0x80)
#define TOKEN_JAM_CRC (0xFF)
#define TOKEN_EOP     (0x14)
#define TOKEN_TXOFF   (0xFE)

const uint8_t DEVICE_ADDR = 0x22;

FUSB302B_DeviceId::FUSB302B_DeviceId() {
  deviceId = 0;
  versionId = 0;
  productId = 0;
  revisionId = 0;
}

/**
 * @brief SOP packets (in contrast to SOP' and SOP'' packets) are for communicating to and from
 * entities at the ends of a USB connection (in contrast to communicating with the cable itself).
 */
struct PD_SOP_Header {
  // Bit 15.
  uint16_t extended : 1;

  // Bits 14..12
  uint16_t data_objects_count : 3;

  // Bits 11...9
  uint16_t message_id : 3;

  // Bit 8
  /** @brief Indicates the Port's current power role: `0` for Sink, `1` for Source. */
  uint16_t port_power_role : 1;

  // Bits 7...6
  /** @brief Should always be `0b01`, to indicate USB PD R2.
   *
   * R1 is the "USB Battery Charging" (BC) spec, which predates USB-C and saw little use,
   * and the FUSB302B is not aware of R3.
   */
  uint16_t spec_rev : 2;

  // Bit 5
  /** @brief Indicates the Port's current data role: `0` for UFP, `0` for DFP. */
  uint16_t port_data_role : 1;

  // Bits 4...0
  uint16_t message_type: 5;
};

struct PD_SOPP_Header {
  // Bit 15.
  /** @brief Set to 0 for a Control Message or Data Message; set to 1 for an Extended Message. */
  uint16_t extended : 1;

  // Bits 14..12
  /**
   * When @ref extended is `0`, this indicates the number of 32-bit Data Objects after this header.
   * `0` indicates a Control Message, as Control Messages do not have any Data Objects.
   */
  // TODO: extended message header
  uint16_t data_objects_count : 3;

  // Bits 11...9
  /**
   * Starts at 0; incremented by the Message originator when a Message is replied to with GoodCRC.
   */
  uint16_t message_id : 3;

  // Bit 8
  /** I think this should always be `0`, as we are not a Vconn-powered device (VPD). */
  uint16_t cable_plug : 1;

  // Bits 7...6
  uint16_t spec_rev : 2;

  // Bit 5
  uint16_t __reserved : 1;

  // Bits 4...0
  uint16_t message_type: 5;
};

union PD_MessageHeader {
  PD_SOP_Header sop;
  PD_SOPP_Header sop_prime;
  uint16_t value;
};

static_assert(sizeof(PD_MessageHeader) == 2, "PD_MessageHeader is wrong size (miscompilation?)");

bool Adafruit_FUSB302B::begin(FUSB302B_PowerRole powerMode) {
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

  Serial.println("FUSB302B: resetting..."); // Qyriad
  // Alright to start, let's reset the device, just in case.
  Adafruit_BusIO_Register resetReg(_i2cDev, REG_RESET);
  Adafruit_BusIO_RegisterBits softwareReset(&resetReg, /*width*/ 1, /*shift*/ 0);
  softwareReset.write(1);

  // And check the device ID again.
  FUSB302B_DeviceId newDeviceId = getDeviceId();
  if (newDeviceId.deviceId != _deviceId.deviceId) {
    Serial.println("FUSB302B: chip ID changed after reset!"); // Qyriad
    Serial.print("Before: 0x");
    Serial.print(_deviceId.deviceId, HEX);
    Serial.print("After: 0x");
    Serial.println(newDeviceId.deviceId, HEX);
    return false;
  }

  Serial.println("Reset complete"); // Qyriad


  if (powerMode == POWER_SOURCE) {
    // Disable device pull-down resistors (Rd) on CC1 and CC2, and
    // apply host pull-up currents (Ip) to CC1 and CC2.
    // USB Type-C R2.2 § 4.5.1.2.1


  } else if (powerMode == POWER_SINK) {
    // Enable device pull-down resistors (Rd) on CC1 and CC2,
    // and disable host pull-up currents (Ip) on CC1 and CC2.
    // USB Type-C R2.2 § 4.5.1.2.1

    Serial.println("FUSB302B: POWER SINK mode is not yet supported!");
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
