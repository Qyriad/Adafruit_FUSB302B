#include <assert.h>
#include <cstdint>

#include "Adafruit_BusIO_Register.h"
#include "Adafruit_FUSB302B.h"
#include "Arduino.h"

#define REG_DEVICE_ID (0x01)

// Used to manipulate currents and resistors on the CC lines.
#define REG_SWITCHES0 (0x02)

#define REG_SWITCHES1 (0x03)

#define REG_MEASURE   (0x04)

#define REG_CONTROL0  (0x06)

// No current
#define CONTROL0_HOST_CURRENT_NONE (0x00)
/// 80 uA (microamperes) — the default USB power.
#define CONTROL0_HOST_CURRENT_80uA (0x01)

#define REG_CONTROL1  (0x07)
#define REG_CONTROL2  (0x08)
#define REG_CONTROL3  (0x09)

#define REG_POWER     (0x0b)

#define REG_STATUS0   (0x40)
#define REG_STATUS1   (0x41)

// This register is 8 bits wide.
#define REG_FIFO      (0x43)

#define REG_MASK      (0x0A)

#define REG_RESET     (0x0C)

#define REG_MASKA     (0x0E)
#define REG_MASKB     (0x0F)
#define REG_INTERRUPTA (0x3E)
#define REG_INTERRUPTB (0x3F)
#define REG_INTERRUPT  (0x42)

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

/** The MDAC value for 1.60 V, the vRd threshold.
 *
 * (37 * 42 mV) + 42 mV = 1.596 V.
 */
const uint32_t THRESHOLD_vRd = 37;

/** The MDAC value for 0.2 V, the vRa threshold.
 *
 * (4 * 42 mV) + 42 mV = 0.21 V.
 */
const uint32_t THRESHOLD_vRa = 4;

inline bool matchesMask(uint8_t byte, uint8_t mask) {
  return (byte & mask) == mask;
}

FUSB302B_DeviceId::FUSB302B_DeviceId() {
  deviceId = 0;
  versionId = 0;
  productId = 0;
  revisionId = 0;
}

bool SourcePortState::isConnected() {
  return connection != CONNECTION_NONE;
}

bool SinkPortState::isConnected() {
  return currentAdvertisement != CURRENT_NONE;
}

Adafruit_FUSB302B::Adafruit_FUSB302B(TwoWire *wire) {
  _i2c = wire;
  _i2cDev = new Adafruit_I2CDevice(
    DEVICE_ADDR, // addr
    _i2c
  );

  if (!_i2cDev->begin(false)) {
    Serial.println("Could not initialize I2C bus");
    while (true) { }
  }


  _deviceId = getDeviceId();
  // The FUSB302B has a bunch of valid device IDs that could be returned here.
  // To check if this one is valid, we'll check that it's not 0xFF or 0x00.
  if (_deviceId.deviceId == 0 || _deviceId.deviceId == 0xFF) {
    Serial.print("Got invalid device ID 0x");
    Serial.println(_deviceId.deviceId, HEX);
    while (true) { }
  }

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
    while (true) { }
  }

  // Setup registers.
  _regSwitches0     = new Adafruit_BusIO_Register(_i2cDev, REG_SWITCHES0);
  _switches0Pdwn1   = new Adafruit_BusIO_RegisterBits(_regSwitches0, 1, 0);
  _switches0Pdwn2   = new Adafruit_BusIO_RegisterBits(_regSwitches0, 1, 1);
  _switches0Meascc1 = new Adafruit_BusIO_RegisterBits(_regSwitches0, 1, 2);
  _switches0Meascc2 = new Adafruit_BusIO_RegisterBits(_regSwitches0, 1, 3);
  _switches0Puen1 = new Adafruit_BusIO_RegisterBits(_regSwitches0, 1, 6);
  _switches0Puen2 = new Adafruit_BusIO_RegisterBits(_regSwitches0, 1, 7);

  _regMeasure = new Adafruit_BusIO_Register(_i2cDev, REG_MEASURE);
  _measureMdac = new Adafruit_BusIO_RegisterBits(_regMeasure, 6, 0);
  _measureMeasVbus = new Adafruit_BusIO_RegisterBits(_regMeasure, 1, 6);

  _regControl0 = new Adafruit_BusIO_Register(_i2cDev, REG_CONTROL0);
  _control0HostCur = new Adafruit_BusIO_RegisterBits(_regControl0, 2, 2);
  _control0IntMask = new Adafruit_BusIO_RegisterBits(_regControl0, 1, 5);

  _regControl2 = new Adafruit_BusIO_Register(_i2cDev, REG_CONTROL2);
  _control2Toggle = new Adafruit_BusIO_RegisterBits(_regControl2, 1, 0);
  _control2Mode = new Adafruit_BusIO_RegisterBits(_regControl2, 2, 2); // TODO: shift by 2 right?

  _regPower = new Adafruit_BusIO_Register(_i2cDev, REG_POWER);
  _powerBandgapWake = new Adafruit_BusIO_RegisterBits(_regPower, 1, 0);
  _powerReceiverCurref = new Adafruit_BusIO_RegisterBits(_regPower, 1, 1);
  _powerMeasureBlock = new Adafruit_BusIO_RegisterBits(_regPower, 1, 2);
  _powerInternalOsc = new Adafruit_BusIO_RegisterBits(_regPower, 1, 3);

  _regMask = new Adafruit_BusIO_Register(_i2cDev, REG_MASK);

  _regReset = new Adafruit_BusIO_Register(_i2cDev, REG_RESET);

  _regMaska = new Adafruit_BusIO_Register(_i2cDev, REG_MASKA);
  _regMaskb = new Adafruit_BusIO_Register(_i2cDev, REG_MASKB);

  _resetSw = new Adafruit_BusIO_RegisterBits(_regReset, 1, 0);

  _regStatus0 = new Adafruit_BusIO_Register(_i2cDev, REG_STATUS0);
  _status0BcLvl = new Adafruit_BusIO_RegisterBits(_regStatus0, 2, 0);
  _status0Comp = new Adafruit_BusIO_RegisterBits(_regStatus0, 1, 5);
  _status0Vbusok = new Adafruit_BusIO_RegisterBits(_regStatus0, 1, 7);

  _regInterrupta = new Adafruit_BusIO_Register(_i2cDev, REG_INTERRUPTA);
  _regInterruptb = new Adafruit_BusIO_Register(_i2cDev, REG_INTERRUPTB);
  _regInterrupt = new Adafruit_BusIO_Register(_i2cDev, REG_INTERRUPT);

  // Reset the device.
  _resetSw->write(1);
  delay(100); // FIXME: determine better delay.
  _resetSw->write(0);
  delay(100); // FIXME: determine better delay.

  // Datasheet says we should read all interrupt register bits to clear them before we enable toggling.
}

SourcePortState Adafruit_FUSB302B::beginSource(CurrentAdvertisement advertisedCurrent) {

  // Power on stuff!
  _powerBandgapWake->write(1);
  _powerReceiverCurref->write(1);
  _powerMeasureBlock->write(1);
  _powerInternalOsc->write(1);

  // Disable the device pull-downs on CC1 and CC2.
  _switches0Pdwn1->write(0);
  _switches0Pdwn2->write(0);


  // Alright, let's do detection.

  // Make sure everything's pull-ups, pull-downs, and measurements are all disabled first.
  _switches0Pdwn1->write(0);
  _switches0Puen1->write(0);
  _switches0Meascc1->write(0);
  _switches0Pdwn2->write(0);
  _switches0Puen2->write(0);
  _switches0Meascc2->write(0);

  // When we do enable pull-ups (in determineCCState), use the requested current.
  _control0HostCur->write(advertisedCurrent);


  CCState cc1State = determineCCState(CC1);
  CCState cc2State = determineCCState(CC2);

  switch (cc1State) {
    case CCOpen:
      switch (cc2State) {
        case CCOpen:
          return SourcePortState(CONNECTION_NONE);

        case CCRd:
          return SourcePortState(CONNECTION_SINK, CABLE_FLIPPED);

        case CCRa:
          return SourcePortState(CONNECTION_POWERED_CABLE_NO_SINK, CABLE_FLIPPED);
      }
      break;

    case CCRd:
      switch (cc2State) {
        case CCOpen:
          return SourcePortState(CONNECTION_SINK, CABLE_NOT_FLIPPED);

        case CCRd:
          return SourcePortState(CONNECTION_DEBUG_ACCESSORY);

        case CCRa:
          return SourcePortState(CONNECTION_VCONN, CABLE_NOT_FLIPPED);
      }
      break;

    case CCRa:
      switch (cc2State) {
        case CCOpen:
          return SourcePortState(CONNECTION_POWERED_CABLE_NO_SINK, CABLE_FLIPPED);

        case CCRd:
          return SourcePortState(CONNECTION_VCONN, CABLE_FLIPPED);

        case CCRa:
          return SourcePortState(CONNECTION_AUDIO_ACCESSORY);
      }
      break;

    default:
      Serial.println("unreachable");
      break;
  }
  return SourcePortState(CONNECTION_NONE);
}

CurrentAdvertisement bcLvlToCurrentAdvertisement(uint8_t bcLvl) {
  switch (bcLvl) {
    case 0x0:
      // < 200 mV
      return CURRENT_NONE;
    case 0x1:
      // > 200 mV, < 660 mV
      return CURRENT_DEFAULT;
    case 0x2:
      // > 660 mV, < 1.23 V
      return CURRENT_1A5;
    case 0x3:
      // > 1.23 V
      return CURRENT_3A;
    default:
      Serial.print("bcLvlToCurrentAdvertisement called with invalid value 0x");
      Serial.println(bcLvl, HEX);
      return CURRENT_NONE;
  }
}

const char * currentAdvertisementToString(CurrentAdvertisement value) {
  switch (value) {
    case CURRENT_NONE:
      return "no current";
    case CURRENT_DEFAULT:
      return "default current";
    case CURRENT_1A5:
      return "1.5 A";
    case CURRENT_3A:
      return "3 A";
    default:
      return "<invalid current advertisement>";
  }
}

SinkPortState Adafruit_FUSB302B::pollSink() {

  // TODO: support other kinds of connections besides CONNECTION_SINK.

  // Power on stuff!
  _powerBandgapWake->write(1);
  _powerReceiverCurref->write(1);
  _powerMeasureBlock->write(1);
  _powerInternalOsc->write(1);

  // Enable the device pull-downs on CC1 and CC2.
  _switches0Pdwn1->write(1);
  _switches0Pdwn2->write(1);

  if (_status0Vbusok->read() == 0) {
    return SinkPortState(CURRENT_NONE);
  }

  _switches0Meascc1->write(1);
  _switches0Meascc2->write(0);

  uint8_t vCC1 = _status0BcLvl->read();
  CurrentAdvertisement cc1Adv = bcLvlToCurrentAdvertisement(vCC1);

  _switches0Meascc1->write(0);
  _switches0Meascc2->write(1);

  uint8_t vCC2 = _status0BcLvl->read();
  CurrentAdvertisement cc2Adv = bcLvlToCurrentAdvertisement(vCC2);

  // The CC pin that is at a higher voltage (i.e. pulled up by Rp in the source) indicates the
  // orientation [USB Type-C R2.2 § 4.5.1.2.1].
  CurrentAdvertisement current = CURRENT_NONE;
  CableFlipped flipped = CABLE_NA;
  if (vCC1 > vCC2) {
    current = bcLvlToCurrentAdvertisement(vCC1);
    flipped = CABLE_NOT_FLIPPED;
  } else if (vCC2 > vCC1) {
    current = bcLvlToCurrentAdvertisement(vCC2);
    flipped = CABLE_FLIPPED;
  } else if (vCC1 != 0 && vCC2 != 0) {
    Serial.println("Both Source's CCs have Rd termination; this is a debug accessory or invalid connection");
  }

  return SinkPortState(current, flipped);
}

void Adafruit_FUSB302B::enableInterrupts(FUSB302B_Interrupts interrupts) {
  // First disable *all* interrupts.
  _control0IntMask->write(1);
  _regMaska->write(0xFF, 1);
  _regMaskb->write(0xFF, 1);
  _regMask->write(0xFF, 1);

  // Read all the interrupt bits, to clear any existing ones.
  clearInterrupts();

  uint8_t intMask = ~(0xFF & interrupts);

  Serial.print("REG_MASK = 0x");
  Serial.println(intMask, HEX);
  _regMask->write(intMask, 1);

  delay(300);
  _control0IntMask->write(0);
}

void Adafruit_FUSB302B::clearInterrupts() {
  noInterrupts();
  uint8_t buffer[1];
  _regInterrupta->read(buffer, 1);
  _regInterruptb->read(buffer, 1);
  _regInterrupt->read(buffer, 1);
  interrupts();
}

void Adafruit_FUSB302B::whatInterrupts() {
  noInterrupts();

  uint8_t interrupt = 0;
  uint8_t interrupta = 0;
  uint8_t interruptb = 0;
  _regInterrupt->read(&interrupt);
  _regInterrupta->read(&interrupta);
  _regInterruptb->read(&interruptb);

  if (interrupt) {
    Serial.println("REG_INTERRUPT:");
  }
  if (interrupt & 0x80) {
    Serial.println("\tI_VBUSOK");
  }
  if (interrupt & 0x40) {
    Serial.println("\tI_ACTIVITY");
  }
  if (interrupt & 0x20) {
    Serial.println("\tI_COMP_CHNG");
  }
  if (interrupt & 0x10) {
    Serial.println("\tI_CRC_CHK");
  }
  if (interrupt & 0x08) {
    Serial.println("\tI_ALERT");
  }
  if (interrupt & 0x04) {
    Serial.println("\tI_WAKE");
  }
  if (interrupt & 0x02) {
    Serial.println("\tI_COLLISION");
  }
  if (interrupt & 0x01) {
    Serial.println("\tI_BC_LVL");
  }

  if (interrupta) {
    Serial.println("REG_INTERRUPTA:");
  }
  if (interrupta & 0x80) {
    Serial.println("\tI_OCP_TEMP");
  }
  if (interrupta & 0x40) {
    Serial.println("\tI_TOGDONE");
  }
  if (interrupta & 0x20) {
    Serial.println("\tI_SOFTFAIL");
  }
  if (interrupta & 0x10) {
    Serial.println("\tI_RETRYFAIL");
  }
  if (interrupta & 0x08) {
    Serial.println("\tI_HARDSENT");
  }
  if (interrupta & 0x04) {
    Serial.println("\tI_TXSENT");
  }
  if (interrupta & 0x02) {
    Serial.println("\tI_SOFTRST");
  }
  if (interrupta & 0x01) {
    Serial.println("\tI_HARDRST");
  }

  if (interruptb) {
    Serial.println("REG_INTERRUPTB:");
  }
  if (interruptb & 0x01) {
    Serial.println("\tI_GCRCSENT");
  }

  //Serial.print("I/A/B: 0x");
  //Serial.print(interrupt);
  //Serial.print(", 0x");
  //Serial.print(interrupta);
  //Serial.print(", 0x");
  //Serial.println(interruptb);

  interrupts();
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

/** Assumes:
 * - The other CC pin's pull-up is disabled (`REG_SWITCHES0->puen2 = 0`)
 * - The other CC pin's measurement is disabled (`REG_SWITCHES0->measureccX = 0`)
 * - Measurement is set to CC instead of VBus (`REG_MEASURE->meas_vbus = 0`)
 */

CCState Adafruit_FUSB302B::determineCCState(CCPin pin) {

  CCState state;

  // Set register variables that abstract which CC pin we're operating on.
  Adafruit_BusIO_RegisterBits *ccPullUp;
  Adafruit_BusIO_RegisterBits *ccMeas;
  if (pin == CC1) {
    ccPullUp = _switches0Puen1;
    ccMeas = _switches0Meascc1;
  } else {
    ccPullUp = _switches0Puen2;
    ccMeas = _switches0Meascc2;
  }

  // Make sure our CC line is pulled-up, and enable measuring it.
  ccPullUp->write(1);
  ccMeas->write(1);

  // Compare to vRd threshold.
  _measureMdac->write(THRESHOLD_vRd);
  delay(100); // TODO

  if (_status0Comp->read()) {
    // vCC > vRd
    // Nothing is connected.
    state = CCOpen;
  } else {
    // If we're lower than vRd, then *something* is connected.
    // we need to make another measurement to determine what.
    _measureMdac->write(THRESHOLD_vRa);

    if (_status0Comp->read()) {
      // vCC > vRa && vCC < vRd
      // (vCC is in-between vRa's and vRd's thresholds).
      // We either have a sink, a Vconn-powered accessory,
      // a Vconn-powered device, a or a debug accessory.
      // Our caller will have to read the other CC pin's state
      // to determine what.
      state = CCRd;
    } else {
      // vCC < vRa
      // We either have a powered cable, a Vconn-powered accessory,
      // a Vconn-powered device, or an audio adapter accessory.
      // Our caller will have to read the other CC pin's state
      // to determine what.
      state = CCRa;;
    }
  }

  // Cleanup.
  ccPullUp->write(0);
  ccMeas->write(0);

  return state;
}
