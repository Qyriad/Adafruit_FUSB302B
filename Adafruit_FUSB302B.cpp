#include <assert.h>

#include "Adafruit_BusIO_Register.h"
#include "Adafruit_FUSB302B.h"

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

//bool Adafruit_FUSB302B::begin(FUSB302B_PowerRole powerMode) {
//Adafruit_FUSB302B::Adafruit_FUSB302B(FUSB302B_PowerRole powerMode) {
Adafruit_FUSB302B::Adafruit_FUSB302B(TwoWire *wire) {
  _i2c = wire;
  _i2cDev = new Adafruit_I2CDevice(
    DEVICE_ADDR, // addr
    _i2c
  );

  if (!_i2cDev->begin()) {
    //return false;
    while (true) { }
  }


  _deviceId = getDeviceId();
  // The FUSB302B has a bunch of valid device IDs that could be returned here.
  // To check if this one is valid, we'll check that it's not 0xFF or 0x00.
  if (_deviceId.deviceId == 0 || _deviceId.deviceId == 0xFF) {
    //return false;
    while (true) { }
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
    //return false;
    while (true) { }
  }

  Serial.println("Reset complete"); // Qyriad


  // POWER SOURCE MODE XXX(Qyriad)

  // Setup registers.
  _reg_switches0     = new Adafruit_BusIO_Register(_i2cDev, REG_SWITCHES0);
  _switches0_pdwn1   = new Adafruit_BusIO_RegisterBits(_reg_switches0, 1, 0);
  _switches0_pdwn2   = new Adafruit_BusIO_RegisterBits(_reg_switches0, 1, 1);
  _switches0_meascc1 = new Adafruit_BusIO_RegisterBits(_reg_switches0, 1, 2);
  _switches0_meascc2 = new Adafruit_BusIO_RegisterBits(_reg_switches0, 1, 3);
  _switches0_puen1 = new Adafruit_BusIO_RegisterBits(_reg_switches0, 1, 6);
  _switches0_puen2 = new Adafruit_BusIO_RegisterBits(_reg_switches0, 1, 7);

  //_reg_switches1 = Adafruit_BusIO_Register(_i2cDev, REG_SWITCHES1);
  //_reg_measure = Adafruit_BusIO_Register(_i2cDev, REG_MEASURE);
  //_reg_slice = Adafruit_BusIO_Register(_i2cDev, REG_SLICE);

  //_reg_control0 = Adafruit_BusIO_Register(_i2cDev, REG_CONTROL0);
  //_control0_tx_start = Adafruit_BusIO_RegisterBits(&_reg_control0, 1, 0);
  //_control0_auto_pre = Adafruit_BusIO_RegisterBits(&_reg_control0, 1, 1);
  //_control0_host_cur = Adafruit_BusIO_RegisterBits(&_reg_control0, 2, 2);
  //_control0_int_mask = Adafruit_BusIO_RegisterBits(&_reg_control0, 1, 5);
  //_control0_tx_flush = Adafruit_BusIO_RegisterBits(&_reg_control0, 1, 6);

  //_reg_control1 = Adafruit_BusIO_Register(_i2cDev, REG_CONTROL1);
  //
  //_reg_control2 = Adafruit_BusIO_Register(_i2cDev, REG_CONTROL2);
  //_control2_toggle = Adafruit_BusIO_RegisterBits(&_reg_control2, 1, 0);
  //_control2_mode = Adafruit_BusIO_RegisterBits(&_reg_control2, 1, 2);
  //_control2_wake_enable = Adafruit_BusIO_RegisterBits(&_reg_control2, 1, 3);
  //_control2_toggle_rd_only = Adafruit_BusIO_RegisterBits(&_reg_control2, 1, 5);

  //_reg_control3 = Adafruit_BusIO_Register(_i2cDev, REG_CONTROL3);
  //_reg_mask = Adafruit_BusIO_Register(_i2cDev, REG_MASK);

  _reg_measure = new Adafruit_BusIO_Register(_i2cDev, REG_MEASURE);
  _measure_mdac = new Adafruit_BusIO_RegisterBits(_reg_measure, 6, 0);
  _measure_meas_vbus = new Adafruit_BusIO_RegisterBits(_reg_measure, 1, 6);
  //_measure_reserved = new Adafruit_BusIO_RegisterBits(_reg_measure, 1, 7);

  _reg_control2 = new Adafruit_BusIO_Register(_i2cDev, REG_CONTROL2);
  _control2_toggle = new Adafruit_BusIO_RegisterBits(_reg_control2, 1, 0);
  _control2_mode = new Adafruit_BusIO_RegisterBits(_reg_control2, 2, 2); // TODO: shift by 2 right?

  _reg_power = new Adafruit_BusIO_Register(_i2cDev, REG_POWER);
  _power_bandgap_wake = new Adafruit_BusIO_RegisterBits(_reg_power, 1, 0);
  _power_receiver_curref = new Adafruit_BusIO_RegisterBits(_reg_power, 1, 1);
  _power_measure_block = new Adafruit_BusIO_RegisterBits(_reg_power, 1, 2);
  _power_internal_osc = new Adafruit_BusIO_RegisterBits(_reg_power, 1, 3);

  _reg_reset = new Adafruit_BusIO_Register(_i2cDev, REG_RESET);

  _reset_sw = new Adafruit_BusIO_RegisterBits(_reg_reset, 1, 0);

  _reg_status0 = new Adafruit_BusIO_Register(_i2cDev, REG_STATUS0);
  _status0_comp = new Adafruit_BusIO_RegisterBits(_reg_status0, 1, 5);

  // Reset the device.
  _reset_sw->write(1);
  delay(100); // FIXME: determine better delay.
  _reset_sw->write(0);
  delay(100); // FIXME: determine better delay.

  // Datasheet says we should read all interrupt register bits to clear them before we enable toggling.




  //if (powerMode == POWER_SOURCE) {
  //  // Disable device pull-down resistors (Rd) on CC1 and CC2, and
  //  // apply host pull-up currents (Ip) to CC1 and CC2 (equivalent to Rp).
  //  // USB Type-C R2.2 § 4.5.1.2.1
  //
  //  Adafruit_BusIO_Register switches0(_i2cDev, REG_SWITCHES0);
  //
  //  Adafruit_BusIO_RegisterBits cc1Rd(&switches0, 1, 0);
  //  Adafruit_BusIO_RegisterBits cc2Rd(&switches0, 1, 1);
  //  Adafruit_BusIO_RegisterBits cc1Measure(&switches0, 1, 4);
  //  Adafruit_BusIO_RegisterBits cc2Measure(&switches0, 1, 5);
  //  Adafruit_BusIO_RegisterBits cc1Ip(&switches0, 1, 6);
  //  Adafruit_BusIO_RegisterBits cc2Ip(&switches0, 1, 7);
  //
  //  // 0: no pull down resistor.
  //  cc1Rd.write(0);
  //  cc2Rd.write(0);
  //
  //  // 1: Apply pull-up current.
  //  cc1Ip.write(1);
  //  cc2Ip.write(1);
  //
  //  // Enable measuring the voltage on the CC lines.
  //  cc1Measure.write(1);
  //  cc2Measure.write(1);
  //
  //  Adafruit_BusIO_Register measure(_i2cDev, REG_MEASURE);
  //  Adafruit_BusIO_RegisterBits measureDac(&measure, 5, 0);
  //
  //  Adafruit_BusIO_Register control0(_i2cDev, REG_CONTROL0);
  //
  //   //HOST_Cur[1:0] (size = 2) is at bits 3:2 (so shift = 2).
  //  Adafruit_BusIO_RegisterBits host_current(&control0, 2, 2);
  //  host_current.write(CONTROL0_HOST_CURRENT_80uA);
  //
  //  // Now tell the FUSB302B to enable SOP' packets.
  //  Adafruit_BusIO_Register control1(_i2cDev, REG_CONTROL1);
  //  Adafruit_BusIO_RegisterBits enable_sop_prime(&control1, 1, 0);
  //  enable_sop_prime.write(1);
  //
  //  // Tell the FUSB302B to set the "Source" bit when replying with a GoodCRC.
  //  Adafruit_BusIO_Register switches1(_i2cDev, REG_SWITCHES1);
  //  Adafruit_BusIO_RegisterBits powerRole(&switches1, 1, 7);
  //  Adafruit_BusIO_RegisterBits autoCrc(&switches1, 1, 2);
  //
  //  powerRole.write(1);
  //  autoCrc.write(1);
  //
  //  // Setup for reading general status information.
  //  Adafruit_BusIO_Register status0(_i2cDev, REG_STATUS0);
  //  Adafruit_BusIO_RegisterBits status0All(&status0, 8, 0);
  //
  //  Adafruit_BusIO_Register power(_i2cDev, REG_POWER);
  //  Adafruit_BusIO_RegisterBits powerMeasureBlock(&power, 1, 2);
  //  powerMeasureBlock.write(1);
  //
  //  Adafruit_BusIO_Register status1(_i2cDev, REG_STATUS1);
  //  Adafruit_BusIO_RegisterBits rxEmpty(&status1, 1, 5);
  //
  //  Adafruit_BusIO_Register fifo(_i2cDev, REG_FIFO);
  //
  //  Serial.println("Entering read loop");
  //
  //  while (true) {
  //
  //    if (rxEmpty.read() != 1) {
  //      Serial.println("");
  //      uint8_t byte = 0;
  //      fifo.read(&byte);
  //
  //      switch (byte) {
  //        case TOKEN_TXON:
  //          Serial.println("TXON");
  //          break;
  //        case TOKEN_SOP1:
  //          Serial.println("SOP1");
  //          break;
  //        case TOKEN_SOP2:
  //          Serial.println("SOP2");
  //          break;
  //        case TOKEN_SOP3:
  //          Serial.println("SOP3");
  //          break;
  //        case TOKEN_RESET1:
  //          Serial.println("RESET1");
  //          break;
  //        case TOKEN_RESET2:
  //          Serial.println("RESET2");
  //          break;
  //        case TOKEN_PACKSYM:
  //          Serial.println("PACKSYM");
  //          break;
  //        case TOKEN_JAM_CRC:
  //          Serial.println("JAM_CRC");
  //          break;
  //        case TOKEN_EOP:
  //          Serial.println("EOP");
  //          break;
  //        case TOKEN_TXOFF:
  //          Serial.println("TXOFF");
  //          break;
  //        default:
  //          Serial.print("Unknown token: 0x");
  //          Serial.println(byte, HEX);
  //
  //          //if ((byte & 0b11100000) == 0b11100000) {
  //          if (matchesMask(byte, 0b11100000)) {
  //            Serial.println("SOP");
  //          }
  //          break;
  //      }
  //    } else {
  //      uint8_t measureResults = measureDac.read();
  //      Serial.print("Measured: 0x");
  //      Serial.print(measureResults, HEX);
  //      Serial.print("\t0x");
  //      Serial.print(status0All.read(), HEX);
  //      Serial.println("");
  //    }
  //
  //    // Wait 10 milliseconds between loops.
  //    delay(10);
  //  }
  //
  //} else if (powerMode == POWER_SINK) {
  //  // Enable device pull-down resistors (Rd) on CC1 and CC2,
  //  // and disable host pull-up currents (Ip) on CC1 and CC2.
  //  // USB Type-C R2.2 § 4.5.1.2.1
  //
  //  Serial.println("FUSB302B: POWER SINK mode is not yet supported!");
  //  return false;
  //}

  //return true;
}

bool Adafruit_FUSB302B::beginSource(uint32_t advertisedVoltage) {

  // Power on stuff!
  _power_bandgap_wake->write(1);
  _power_receiver_curref->write(1);
  _power_measure_block->write(1);
  _power_internal_osc->write(1);

  // Disable the device pull-downs on CC1 and CC2.
  _switches0_pdwn1->write(0);
  _switches0_pdwn2->write(0);

  // FIXME: enabling SRC polling seems to take over CC pull-ups, which I don't think we want.
  //// Enable SRC polling.
  //_control2_mode->write(0x03);
  //_control2_toggle->write(1);


  // Alright, let's do detection.

  while (true) {

    // Make sure everything's pull-ups, pull-downs, and measurements are all disabled first.
    _switches0_pdwn1->write(0);
    _switches0_puen1->write(0);
    _switches0_meascc1->write(0);
    _switches0_pdwn2->write(0);
    _switches0_puen2->write(0);
    _switches0_meascc2->write(0);

    CCState cc1State = determineCCState(CC1);
    CCState cc2State = determineCCState(CC2);

    switch (cc1State) {
      case CCOpen:
        switch (cc2State) {
          case CCOpen:
            Serial.println("Nothing attached");
            break;

          case CCRd:
            Serial.println("Sink attached; flipped");
            break;

          case CCRa:
            Serial.println("Powered cable without sink attached; not flipped");
            break;
        }
        break;

      case CCRd:
        switch (cc2State) {
          case CCOpen:
            Serial.println("Sink attached; not flipped");
            break;

          case CCRd:
            Serial.println("Debug accessory attached");
            break;

          case CCRa:
            Serial.println("Powered cable with sink, VPA, or VPD attached; not flipped");
            break;
        }
        break;

      case CCRa:
        switch (cc2State) {
          case CCOpen:
            Serial.println("Powered cable without sink attached; flipped");
            break;

          case CCRd:
            Serial.println("Powered cable with sink, VPA, or VPD attached; flipped");
            break;

          case CCRa:
            Serial.println("Audio adapter accessory attached");
            break;
        }
        break;

      default:
        Serial.println("unreachable");
        break;
    }

    delay(3000);
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

  Serial.print("Version ID: 0x");
  Serial.println(deviceId.versionId, HEX);

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
    ccPullUp = _switches0_puen1;
    ccMeas = _switches0_meascc1;
  } else {
    ccPullUp = _switches0_puen2;
    ccMeas = _switches0_meascc2;
  }

  // Make sure our CC line is pulled-up, and enable measuring it.
  ccPullUp->write(1);
  ccMeas->write(1);

  // Compare to vRd threshold.
  _measure_mdac->write(THRESHOLD_vRd);
  delay(100); // TODO

  if (_status0_comp->read()) {
    // vCC > vRd
    // Nothing is connected.
    state = CCOpen;
  } else {
    // If we're lower than vRd, then *something* is connected.
    // we need to make another measurement to determine what.
    _measure_mdac->write(THRESHOLD_vRa);
    delay(100); // TODO

    if (_status0_comp->read()) {
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
