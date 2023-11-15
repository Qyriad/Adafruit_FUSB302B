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

#include <stdint.h>

enum FUSB302B_VersionId {
  FUSB302B_A = 0b1000,
  FUSB302B_B = 0b1001,
  FUSB302B_C = 0b1010,
};

enum FUSB302B_PowerRole {
  POWER_SINK,
  POWER_SOURCE,
  POWER_DUAL,
};

enum FUSB302B_DataRole {
  /** @brief The "Device" role, typically used by peripherals. */
  DATA_UPSTREAM_FACING,

  /** @brief The "Host" role, typically used by computers. */
  DATA_DOWNSTREAM_FACING,

  DATA_DUAL,
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
  //bool begin(FUSB302B_PowerRole powerMode);
  //Adafruit_FUSB302B(FUSB302B_PowerRole powerMode);
  Adafruit_FUSB302B(TwoWire *wire = &Wire);

  bool beginSource(uint32_t advertisedVoltage); // FIXME: should be an enumeration.

  FUSB302B_DeviceId getDeviceId();

private:
  Adafruit_I2CDevice *_i2cDev;
  TwoWire *_i2c;

  FUSB302B_DeviceId _deviceId;

  Adafruit_BusIO_Register *_reg_switches0;

  /** @brief `PDWN1` of `SWITCHES0` register, for the device pull-down on CC1. Bit 0. */
  Adafruit_BusIO_RegisterBits *_switches0_pdwn1;
  /** @brief `PDWN2` of `SWITCHES0` register, for the device pull-down on CC2. Bit 1. */
  Adafruit_BusIO_RegisterBits *_switches0_pdwn2;
  /** @brief `MEAS_CC1` of `SWITCHES0` register, for enabling the CC1 measure block. Bit 2. */
  Adafruit_BusIO_RegisterBits *_switches0_meascc1;
  /** @brief `MEAS_CC2` of `SWITCHES0` register, for enabling the CC2 measure block. Bit 3. */
  Adafruit_BusIO_RegisterBits *_switches0_meascc2;

  Adafruit_BusIO_RegisterBits *_switches0_puen1;
  Adafruit_BusIO_RegisterBits *_switches0_puen2;


  //Adafruit_BusIO_Register _reg_switches1;

  Adafruit_BusIO_Register *_reg_measure;
  Adafruit_BusIO_RegisterBits *_measure_mdac;
  Adafruit_BusIO_RegisterBits *_measure_meas_vbus;
  //// XXX: Should always be 1.
  //Adafruit_BusIO_RegisterBits *_measure_reserved;

  ////Adafruit_BusIO_Register _reg_slice;
  //
  //Adafruit_BusIO_Register _reg_control0;
  //
  ///** @brief `TX_START` of `CONTROL0` register, for starting the TX FIFO. Bit 0. */
  //Adafruit_BusIO_RegisterBits _control0_tx_start;
  //
  ///** @brief `AUTO_PRE` of `CONTROL0` register, for auto-starting TX when a good CRC is received. Bit 1. */
  //Adafruit_BusIO_RegisterBits _control0_auto_pre;
  //
  ///** @brief `HOST_CUR` of `CONTROL0` register, for controlling the host pull-up current value. Bits 3:2.
  // *
  // * 00: No current
  // *
  // * 01: 80 uA for default USB power
  // *
  // * 10: 180 uA for 1.5 A
  // *
  // * 11: 330 uA for 3 A
  // */
  //Adafruit_BusIO_RegisterBits _control0_host_cur;
  //
  ///** @brief `INT_MASK` of `CONTROL0` register, for controlling if interrupts are enabled. Bit 5.
  // *
  // * Enabled by default, meanining that interrupts are masked by default.
  // */
  //Adafruit_BusIO_RegisterBits _control0_int_mask;
  //
  ///** @brief `TX_FLUSH` of `CONTROL0` register. Write to flush the TX FIFO content. Bit 6. */
  //Adafruit_BusIO_RegisterBits _control0_tx_flush;
  //
  //Adafruit_BusIO_Register _reg_control1;

  Adafruit_BusIO_Register *_reg_control2;

  /** @brief `TOGGLE` of `CONTROL2` register, for enabling auto-toggling(?). Bit 0. */
  Adafruit_BusIO_RegisterBits *_control2_toggle;

  /** @brief `MODE` of `CONTROL2` register, for setting what functionality `_control3_toggle` controls. Bits 2:1.
   *
   * 01: `TOGGLE` enables DRP polling.
   *
   * 10: `TOGGLE` enables SNK polling.
   *
   * 11: `TOGGLE` enables SRC polling.
   */
  Adafruit_BusIO_RegisterBits *_control2_mode;
  //
  ///** @brief `WAKE_EN` of `CONTROL2` register, for enabling wake detection functionality. Bit 3. */
  //Adafruit_BusIO_RegisterBits _control2_wake_enable;
  //
  ///** @brief `TOG_RD_ONLY` of `CONTROL2` register, for making only Rd values trigger `I_TOGGLE`. Bit 5. */
  //Adafruit_BusIO_RegisterBits _control2_toggle_rd_only;
  //
  //Adafruit_BusIO_Register _reg_control3;
  ////Adafruit_BusIO_Register _reg_mask;
  //
  Adafruit_BusIO_Register *_reg_power;
  /** @brief `PWR[0]` of the `POWER` register, for the bandgap and wake circuit. Bit 0. */
  Adafruit_BusIO_RegisterBits *_power_bandgap_wake;
  /** @brief `PWR[1]` of the `POWER` register, for the receiver and current references. Bit 1. */
  Adafruit_BusIO_RegisterBits *_power_receiver_curref;
  /** @brief `PWR[2]` of the `POWER` register, for the measure block. Bit 2. */
  Adafruit_BusIO_RegisterBits *_power_measure_block;
  /** @brief `PWR[3]` of the `POWER` register, for the internal oscillator. Bit 3. */
  Adafruit_BusIO_RegisterBits *_power_internal_osc;
  //
  Adafruit_BusIO_Register *_reg_reset;
  //
  ///** @brief The `SW_RES` field of the `RESET` register. Resets the FUSB302B including I2C registers. Bit 0. */
  Adafruit_BusIO_RegisterBits *_reset_sw;

  Adafruit_BusIO_Register *_reg_status0;
  Adafruit_BusIO_RegisterBits *_status0_comp;
};
