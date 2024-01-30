/**
 * @file Adafruit_FUSB302B.h
 *
 * @mainpage Onsemi FUSB302B USB-C PowerDelivery Controller Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the Onsemi FUSB302B USB-C Power Delivery controller.
 *
 * @see Adafruit_FUSB302B
 * @see Adafruit_FUSB302B::pollSink
 * @see Adafruit_FUSB302B.h
 *
 * @section author Author
 *
 * Written by Qyriad <qyriad@qyriad.me>, 2023.
 *
 * @section license License
 *
 * MIT license. All text above must be included in any redistribution.
 *
 * @section usage Usage
 *
 * @subsection connecting Connecting the breakout
 *
 * The breakout board will have 8 through-hole pins. At least 4 of those
 * pins must be connected: SDA, SCL, GND, and 3V, which should be connected
 * as follows:
 *
 * | FUSB302B Breakout Pin | Arduino Pin |
 * | --------------------- | ----------- |
 * | SDA                   | SDA         |
 * | SCL                   | SCL         |
 * | GND                   | GND         |
 * | 3V                    | +3V3        |
 *
 *
 * What you do with VBus changes depending on how you are using this chip. If
 * you are using this chip as a **sink**, then VBus must **not** be driven —
 * it may be connected to an *input* pin of something, or may be left
 * not connected at all (floating), but must not be connected to GND, any
 * output pin, or any voltage supply.
 *
 * If you are using this chip as a **source**, then VBus **must** be connected
 * to a 5 volt output.
 *
 * @subsection code Using the library
 *
 * @subsubsection code_as_sink As a USB-PD sink
 *
 * Once you have the board connected up, interacting with it will be done with
 * the Adafruit_FUSB302B class. First, create an instance of this class (likely
 * as a global variable). Then call Adafruit_FUSB302B::pollSink, which returns
 * a SinkPortState. Then use SinkPortState::isConnected to check if the
 * previous poll detected a connection. Repeatedly call
 * Adafruit_FUSB302B::pollSink until SinkPortState::isConnected returns `true`
 * to wait for a connection.
 *
 * Once you have a connection, SinkPortState::currentAdvertisement will tell
 * you how much current the other side of the connection is willing to provide.
 *
 * You may also check SinkPortState::flipped if you would like to know if the
 * USB-C cable connecting the FUSB302B to its source has a different
 * orientation on one side than the other.
 *
 *
 * A full example can be found in the `examples/sinkdemo` directory.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>

#include <stdint.h>

/** @private While this API is WIP. */
enum FUSB302B_Interrupts {
  FUSB302B_VBUSOK = 0x80,
};

/** @private */
enum CCState {
  /** Nothing is attached. */
  CCOpen,
  /** A sink, VPA, VPD, debug accessory is attached.
   * (Read other CC line to find out which).
   */
  CCRd,
  /** A powered cable, VPA, VPD, or audio accessory is attached.
   * (Read other CC line to find out which).
   */
  CCRa,
};

/** @private */
enum CCPin {
  CC1,
  CC2,
};

/**
 * @brief The amount of current a USB-C port is advertising as being capable of sourcing.
 *
 * This type is returned as part of @ref SinkPortState; see that for more information.
 *
 * @see SinkPortState
 */
enum CurrentAdvertisement {
  /** @brief Nothing is attached. */
  CURRENT_NONE = 0x0,

  /** @brief 500 mA for USB 2.0 ports, 900 mA for single-lane USB 3.2, and 1.5 A for dual-lane USB 3.2.
   *
   * Applies 80 uA to the CC line.
   */
  CURRENT_DEFAULT = 0x1,

  /** @brief Medium current mode: 1.5 A
   *
   * Applies 180 uA to the CC line.
   */
  CURRENT_1A5 = 0x2,

  /** @brief High current mode: 3 A
   *
   * Applies 330 uA to the CC line.
   */
  CURRENT_3A = 0x3,
};

/** @brief The type returned by Adafruit_FUSB302B::beginSource. */
enum PortConnection {
  /** @brief Indicates that there is nothing on the other side of the USB-C port.
   *
   * @note This might not indicate that there is no cable physically connected,
   * but does indicate that nothing that "talks USB-C" is connected on the other side.
   */
  CONNECTION_NONE,

  /** @brief Indicates that the other side of this USB-C port is a power *sink*
   * — something that accepts power.
   */
  CONNECTION_SINK,

  /** @brief Indicates that there is a powered cable connected to this USB-C
   * port, but without any sink connected.
   */
  CONNECTION_POWERED_CABLE_NO_SINK,

  /** @brief A powered cable with sink, Vconn-powered accessory, or Vconn-powered device
   * is connected to this USB-C port.
   */
  CONNECTION_VCONN,

  /** @brief A USB-C debug accessory is connected to this USB-C port. */
  CONNECTION_DEBUG_ACCESSORY,

  /** @brief A USB-C audio accessory — like USB-C to 3.5mm audio jack adapters — is connected. */
  CONNECTION_AUDIO_ACCESSORY,
};

/** @brief Represents the "flipped" state of a USB-C connection.
 *
 * The USB-C connector is reversible, so one side could be connected with one orientation,
 * and the other side with a different orientation. When the two orientations are different,
 * the USB-C cable is said to be "flipped".
 */
enum CableFlipped {
  /** @brief The connections between the cable and the USB-C ports are the same on both sides. */
  CABLE_NOT_FLIPPED,

  /** @brief The connections between the cable and the USB-C ports are different between the two sides. */
  CABLE_FLIPPED,

  /** @brief The question of "is the cable flipped" doesn't apply here.
   *
   * This might happen if there is no connection at all, or if the connection is one that can't be
   * "flipped" such as a debug accessory or audio accessory.
   */
  CABLE_NA,
};

/** @private while this API is WIP. */
struct SourcePortState {
  PortConnection connection;
  CableFlipped flipped;

  SourcePortState(PortConnection connection = CONNECTION_NONE, CableFlipped flipped = CABLE_NA) :
    connection(connection), flipped(flipped) { }

  bool isConnected();
};

/**
 * @brief The type returned by @ref Adafruit_FUSB302B::pollSink, representing the status of
 * FUSB302B's port in sink mode. You should not need to construct this type yourself.
 *
 * When the FUSB302B is in sink mode (capable of *drawing* power), this type tells you what is
 * connected to that sink. If nothing is connected, then @ref currentAdvertisement will be
 * @ref CONNECTION_NONE, and @ref flipped will be @ref CABLE_NA.
 */
struct SinkPortState {
  /** @brief The amount of current the thing connected to this port is advertising.
   * See CurrentAdvertisement for more information.
   */
  CurrentAdvertisement currentAdvertisement;

  /** @brief The "flipped" state of this connection. See CableFlipped for more information. */
  CableFlipped flipped;

  /** @private */
  SinkPortState(CurrentAdvertisement advertisement = CURRENT_DEFAULT, CableFlipped flipped = CABLE_NA) :
    currentAdvertisement(advertisement), flipped(flipped) { }

  /** @brief Returns true if this connection state is one that has a connection to a sink.
   *
   * @returns True if there is a current advertisement, which indicates a connection. */
  bool isConnected();
};

/** @brief Represents the value of the Device ID register of the FUSB302B.
 *
 * You can use these values to check what exact product, version, and revision this chip is.
 */
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
   *
   * | Value  | Version |
   * | ------ | ------- |
   * | 0b1000 | A       |
   * | 0b1001 | B       |
   * | 0b1010 | C       |
   */
  uint8_t versionId;

  /**
   * @brief The part of the I2C device ID that indicates which exact product this is.
   *
   * According to the datasheet, the following values correspond to the following products:
   * | Value | Version                                  |
   * | ----- | ---------------------------------------- |
   * | 0b00 | FUSB302BMPX, FUSB302BVMPX, or FUSB302BUCX |
   * | 0b01 | FUSB302B01MPX                             |
   * | 0b10 | FUSB302B10MPX                             |
   * | 0b11 | FUSB302B11MPX                             |
   */
  uint8_t productId;

  /**
   * @brief The part of the I2C device ID that indicates what revision of the device version specified in
   * @ref deviceId.
   *
   * According to the datasheet, the following values correspond to the following revisions:
   * | Value | Version |
   * | ----- | ------- |
   * | 0b00  | revA    |
   * | 0b01  | revB    |
   * | 0b10  | revC    |
   * | 0b11  | revD    |
   */
  uint8_t revisionId;
};


/** @brief Class for interfacing with a FUSB302B Power Delivery controller.
 *
 * Notable method: Adafruit_FUSB302B::pollSink
 */
class Adafruit_FUSB302B {
public:
  /** @brief Initialize this FUSB302B using I²C.
   *
   * @param wire The I2C interface to to use when communicating to this FUSB302B chip.
   * Defaults to [Wire], the default I2C interface on Arduino boards. This is often connected
   * to the pins labeled `SDA` and `SCL` on the physical board. For example, on the Arduino Uno
   * the SDA of the default I2C interface is pin 18.
   */
  Adafruit_FUSB302B(TwoWire *wire = &Wire);

  /** @private While this API is WIP. */
  SourcePortState beginSource(CurrentAdvertisement advertisedCurrent);

  /** @brief Poll the USB-C port for a connection, with the USB-C port in *sink* mode.
   *
   * *Sink mode* means that this method will look for a *source* or other compatible connection.
   *
   * Call SinkPortState::isConnected on the return value of this function to check if this poll
   * detected a connection. If it did, you may check the value of
   * SinkPortState::currentAdvertisement to determine how much current the other side is
   * advertising, if any.
   *
   * When using this function, the VBus pin of the FUSB302B must **not** be driven.
   *
   * @returns A SinkPortState representing the state of this USB-C port from this poll.
   */
  SinkPortState pollSink();

  /** @private While this API is WIP. */
  void enableInterrupts(FUSB302B_Interrupts interrupts);
  /** @private While this API is WIP. */
  void clearInterrupts();
  /** @private While this API is WIP. */
  void whatInterrupts();

  /** @brief Return the I2C chip ID for this FUSB302B. See FUSB302B_DeviceId for more information.
   *
   * @returns A FUSB302B_DeviceId which contains full information about the value of
   * the Device ID register.
   */
  FUSB302B_DeviceId getDeviceId();

private:

  CCState determineCCState(CCPin pin);

  Adafruit_I2CDevice *_i2cDev;
  TwoWire *_i2c;

  FUSB302B_DeviceId _deviceId;

  Adafruit_BusIO_Register *_regSwitches0;

  /** @brief `PDWN1` of `SWITCHES0` register, for the device pull-down on CC1. Bit 0. */
  Adafruit_BusIO_RegisterBits *_switches0Pdwn1;
  /** @brief `PDWN2` of `SWITCHES0` register, for the device pull-down on CC2. Bit 1. */
  Adafruit_BusIO_RegisterBits *_switches0Pdwn2;
  /** @brief `MEAS_CC1` of `SWITCHES0` register, for enabling the CC1 measure block. Bit 2. */
  Adafruit_BusIO_RegisterBits *_switches0Meascc1;
  /** @brief `MEAS_CC2` of `SWITCHES0` register, for enabling the CC2 measure block. Bit 3. */
  Adafruit_BusIO_RegisterBits *_switches0Meascc2;

  Adafruit_BusIO_RegisterBits *_switches0Puen1;
  Adafruit_BusIO_RegisterBits *_switches0Puen2;


  Adafruit_BusIO_Register *_regMeasure;
  Adafruit_BusIO_RegisterBits *_measureMdac;
  Adafruit_BusIO_RegisterBits *_measureMeasVbus;

  Adafruit_BusIO_Register *_regControl0;

  /** @brief `HOST_CUR` of `CONTROL0` register, for controlling the host pull-up current value. Bits 3:2.
   *
   * 00: No current
   *
   * 01: 80 uA for default USB power
   *
   * 10: 180 uA for 1.5 A
   *
   * 11: 330 uA for 3 A
   */
  Adafruit_BusIO_RegisterBits *_control0HostCur;

  /** @brief `INT_MASK` of `CONTROL0` register, for controlling if interrupts are enabled. Bit 5.
   *
   * Enabled by default, meanining that interrupts are masked by default.
   */
  Adafruit_BusIO_RegisterBits *_control0IntMask;

  Adafruit_BusIO_Register *_regControl2;

  /** @brief `TOGGLE` of `CONTROL2` register, for enabling auto-toggling(?). Bit 0. */
  Adafruit_BusIO_RegisterBits *_control2Toggle;

  /** @brief `MODE` of `CONTROL2` register, for setting what functionality `_control3_toggle` controls. Bits 2:1.
   *
   * 01: `TOGGLE` enables DRP polling.
   *
   * 10: `TOGGLE` enables SNK polling.
   *
   * 11: `TOGGLE` enables SRC polling.
   */
  Adafruit_BusIO_RegisterBits *_control2Mode;
  Adafruit_BusIO_Register *_regPower;
  /** @brief `PWR[0]` of the `POWER` register, for the bandgap and wake circuit. Bit 0. */
  Adafruit_BusIO_RegisterBits *_powerBandgapWake;
  /** @brief `PWR[1]` of the `POWER` register, for the receiver and current references. Bit 1. */
  Adafruit_BusIO_RegisterBits *_powerReceiverCurref;
  /** @brief `PWR[2]` of the `POWER` register, for the measure block. Bit 2. */
  Adafruit_BusIO_RegisterBits *_powerMeasureBlock;
  /** @brief `PWR[3]` of the `POWER` register, for the internal oscillator. Bit 3. */
  Adafruit_BusIO_RegisterBits *_powerInternalOsc;

  Adafruit_BusIO_Register *_regMask;

  Adafruit_BusIO_Register *_regReset;

  /** @brief The `SW_RES` field of the `RESET` register. Resets the FUSB302B including I2C registers. Bit 0. */
  Adafruit_BusIO_RegisterBits *_resetSw;

  Adafruit_BusIO_Register *_regMaska;
  Adafruit_BusIO_Register *_regMaskb;

  Adafruit_BusIO_Register *_regStatus0;
  Adafruit_BusIO_RegisterBits *_status0BcLvl;
  Adafruit_BusIO_RegisterBits *_status0Comp;
  Adafruit_BusIO_RegisterBits *_status0Vbusok;

  Adafruit_BusIO_Register *_regInterrupta;
  Adafruit_BusIO_Register *_regInterruptb;

  Adafruit_BusIO_Register *_regInterrupt;
};
