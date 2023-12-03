#include <Adafruit_FUSB302B.h>

TwoWire *i2c = &Wire;
Adafruit_I2CDevice *i2c_dev = nullptr;

void setup() {
  while (!Serial) {
    delay(10);
  }
  Serial.begin(115200);

  Adafruit_FUSB302B usbc;
  SourcePortState sourcePort;

  while (true) {
    sourcePort = usbc.beginSource(CURRENT_DEFAULT);
    if (!sourcePort.isConnected()) {
      Serial.println("Waiting for connection...");
      do {
        delay(200);
        sourcePort = usbc.beginSource(CURRENT_DEFAULT);
      } while (!sourcePort.isConnected());
    }

    const char *connectionDesc = "";
    switch (sourcePort.connection) {
      case CONNECTION_SINK:
        connectionDesc = "Sink";
        break;
      case CONNECTION_POWERED_CABLE_NO_SINK:
        connectionDesc = "Powered cable without sink";
        break;
      case CONNECTION_VCONN:
        connectionDesc = "Powered cable with sink, VPA, or VPD";
        break;
      case CONNECTION_DEBUG_ACCESSORY:
        connectionDesc = "Debug accessory";
        break;
      case CONNECTION_AUDIO_ACCESSORY:
        connectionDesc = "Audio accessory";
        break;
    }

    const char *flippedDesc = "";
    switch (sourcePort.flipped) {
      case CABLE_NOT_FLIPPED:
        flippedDesc = "not flipped";
        break;
      case CABLE_FLIPPED:
        flippedDesc = "flipped";
        break;
    }

    Serial.print(connectionDesc);
    Serial.print(" connected, ");
    Serial.println(flippedDesc);
    delay(2000);
  }

  //auto devId = usbc.getDeviceId();
  //
  //Serial.print("Got device ID: 0x");
  //Serial.println(devId.deviceId, HEX);
}

void loop() {
}

