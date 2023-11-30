#include <Adafruit_FUSB302B.h>

TwoWire *i2c = &Wire;
Adafruit_I2CDevice *i2c_dev = nullptr;

void setup() {
  while (!Serial) {
    delay(10);
  }
  Serial.begin(115200);

  //Adafruit_FUSB302B usbc(POWER_SOURCE);
  Adafruit_FUSB302B usbc;
  //usbc.begin(POWER_SOURCE);
  //while (true) {
  //  usbc.beginSource(CURRENT_DEFAULT);
  //  delay(2000);
  //}

  SinkPortState sinkPort;

  while (true) {
    sinkPort = usbc.beginSink(CONNECTION_SINK);
    if (!sinkPort.isConnected()) {
      Serial.println("Waiting for connection...");
      do {
        delay(200);
        sinkPort = usbc.beginSink(CONNECTION_SINK);
      } while (!sinkPort.isConnected());
    }

    const char *currentDesc = "";
    switch (sinkPort.currentAdvertisement) {
      case CURRENT_DEFAULT:
        currentDesc = "default current";
        break;
      case CURRENT_1A5:
        currentDesc = "1.5 A";
        break;
      case CURRENT_3A:
        currentDesc = "3 A";
        break;
    }

    const char *flippedDesc = "";
    switch (sinkPort.flipped) {
      case CABLE_NOT_FLIPPED:
        flippedDesc = "not flipped";
        break;
      case CABLE_FLIPPED:
        flippedDesc = "flipped";
        break;
    }

    Serial.print("Source connected, advertising ");
    Serial.print(currentDesc);
    Serial.print(", ");
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
