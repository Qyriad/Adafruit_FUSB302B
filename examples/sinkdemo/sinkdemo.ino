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
  while (true) {
    usbc.beginSource(CURRENT_DEFAULT);
    delay(2000);
  }

  //auto devId = usbc.getDeviceId();
  //
  //Serial.print("Got device ID: 0x");
  //Serial.println(devId.deviceId, HEX);
}

void loop() {
}
