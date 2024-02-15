#include <Adafruit_FUSB302B.h>

void setup() {
  while (!Serial) {
    delay(10);
  }
  Serial.begin(115200);
}

void loop() {
  Adafruit_FUSB302B usbc;

  SinkPortState sinkPort;

  Serial.println("Waiting for connection...");
  while (true) {
    SinkPortState sinkPort;
    do {
      delay(200);
      sinkPort = usbc.pollSink();
    } while (!sinkPort.isConnected());

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
}
