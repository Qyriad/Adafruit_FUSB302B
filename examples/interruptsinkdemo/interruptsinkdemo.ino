#include <Adafruit_FUSB302B.h>

Adafruit_FUSB302B *usbc = nullptr;
bool connectionDidChange = false;

void connectionChanged() {
  Serial.println("connection changed!");
  connectionDidChange = true;
  //usbc->clearInterrupts();
}

void setup() {
  noInterrupts();
  while (!Serial) {
    delay(10);
  }
  Serial.begin(115200);
  Serial.println("began");
  usbc = new Adafruit_FUSB302B;
  //usbc->pollSink();
  Serial.println("new");
  //usbc->clearInterrupts();
  pinMode(12, INPUT_PULLUP);
  usbc->setInterruptHandler(connectionChanged);
  usbc->enableInterrupts(FUSB302B_VBUSOK);
  Serial.println("mostly enabled");
  //usbc->clearInterrupts();
  Serial.println("cleared");
  //interrupts();
}

void loop() {
  Serial.println("Waiting");
  //usbc->pollSink();
  //if (connectionChanged) {
  //  noInterrupts();
  //  connectionDidChange = false;
  //  Serial.println("Connection changed");
  //  usbc->clearInterrupts();
  //  interrupts();
  //}
  //usbc.pollSink();
  delay(500);
}
