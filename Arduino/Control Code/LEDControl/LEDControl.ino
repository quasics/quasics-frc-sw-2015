#include "Includes.h"

LEDSerialController* lightControl = NULL;
const unsigned long kHeartRateSeconds = 3600;

void setup() {
  lightControl = new LEDSerialController (RedPin, GreenPin, BluePin, 6, 3600);
}

void loop() {
  lightControl->LEDSerialProcess();
}









