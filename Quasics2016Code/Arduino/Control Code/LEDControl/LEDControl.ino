#include "Includes.h"

LEDSerialController* lightControl = NULL;

void setup() {
  lightControl = new LEDSerialController (RedPin, GreenPin, BluePin, 2);
}

void loop() {
  lightControl->LEDSerialProcess();
}









