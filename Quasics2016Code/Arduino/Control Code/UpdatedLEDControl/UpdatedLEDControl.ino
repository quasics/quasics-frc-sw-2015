#include <ExpandedLEDSerialControllerMega.h>


ExpandedLEDSerialControllerMega* ledController;


void setup() {
  // put your setup code here, to run once:
  ledController = (new ExpandedLEDSerialControllerMega (11, 10, 9, 6, 2));
}

void loop() {
  // put your main code here, to run repeatedly:
  ledController->LEDSerialProcess();
}
