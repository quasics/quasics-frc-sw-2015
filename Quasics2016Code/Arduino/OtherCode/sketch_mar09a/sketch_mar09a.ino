#include <ExpandedLEDSerialController.h>
#include <LEDController.h>
#include <LEDSerialController.h>

LEDController* ledControll;

void setup() {
  // put your setup code here, to run once:
  ledControll = new LEDController (11, 10, 9);
}

void loop() {
  // put your main code here, to run repeatedly:
  ledControll->SetRed(204);
  ledControll->SetGreen(204);
  ledControll->SetBlue(255);
  ledControll->SetBrightness(1);
}
