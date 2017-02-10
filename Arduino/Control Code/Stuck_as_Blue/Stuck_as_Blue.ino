#include <ExpandedLEDSerialController.h>
#include <LEDController.h>
#include <LEDSerialController.h>

ExpandedLEDSerialController* ledController;

void setup() {
  // put your setup code here, to run once:
  ledController = (new ExpandedLEDSerialController (11, 10, 9, 6, 3));
}

void loop() {
  // put your main code here, to run repeatedly:
  ledController->SetMode(ExpandedLEDSerialController::kBlue);
  ledController->SetState(ExpandedLEDSerialController::kBreathing);
  
}
