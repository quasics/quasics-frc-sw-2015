#include "SerialInterface.h"
//#define USE_MEGA_SERIAL_

SerialInterface* serialInterface;

void setup() {
  // put your setup code here, to run once:
  serialInterface = (new SerialInterface ());
}

void loop() {
  // put your main code here, to run repeatedly:
  bool changed = false;
  static String output = "";
  serialInterface->SerialRead(changed, output);
  if (changed){
    serialInterface->SerialWrite(output);
    output = "";
  }
}
