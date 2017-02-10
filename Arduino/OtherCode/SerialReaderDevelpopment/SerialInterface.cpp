#include "SerialInterface.h"
SerialInterface::SerialInterface (uint32_t baud) {
  Serial.begin(baud);
  serialIn = "";
}

void SerialInterface::SerialRead (bool& stringChanged, String& output) {
  if (Serial.available() == 0) {
    stringChanged = false;
    output = "";
  } else {
    while (Serial.available() > 0 ) {
      serialIn += char(Serial.read());
      delayMicroseconds(100);
    }
    stringChanged = true;
    output = serialIn;
    serialIn = "";
  }
}

void SerialInterface::SerialWrite (String toWrite) {
  Serial.println (toWrite);
}

