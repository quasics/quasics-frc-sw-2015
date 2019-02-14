#include "SerialUtilities.h"

// Tries to read a command set from the specified serial port.
bool getSerialCommand(HardwareSerial& serial, String& serialText) {
  serialText = "";
  static String buffer;
  while (serial.available() > 0) {
    char rc = serial.read();
    if (rc == '\n' || rc == '\r') {
      serialText = buffer;
      buffer = "";
      break;
    }
    buffer += rc;
  }
  return serialText.length() > 0;
}

