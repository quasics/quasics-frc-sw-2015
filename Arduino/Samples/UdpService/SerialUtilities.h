#ifndef SERIAL_UTILITIES_H
#define SERIAL_UTILITIES_H

#include <Arduino.h>  // to pull in Serial class

// Tries to read a "line" (ending with CR or newline) from the specified serial port.
bool getSerialCommand(HardwareSerial& serial, String& serialText);

#endif  // SERIAL_UTILITIES_H

