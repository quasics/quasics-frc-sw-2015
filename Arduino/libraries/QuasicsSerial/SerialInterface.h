#ifndef SERIAL_INTERFACE_H_
#define SERIAL_INTERFACE_H_

#include "Arduino.h"

class SerialInterface{
  public:
    SerialInterface (uint32_t baud = 115200);

    void SerialRead (bool& stringChanged, String& output);
    void SerialWrite (String toWrite);
  private:
    String serialIn;
};

#endif
