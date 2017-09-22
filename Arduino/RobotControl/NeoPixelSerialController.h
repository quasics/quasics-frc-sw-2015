#ifndef NEO_PIXEL_SERIAL_CONTROLLER_H_
#define NEO_PIXEL_SERIAL_CONTROLLER_H_

#include "NeoPixelController.h"
#include "Arduino.h"

class NeoPixelSerialController {
  public:
    NeoPixelSerialController (uint32_t pin, float loopSeconds, uint8_t brightness, uint32_t stripLength,
                              neoPixelType type = NEOPIXEL_RGB_STRIP_TYPE);
    void NeoPixelSerialProcess ();
  
  private:
    NeoPixelController* strip;

    void Translator (const char * input);
};

#endif
