#ifndef NEO_PIXEL_USB_CONTROLLER_H_
#define NEO_PIXEL_USB_CONTROLLER_H_

#include "NeoPixelController.h"
#include "Arduino.h"

class NeoPixelUSBController {
  public:
    NeoPixelUSBController (uint32_t pin, float loopSeconds, uint8_t brightness, uint32_t stripLength,
                           neoPixelType type = NEOPIXEL_RGB_STRIP_TYPE);
    void NeoPixelSerialProcess ();

  private:
    NeoPixelController* strip;

    String serialIn;
    void Translator (String input);
    void PrintHelpInfo ();
};

#endif //NEO_PIXEL_USB_CONTROLLER_H_
