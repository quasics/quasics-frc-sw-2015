#include "Arduino.h"

#include <NeoPixelUSBController.h>

//#define PracticeBoard

//Variables---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const uint32_t delayMS = 0;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//Objects-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifndef PracticeBoard
const uint32_t stripLength = 36;
#else
const uint32_t stripLength = 24;
#endif

const uint32_t pin = 6;
const uint8_t brightness = 255;
const float loopLength = 2;
NeoPixelUSBController* strip;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  strip = new NeoPixelUSBController(pin, loopLength, brightness, stripLength);
}

void loop() {
  strip->NeoPixelSerialProcess();
}
