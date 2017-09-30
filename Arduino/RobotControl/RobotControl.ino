#include "NeoPixelController.h"

// #define USE_NEOPIXEL_RING

const uint32_t pin = 6;
const uint8_t brightness = 255;
const float loopLength = 1.5;

#ifdef USE_NEOPIXEL_RING
// Assuming that we're pointing at a medium-sized ring, like Matt's.
const uint32_t stripLength = 16;
const neoPixelType stripType = NEOPIXEL_RING_TYPE;
#else
// Assuming that we're pointing at the lighting on Dante.
const uint32_t stripLength = 35;
const neoPixelType stripType = NEOPIXEL_RGB_STRIP_TYPE;
#endif

NeoPixelController* strip;


void setup() {
  // put your setup code here, to run once:
  strip = new NeoPixelController(pin, loopLength, brightness, stripLength, stripType);
  strip->SetColorMode(NeoPixelController::kRainbowReverse);
  strip->SetBrightnessMode (NeoPixelController::kPSnake);
  strip->SetPixelsPerSegment(6);
}

void loop() {
  // put your main code here, to run repeatedly:
  strip->NeoPixelProcess ();
}

