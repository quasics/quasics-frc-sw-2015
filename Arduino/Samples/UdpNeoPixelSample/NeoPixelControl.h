#ifndef NEOPIXEL_CONTROL_H
#define NEOPIXEL_CONTROL_H

#include <Adafruit_NeoPixel.h>

constexpr neoPixelType NEOPIXEL_RING_RGBW = NEO_GRBW + NEO_KHZ800;
constexpr neoPixelType NEOPIXEL_STRIP_RGB = NEO_GRB;
constexpr neoPixelType NEOPIXEL_WS2811_RGB = NEO_RGB;

enum class NeoPixelMode {
  eOff,
  eOn
};

void initializeNeoPixels(int pin, int length, neoPixelType type);

void setNeoPixelMode(NeoPixelMode newMode);

void stepNeoPixels();

#endif  // NEOPIXEL_CONTROL_H

