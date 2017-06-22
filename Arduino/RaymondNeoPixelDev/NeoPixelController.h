#ifndef NEO_PIXEL_CONTROLLER_H_
#define NEO_PIXEL_CONTROLLER_H_

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>
#include <RGBConverter.h>

class NeoPixelController {
  public:
    NeoPixelController(uint32_t pin, float loopSeconds, uint8_t brightness, uint32_t stripLength);
    
    enum ColorMode {
      kNone, kRed, kGreen, kBlue, kWhite, kRainbow, kRainbowReverse, kOldCycle, kYellow, kQuasics, kBrown, kPurple
    };

    enum BrightnessMode {
      kOff, kOn, kBreathing, kBlinking, kDashed, kRollIn, kRollOut, kRolling, kSnakeIn, kSnakeOut, kSnake, kPSnake
    };

    void NeoPixelProcess ();
    void SetColorMode (ColorMode color);
    void SetBrightnessMode (BrightnessMode brightness);

    uint32_t GetStripLength();
    void SetLoopTime (float seconds);
    float LoopTime ();
    void SetMaxBrightness (float brightness);
    float GetMaxBrightness ();
    uint32_t GetPixelsPerSegment ();
    void SetPixelsPerSegment (uint32_t pixels);
    
  private:
    void SetRangeRGB (uint32_t startPixel, uint32_t endPixel, uint32_t red, uint32_t green, uint32_t blue) ;
    void SetRangeHSV(uint32_t startPixel, uint32_t endPixel, float hue, float saturation, float value);
    void CycleBrightnessData (uint32_t numberOfSpaces);
    void CycleColorData (uint32_t numberOfSpaces);
    void SetRangeBrightness(uint32_t first, uint32_t last, float brightnessLevel);
    
    uint32_t pixelsPerSegment = 6;
    float loopSeconds;
    const float deltaHueForCycle = 2.5;

    uint32_t modeIteration;
    float * brightnessData;
    float universalBrightness;

    ColorMode colorMode;
    BrightnessMode brightnessMode;
    bool colorInitialized;
    bool modeInitialized;
    
    Adafruit_NeoPixel* strip;
    RGBConverter rgbConverter;
};

#endif //NEO_PIXEL_CONTROLLER_H_
