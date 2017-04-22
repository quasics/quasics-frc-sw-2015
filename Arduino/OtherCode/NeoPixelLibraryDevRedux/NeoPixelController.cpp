#include "NeoPixelController.h"

//--------------------------------------Constructor and Destructor---------------------------------------
NeoPixelController::NeoPixelController(uint32_t stripLength, uint32_t pin,
                                       uint32_t brightnessInit, bool isRGBW) {
  //Neopixel object Setup
  if (isRGBW) {
    strip = new Adafruit_NeoPixel(stripLength, pin, NEO_GRBW + NEO_KHZ800);
    isGRBW = true;
  } else {
    strip = new Adafruit_NeoPixel(stripLength, pin, NEO_GRB + NEO_KHZ800);
    isGRBW = false;
  }

  strip->begin();
  strip->setBrightness(brightnessInit);
  strip->show();

  //Color & brightness aray setup
  colorData = new uint32_t[stripLength];
  brightnessData = new float[stripLength];
  for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
    colorData[pixel] = strip->Color(0, 0, 0);
  }
  for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
    brightnessData[pixel] = 1;
  }

  //Store the physical parameters about the strip
  lastPixel = stripLength - 1;
  halfPixel = uint32_t(lastPixel / 2 + .5);


}

NeoPixelController::~NeoPixelController() {
  delete[] colorData;
  colorData = NULL;
  delete strip;
  strip = NULL;
  delete[] brightnessData;
  brightnessData = NULL;
}

//--------------------------------------Basic Control Mechanisms-----------------------------------------
void NeoPixelController::SetStripColor(uint32_t red, uint32_t green,
                                       uint32_t blue) {
  SetRangeColor(0, lastPixel, red, green, blue);
}

void NeoPixelController::SetStripHSV(float h, float s, float v) {
  SetRangeHSV(0, lastPixel, h, s, v);
}

void NeoPixelController::SetRangeColor(uint32_t startPixel, uint32_t endPixel,
                                       uint32_t r, uint32_t g, uint32_t b) {
  if ((endPixel <= lastPixel)) {
    for (uint32_t pixel = startPixel; pixel <= endPixel; pixel++) {
      uint32_t combined = strip->Color(
                            r * 255 * brightnessData[pixel],
                            g * 255 * brightnessData[pixel],
                            b * 255 * brightnessData[pixel]);
      strip->setPixelColor(pixel, combined);
      colorData[pixel] = combined;
    }
  } else {
    for (uint32_t pixel = startPixel; pixel <= lastPixel; pixel++) {
      uint32_t combined = strip->Color(
                            r * 255 * brightnessData[pixel],
                            g * 255 * brightnessData[pixel],
                            b * 255 * brightnessData[pixel]);
      strip->setPixelColor(pixel, combined);
      colorData[pixel] = combined;
    }
    for (uint32_t pixel = 0; pixel <= endPixel % lastPixel - 1;
         pixel++) {
      uint32_t combined = strip->Color(
                            r * 255 * brightnessData[pixel],
                            g * 255 * brightnessData[pixel],
                            b * 255 * brightnessData[pixel]);
      strip->setPixelColor(pixel, combined);
      colorData[pixel] = combined;
    }
  }
}

void NeoPixelController::SetRangeHSV(uint32_t startPixel, uint32_t endPixel,
                                     float h, float s, float v) {
  uint32_t r = 0;
  uint32_t g = 0;
  uint32_t b = 0;
  HSVToRGB(h, s, v, r, g, b);

  if ((endPixel <= lastPixel)) {
    for (uint32_t pixel = startPixel; pixel <= endPixel; pixel++) {
      uint32_t combined = strip->Color(
                            r * 255 * brightnessData[pixel],
                            g * 255 * brightnessData[pixel],
                            b * 255 * brightnessData[pixel]);
      strip->setPixelColor(pixel, combined);
      colorData[pixel] = combined;
    }
  } else {
    for (uint32_t pixel = startPixel; pixel <= lastPixel; pixel++) {
      uint32_t combined = strip->Color(
                            r * 255 * brightnessData[pixel],
                            g * 255 * brightnessData[pixel],
                            b * 255 * brightnessData[pixel]);
      strip->setPixelColor(pixel, combined);
      colorData[pixel] = combined;
    }
    for (uint32_t pixel = 0; pixel <= endPixel % lastPixel - 1;
         pixel++) {
      uint32_t combined = strip->Color(
                            r * 255 * brightnessData[pixel],
                            g * 255 * brightnessData[pixel],
                            b * 255 * brightnessData[pixel]);
      strip->setPixelColor(pixel, combined);
      colorData[pixel] = combined;
    }

  }
}

void NeoPixelController::SetStripBrightness(float brightnessLevel) {
  for (uint32_t pixel = 0; pixel <= lastPixel; pixel++)
    brightnessData[pixel] = brightnessLevel;
}

void NeoPixelController::SetRangeBrightness(uint32_t first, uint32_t last,
    float brightnessLevel) {
  for (uint32_t pixel = first; pixel <= last; pixel++)
    brightnessData[pixel] = brightnessLevel;
}

void NeoPixelController::AdvanceStripSetting(int numberOfPlaces) {
  bool isForward = true;
  if (numberOfPlaces < 0)
    isForward = false;

  uint32_t step = abs(numberOfPlaces) % lastPixel;

  if (isForward) {
    for (uint32_t i = 1; i <= step; i++) {
      uint32_t overflowValue = colorData[lastPixel];
      float overflowBrightness = brightnessData[lastPixel];

      for (uint32_t pixel = 1; pixel <= lastPixel; pixel++) {
        colorData[pixel] = colorData[pixel - 1];
        strip->setPixelColor(pixel, colorData[pixel]);
        brightnessData[pixel] = brightnessData[pixel - 1];
      }
      colorData[0] = overflowValue;
      strip->setPixelColor(0, colorData[0]);
      brightnessData[0] = overflowBrightness;
    }
  } else {
    for (uint32_t i = 1; i <= step; i++) {
      uint32_t overflowValue = colorData[0];
      float overflowBrightness = brightnessData[lastPixel];

      for (uint32_t pixel = 0; pixel < lastPixel; pixel++) {
        colorData[pixel] = colorData[pixel + 1];
        strip->setPixelColor(pixel, colorData[pixel]);
        brightnessData[pixel] = brightnessData[pixel + 1];
      }
      colorData[lastPixel] = overflowValue;
      strip->setPixelColor(lastPixel, colorData[lastPixel]);
      brightnessData[lastPixel] = overflowBrightness;
    }
  }
}

void NeoPixelController::Show() {
  strip->show();
}

//--------------------------------Data Return Functions--------------------------------------------------
uint32_t NeoPixelController::GetPixelColor(uint32_t pixel) {
  return colorData[pixel];
}

float NeoPixelController::GetPixelBrightness(uint32_t pixel) {
  return brightnessData[pixel];
}

uint32_t NeoPixelController::GetLength() {
  return lastPixel + 1;
}

//-----------------------------------<Converters>--------------------------------------------------------
void NeoPixelController::HSVToRGB(float hue, float saturation, float value,
                                  uint32_t& red, uint32_t& green, uint32_t& blue) {
  double r = 0;
  double g = 0;
  double b = 0;

  int i = int(hue * 6);
  double f = hue * 6 - i;
  double p = value * (1 - saturation);
  double q = value * (1 - f * saturation);
  double t = value * (1 - (1 - f) * saturation);

  switch (i % 6) {
    case 0:
      r = value;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = value;
      b = p;
      break;
    case 2:
      r = p;
      g = value;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = value;
      break;
    case 4:
      r = t;
      g = p;
      b = value;
      break;
    case 5:
      r = value;
      g = p;
      b = q;
      break;
  }

  red = uint32_t(r * 255);
  green = uint32_t(g * 255);
  blue = uint32_t(b * 255);

}

void NeoPixelController::RGBToHSV(uint32_t red, uint32_t green, uint32_t blue,
                                  float& h, float& s, float& v) {
  int i;

  float r = float(red) / 255;
  float g = float(green) / 255;
  float b = float(blue) / 255;

  float f, p, q, t;
  if (s == 0) {
    // achromatic (grey)
    r = g = b = v;
    return;
  }
  h /= 60;			// sector 0 to 5
  i = floor(h);
  f = h - i;			// factorial part of h
  p = v * (1 - s);
  q = v * (1 - s * f);
  t = v * (1 - s * (1 - f));
  switch (i) {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    default:		// case 5:
      r = v;
      g = p;
      b = q;
      break;
  }
}

void NeoPixelController::ColorCodeToRGB(uint32_t code, uint32_t& red,
                                        uint32_t& green, uint32_t& blue) {
  for (int i = 0; 1 <= 7; i++) {
    bitWrite(red, i % 8, bitRead(code, i));
    bitWrite(green, i % 8, bitRead(code, i + 8));
    bitWrite(blue, i % 8, bitRead(code, i + 16));
  }
}

void NeoPixelController::ColorCodeToHSV(uint32_t code, float& hue,
                                        float& saturation, float& value) {
  uint32_t red = 0;
  uint32_t green = 0;
  uint32_t blue = 0;
  ColorCodeToRGB(code, red, green, blue);
}
