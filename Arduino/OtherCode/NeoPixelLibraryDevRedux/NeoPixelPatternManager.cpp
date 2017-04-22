/*
   NeoPixelPatternManager.cpp

    Created on: Feb 22, 2017
        Author: Yogna
*/

#include "NeoPixelPatternManager.h"

NeoPixelPatternManager::NeoPixelPatternManager(uint32_t stripLength,
    uint32_t pin, uint32_t brightnessInit, bool isRGBW) {
  // TODO Auto-generated constructor stub
  mode = kDefaultMode;
  pattern = kDefaultPattern;
  evolution = kNoEvolution;
  inPattern = true;
  inMode = true;
  inEvolution = false;

  strip = new NeoPixelController(stripLength, pin, brightnessInit, isRGBW);

  lastPixel = strip->GetLength();
  halfPixel = uint32_t(lastPixel / 2 + .5);

  modeIteration = 0;

  HSVTranslateFirstPixel = 0;
  HSVTranslateLastPixel = 0;
  HSVTranslateHue = 0;
  HSVTranslateSaturation = 0;
  HSVTranslateValue = 0;
  HSVTranslateIterations = 0;
  HSVTranslateCounter = 0;
}

NeoPixelPatternManager::~NeoPixelPatternManager() {
  // TODO Auto-generated destructor stub
  delete strip;
  strip = NULL;
}

void NeoPixelPatternManager::Process() {
  switch (mode) {
    case kCycle:
      for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
        strip->SetRangeHSV(pixel, pixel,
                           float(
                             int(modeIteration + 360 / (lastPixel + 1) * pixel)
                             % 360) / 360, 1, 1);
      }
      modeIteration = ((modeIteration) % 360) + 1;
      break;
    case kRed:
      strip->SetStripColor(255, 0, 0);
      break;
    case kGreen:
      strip->SetStripColor(0, 255, 0);
      break;
    case kBlue:
      strip->SetStripColor(0, 0, 255);
      break;
    case kRainbow: {

        /*float baseHue = modeIteration % 360;
          float deltaHue = float(float(360) / float(lastPixel));
          for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
        	strip->SetRangeHSV(pixel, pixel, deltaHue * pixel + baseHue, 1, 1);
          }
          modeIteration++;*/
        for (uint32_t pixel = 0; pixel <= lastPixel; pixel++) {
          strip->SetRangeHSV(pixel, pixel,
                             float(int(360 / (lastPixel + 1) * pixel) % 360) / 360, 1,
                             strip->GetPixelBrightness(pixel));
        }
      }
      break;
    case kGreenToWhite:
      if (StripHSVTranslation()) {
        mode = kWhite;
      }
      break;
    default:
      strip->SetStripColor(255, 255, 255);
  }

  switch (pattern) {
    case kRollIn:
      static uint32_t pixelsFilled = 1;
      static uint32_t iteration = 0;
      strip->SetRangeBrightness(0, strip->GetLength() - pixelsFilled, 0);
      strip->SetRangeBrightness(iteration, iteration, 1);

      if (iteration >= strip->GetLength() - pixelsFilled) {
        iteration = 0;
        if (pixelsFilled >= strip->GetLength()) {
          inPattern = false;
          pattern = kDefaultPattern;
          pixelsFilled = 1;
        } else {
          pixelsFilled++;
        }
      } else {
        iteration++;
      }
      break;
    case kRollOut:
      strip->SetRangeBrightness(0, strip->GetLength() - pixelsFilled, 1);
      strip->SetRangeBrightness(iteration, iteration, 0);

      if (iteration >= strip->GetLength() - pixelsFilled) {
        iteration = 0;
        if (pixelsFilled >= strip->GetLength()) {
          inPattern = false;
          pattern = kDefaultPattern;
          pixelsFilled = 1;
        } else {
          pixelsFilled++;
        }
      } else {
        iteration++;
      }
      break;
    case kRolling:
      static uint32_t stage = 0;
      if (stage == 0) {
        static uint32_t pixelsFilled = 1;
        static uint32_t iteration = 0;
        strip->SetRangeBrightness(0, strip->GetLength() - pixelsFilled, 0);
        strip->SetRangeBrightness(iteration, iteration, 1);

        if (iteration >= strip->GetLength() - pixelsFilled) {
          iteration = 0;
          if (pixelsFilled >= strip->GetLength()) {
            stage = 1;
            pixelsFilled = 1;
          } else {
            pixelsFilled++;
          }
        } else {
          iteration++;
        }
      } else if (stage == 1) {
        static uint32_t pixelsFilled = 1;
        static uint32_t iteration = 0;
        strip->SetRangeBrightness(0, strip->GetLength() - pixelsFilled, 1);
        strip->SetRangeBrightness(iteration, iteration, 0);

        if (iteration >= strip->GetLength() - pixelsFilled) {
          iteration = 0;
          if (pixelsFilled >= strip->GetLength()) {
            stage = 0;
            pixelsFilled = 1;
          } else {
            pixelsFilled++;
          }
        } else {
          iteration++;
        }
      }
      break;
    case kOff:
      strip->SetStripBrightness(0);
      inPattern = false;
      break;
    default:
      strip->SetStripBrightness(1);
      inPattern = false;
      break;
  }

  strip->Show();
}

void NeoPixelPatternManager::Evolve() {
  switch (evolution) {
    case kRollingCycle:
      static uint32_t stage = 0;

      static uint32_t startTime = 0;
      switch (stage) {
        case 0:
          static bool initialized = false;
          if (!initialized) {
            SetMode(NeoPixelPatternManager::kRainbow);
            SetPattern(NeoPixelPatternManager::kRollIn);
            initialized = true;
          }
          delay(25);
          if (!inPattern) {
            stage = 1;
            initialized = false;
          }
          break;
        case 1:
          if (!initialized) {
            SetMode(NeoPixelPatternManager::kCycle);
            SetPattern(NeoPixelPatternManager::kOn);
            initialized = true;
            startTime = millis();
          }
          if (millis() - startTime >= 6775) {
            stage = 2;
            initialized = false;
          }
          break;
        case 2:
          if (!initialized) {
            SetMode(NeoPixelPatternManager::kRainbow);
            SetPattern(NeoPixelPatternManager::kRollOut);
            initialized = true;
          }
          delay(25);
          if (!inPattern) {
            stage = 0;
            initialized = false;
          }
          break;
        default:
          stage = 0;
          initialized = false;
      }
      break;
    default:
      break;
  }

  Process();
}

void NeoPixelPatternManager::SetPattern(Pattern whichPattern) {
  pattern = whichPattern;
  inPattern = true;
}

void NeoPixelPatternManager::SetMode(Mode whichMode) {
  mode = whichMode;
  if (mode == kGreenToWhite) {
    strip->SetStripHSV(120, 1, 1);
    StripHSVTranslation(120, 0, 1, hsvTranslationSteps);
  }
  modeIteration = 0;
}

void NeoPixelPatternManager::SetEvolution(Evolution whichEvolution) {
  evolution = whichEvolution;
}

bool NeoPixelPatternManager::InPattern() {
  return inPattern;
}

bool NeoPixelPatternManager::InMode() {
  return inMode;
}

bool NeoPixelPatternManager::InEvolution() {
  return inEvolution;
  inEvolution = true;
}

NeoPixelPatternManager::Pattern NeoPixelPatternManager::GetPattern() {
  return pattern;
}

NeoPixelPatternManager::Mode NeoPixelPatternManager::GetMode() {
  return mode;
}

NeoPixelPatternManager::Evolution NeoPixelPatternManager::GetEvolution() {
  return evolution;
}

void NeoPixelPatternManager::RangeHSVTranslation(uint32_t firstPixel,
    uint32_t finalPixel, float finalHue, float finalSaturation,
    float finalValue, uint32_t numberOfIterations) {
  HSVTranslateFirstPixel = firstPixel;
  HSVTranslateLastPixel = finalPixel;
  HSVTranslateHue = finalHue;
  HSVTranslateSaturation = finalSaturation;
  HSVTranslateValue = finalValue;
  HSVTranslateIterations = numberOfIterations;
  HSVTranslateCounter = 0;

  for (uint32_t pixel = HSVTranslateFirstPixel;
       pixel <= HSVTranslateLastPixel; pixel++) {
    float currentHue = 0;
    float currentSaturation = 0;
    float currentValue = 0;
    strip->ColorCodeToHSV(strip->GetPixelColor(pixel), currentHue,
                          currentSaturation, currentValue);

    strip->SetRangeHSV(HSVTranslateFirstPixel, HSVTranslateLastPixel,
                       (HSVTranslateHue - currentHue)
                       / (HSVTranslateIterations - HSVTranslateCounter),
                       (HSVTranslateSaturation - currentSaturation)
                       / (HSVTranslateIterations - HSVTranslateCounter),
                       (HSVTranslateValue - currentValue)
                       / (HSVTranslateIterations - HSVTranslateCounter));
  }
  HSVTranslateCounter++;
}

bool NeoPixelPatternManager::RangeHSVTranslation() {
  if (HSVTranslateCounter <= HSVTranslateIterations) {
    for (uint32_t pixel = HSVTranslateFirstPixel;
         pixel <= HSVTranslateLastPixel; pixel++) {
      float currentHue = 0;
      float currentSaturation = 0;
      float currentValue = 0;
      strip->ColorCodeToHSV(strip->GetPixelColor(pixel), currentHue,
                            currentSaturation, currentValue);

      strip->SetRangeHSV(HSVTranslateFirstPixel, HSVTranslateLastPixel,
                         (HSVTranslateHue - currentHue)
                         / (HSVTranslateIterations - HSVTranslateCounter),
                         (HSVTranslateSaturation - currentSaturation)
                         / (HSVTranslateIterations - HSVTranslateCounter),
                         (HSVTranslateValue - currentValue)
                         / (HSVTranslateIterations - HSVTranslateCounter));
    }
    return false;
    HSVTranslateCounter++;
  } else {
    return true;
  }
}

void NeoPixelPatternManager::StripHSVTranslation(float finalHue,
    float finalSaturation, float finalValue, uint32_t numberOfIterations) {
  RangeHSVTranslation(0, lastPixel, finalHue, finalSaturation, finalValue,
                      numberOfIterations);
}

bool NeoPixelPatternManager::StripHSVTranslation() {
  return RangeHSVTranslation();
}
