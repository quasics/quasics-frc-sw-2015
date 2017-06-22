#include "NeoPixelController.h"

void NeoPixelController::NeoPixelProcess () {
  switch (brightnessMode) {
    case kDashed: {
        uint32_t iterationMS = loopSeconds * 1000 / strip->numPixels();
        static uint32_t lastIteration = 0;
        if (pixelsPerSegment < strip->numPixels()) {
          if (millis() >= lastIteration + iterationMS) {
            lastIteration = millis();
            if (!modeInitialized) {
              SetRangeBrightness (0, strip->numPixels() - 1, 0);
              int numberOfSegments = strip->numPixels() / pixelsPerSegment;
              for (int i = 0; i < numberOfSegments; i++) {
                if (i * pixelsPerSegment + pixelsPerSegment / 2 - 1 < strip->numPixels()) {
                  SetRangeBrightness(i * pixelsPerSegment, i * pixelsPerSegment + pixelsPerSegment / 2 - 1, 1);
                }
                if (i * pixelsPerSegment + pixelsPerSegment - 1 < strip->numPixels()) {
                  SetRangeBrightness(i * pixelsPerSegment + pixelsPerSegment / 2, i * pixelsPerSegment + pixelsPerSegment / 2 - 1, 0);
                } else {
                  SetRangeBrightness(i * pixelsPerSegment, strip->numPixels() - 1, 0);
                }
                modeInitialized = true;
              }
            } else {
              CycleBrightnessData(1);
            }
          }
        } else {
          SetRangeBrightness (0, strip->numPixels() - 1, 1);
          brightnessMode = kOn;
        }

      }
      break;
    case kBlinking: {
        if (millis() % int(loopSeconds * 1000 + .5) < (loopSeconds * 1000 + .5) / 2) {
          SetRangeBrightness(0, strip->numPixels() - 1, 0);
        } else {
          SetRangeBrightness(0, strip->numPixels() - 1, 1);
        }
      }
      break;
    case kRollIn: {
        int iterationMS = loopSeconds * 1000 * 2 * 2 / strip->numPixels() / (strip->numPixels() + 1);
        static uint32_t lastIterationMillis = 0;

        if (millis() >= lastIterationMillis + iterationMS) {
          lastIterationMillis = millis();
          static uint32_t pixelsFilled = 1;
          static uint32_t iteration = 0;
          SetRangeBrightness(0, strip->numPixels() - pixelsFilled, 0);
          SetRangeBrightness(iteration, iteration, 1);

          if (iteration >= strip->numPixels() - pixelsFilled) {
            iteration = 0;
            if (pixelsFilled >= strip->numPixels()) {
              brightnessMode = kOn;
              pixelsFilled = 1;
            } else {
              pixelsFilled++;
            }
          } else {
            iteration++;
          }
        }
      }
      break;
    case kRollOut: {
        int iterationMS = loopSeconds * 1000 * 2 * 2 / strip->numPixels() / (strip->numPixels() + 1);
        static uint32_t lastIterationMillis = 0;

        if (millis() >= lastIterationMillis + iterationMS) {
          lastIterationMillis = millis();
          static bool iterated = false;
          static uint32_t pixelsFilled = 1;
          static uint32_t iteration = 0;
          SetRangeBrightness(0, strip->numPixels() - pixelsFilled, 1);
          SetRangeBrightness(iteration, iteration, 0);

          if (iteration >= strip->numPixels() - pixelsFilled) {
            iteration = 0;
            if (pixelsFilled >= strip->numPixels()) {
              brightnessMode = kOff;
              pixelsFilled = 1;
            } else {
              pixelsFilled++;
            }
          } else {
            iteration++;
          }
        }
      }
      break;
    case kRolling: {
        int iterationMS = loopSeconds * 1000 * 2 * 2 / strip->numPixels() / (strip->numPixels() + 1);
        static uint32_t lastIterationMillis = 0;
        static uint32_t stage = 0;
        if (millis() >= lastIterationMillis + iterationMS) {
          lastIterationMillis = millis();
          if (stage == 0) {
            static uint32_t pixelsFilled = 1;
            static uint32_t iteration = 0;
            SetRangeBrightness(0, strip->numPixels() - pixelsFilled, 0);
            SetRangeBrightness(iteration, iteration, 1);

            if (iteration >= strip->numPixels() - pixelsFilled) {
              iteration = 0;
              if (pixelsFilled >= strip->numPixels()) {
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
            SetRangeBrightness(0, strip->numPixels() - pixelsFilled, 1);
            SetRangeBrightness(iteration, iteration, 0);

            if (iteration >= strip->numPixels() - pixelsFilled) {
              iteration = 0;
              if (pixelsFilled >= strip->numPixels()) {
                stage = 0;
                pixelsFilled = 1;
              } else {
                pixelsFilled++;
              }
            } else {
              iteration++;
            }
          }
        }
      }
      break;
    case kBreathing: {
        float timeLoop = float(millis() % int(loopSeconds * 1000)) / 1000;
        float timeHalf = float(millis() % int(loopSeconds / 2 * 1000)) / 1000;

        if (timeLoop <= loopSeconds / 2)
          SetRangeBrightness( 0, strip->numPixels() - 1, (timeHalf * timeHalf) / (loopSeconds / 2 * loopSeconds / 2));
        else
          SetRangeBrightness( 0, strip->numPixels() - 1, ((loopSeconds / 2 * loopSeconds / 2) - (timeHalf * timeHalf)) /
                              (loopSeconds / 2 * loopSeconds / 2));
      }
      break;
    case kOff: {
        SetRangeBrightness (0, strip->numPixels() - 1, 0);
      }
      break;
    //kSnakeIn, kSnakeOut, kSnake
    case kSnakeIn: {
        uint32_t itterationMS = loopSeconds * 1000 / strip->numPixels();
        static uint32_t pixelsFilled = 1;
        static uint32_t lastItterationMS = millis();
        if (!modeInitialized) {
          pixelsFilled = 1;
          lastItterationMS = millis();
          modeInitialized = true;
          SetRangeBrightness(1, strip->numPixels(), 0);
        }
        if (millis() >= itterationMS + lastItterationMS) {
          pixelsFilled++;
          lastItterationMS = millis();
          SetRangeBrightness (0, pixelsFilled - 1, 1);
        }
        if (pixelsFilled >= strip->numPixels()) {
          brightnessMode = kOn;
          pixelsFilled = 0;
        }
      }
      break;
    case kSnakeOut: {
        uint32_t itterationMS = loopSeconds * 1000 / strip->numPixels();
        static uint32_t pixelsFilled = 1;
        static uint32_t lastItterationMS = millis();
        if (!modeInitialized) {
          pixelsFilled = 1;
          lastItterationMS = millis();
          modeInitialized = true;
          SetRangeBrightness(1, strip->numPixels(), 1);
        }
        if (millis() >= itterationMS + lastItterationMS) {
          pixelsFilled++;
          lastItterationMS = millis();
          SetRangeBrightness (0, pixelsFilled - 1, 0);
        }
        if (pixelsFilled >= strip->numPixels()) {
          brightnessMode = kOff;
          pixelsFilled = 0;
        }
      }
      break;
    case kSnake: {
        uint32_t itterationMS = loopSeconds * 1000 / strip->numPixels();
        static uint32_t pixelsFilled = 1;
        static uint32_t lastItterationMS = millis();
        static uint8_t stage = 0;
        if (stage == 0) {
          if (!modeInitialized) {
            pixelsFilled = 1;
            lastItterationMS = millis();
            modeInitialized = true;
            SetRangeBrightness(1, strip->numPixels(), 0);
          }
          if (millis() >= itterationMS + lastItterationMS) {
            pixelsFilled++;
            lastItterationMS = millis();
            SetRangeBrightness (0, pixelsFilled - 1, 1);
          }
          if (pixelsFilled >= strip->numPixels()) {
            stage = 1;
            pixelsFilled = 1;
            modeInitialized = false;
          }
        } else {
          if (!modeInitialized) {
            pixelsFilled = 1;
            lastItterationMS = millis();
            modeInitialized = true;
            SetRangeBrightness(1, strip->numPixels(), 1);
          }
          if (millis() >= itterationMS + lastItterationMS) {
            pixelsFilled++;
            lastItterationMS = millis();
            SetRangeBrightness (0, pixelsFilled - 1, 0);
          }
          if (pixelsFilled >= strip->numPixels()) {
            stage = 0;
            pixelsFilled = 1;
            modeInitialized = false;
          }
        }
      }
      break;
    case kPSnake: {
        uint32_t itterationMS = loopSeconds * 1000 / strip->numPixels();
        static uint32_t pixelsFilled = 1;
        static uint32_t lastItterationMS = millis();
        static uint8_t stage = 0;
        if (stage == 0) {
          if (!modeInitialized) {
            pixelsFilled = 1;
            lastItterationMS = millis();
            modeInitialized = true;
            SetRangeBrightness(1, strip->numPixels(), 0);
          }
          if (millis() >= itterationMS + lastItterationMS) {
            pixelsFilled++;
            lastItterationMS = millis();
            SetRangeBrightness (0, pixelsFilled - 1, 1);
          }
          if (pixelsFilled >= strip->numPixels()) {
            stage = 1;
            pixelsFilled = 1;
            modeInitialized = false;
            lastItterationMS = millis();
          }
        } else if (stage == 1) {
          if (lastItterationMS + loopSeconds * 1000 <= millis()) {
            stage = 2;
            lastItterationMS = millis();
          }
        } else {
          if (!modeInitialized) {
            pixelsFilled = 1;
            lastItterationMS = millis();
            modeInitialized = true;
            SetRangeBrightness(1, strip->numPixels(), 1);
          }
          if (millis() >= itterationMS + lastItterationMS) {
            pixelsFilled++;
            lastItterationMS = millis();
            SetRangeBrightness (0, pixelsFilled - 1, 0);
          }
          if (pixelsFilled >= strip->numPixels()) {
            stage = 0;
            pixelsFilled = 1;
            modeInitialized = false;
          }
        }
      }
      break;
    case kOn:
    default: {
        SetRangeBrightness (0, strip->numPixels() - 1, 1);
      }
      break;
  }

  //------------------------------------------------------------Color Mode Switch------------------------------------------------------------
  switch (colorMode) {
    case kPurple: {
        SetRangeRGB(0, strip->numPixels() - 1, 192, 0, 255);
      }
      break;
    case kRainbowReverse: {
        float baseHue = float(millis() % int(loopSeconds * 1000)) * -360 / loopSeconds / 1000;

        while (baseHue < 0) {
          baseHue = 360 - fabs(baseHue);
        }


        for (uint32_t pixel = 0; pixel <= strip->numPixels() - 1; pixel++) {
          SetRangeHSV(pixel, pixel, baseHue +  pixel * 360 / strip->numPixels(), 1, 1);
        }
      }
      break;
    case kBrown:
      SetRangeRGB(0, strip->numPixels() - 1, 54, 27, 0);
      break;
    case kQuasics:
      SetRangeRGB(0, strip->numPixels() - 1, 6 * .25, 50 * .25, 3.25 * .25);
      break;
    case kYellow:
      SetRangeRGB(0, strip->numPixels() - 1, 255, 255, 0);
      break;
    case kRed: {
        SetRangeRGB(0, strip->numPixels() - 1, 255, 0, 0);
      }
      break;
    case kGreen: {
        SetRangeRGB(0, strip->numPixels() - 1, 0, 255, 0);
      }
      break;
    case kBlue: {
        SetRangeRGB(0, strip->numPixels() - 1, 0, 0, 255);
      }
      break;
    case kWhite: {
        SetRangeRGB(0, strip->numPixels() - 1, 255, 255, 255);
      }
      break;
    case kRainbow: {
        float baseHue = float(millis() % int(loopSeconds * 1000)) * 360 / loopSeconds / 1000;

        while (baseHue < 0) {
          baseHue = 360 - fabs(baseHue);
        }


        for (uint32_t pixel = 0; pixel <= strip->numPixels() - 1; pixel++) {
          SetRangeHSV(pixel, pixel, baseHue +  pixel * 360 / strip->numPixels(), 1, 1);
        }
      }
      break;
    case kOldCycle: {
        SetRangeHSV(0, strip->numPixels() - 1, float(millis() % int(loopSeconds * 1000)) * 360 / loopSeconds / 1000, 1, 1);
      }
      break;
    default: {
        SetRangeRGB(0, strip->numPixels() - 1, 0, 0, 0);
      }
      break;
  }

  modeIteration++;
  strip->show();
}

