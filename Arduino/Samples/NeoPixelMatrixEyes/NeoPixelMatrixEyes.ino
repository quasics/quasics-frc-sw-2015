// Adafruit_NeoMatrix example for single NeoPixel Shield.
// Draws a pair of eyes on an 8x32 NeoPixel matrix, shifting back and forth.
//
// For more info, check out https://learn.adafruit.com/adafruit-neopixel-uberguide/neomatrix-library
// and https://learn.adafruit.com/adafruit-gfx-graphics-library?view=all.

#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

#include "defs.h"

#ifndef PSTR
 #define PSTR // Make Arduino Due happy
#endif

#define PIN 6

// MATRIX DECLARATION:
// Parameter 1 = width of NeoPixel matrix
// Parameter 2 = height of matrix
// Parameter 3 = pin number (most are valid)
// Parameter 4 = matrix layout flags, add together as needed:
//   NEO_MATRIX_TOP, NEO_MATRIX_BOTTOM, NEO_MATRIX_LEFT, NEO_MATRIX_RIGHT:
//     Position of the FIRST LED in the matrix; pick two, e.g.
//     NEO_MATRIX_TOP + NEO_MATRIX_LEFT for the top-left corner.
//   NEO_MATRIX_ROWS, NEO_MATRIX_COLUMNS: LEDs are arranged in horizontal
//     rows or in vertical columns, respectively; pick one or the other.
//   NEO_MATRIX_PROGRESSIVE, NEO_MATRIX_ZIGZAG: all rows/columns proceed
//     in the same order, or alternate lines reverse direction; pick one.
//   See example below for these values in action.
// Parameter 5 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_GRBW    Pixels are wired for GRBW bitstream (RGB+W NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)


// Example for NeoPixel Shield.  In this application we'd like to use it
// as a 8x32 matrix.
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(32, 8, PIN,
  NEO_MATRIX_BOTTOM  + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);

const uint16_t colors[] = {
  matrix.Color(255, 0, 0), matrix.Color(0, 255, 0), matrix.Color(0, 0, 255) };

void setup() {
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(40);
  matrix.setTextColor(colors[0]);
}

void howdy(uint16_t color) {
  static const int kWidth = matrix.width();
  int x    = kWidth;
  int pass = 0;

  matrix.setTextColor(color);

  for(int i = 0; i < kWidth + 30; ++i) {
    matrix.fillScreen(0);
    matrix.setCursor(x, 0);
    matrix.print(F("Howdy"));
    if(--x < -36) {
      x = matrix.width();
      if(++pass >= 3) pass = 0;
      matrix.setTextColor(colors[pass]);
    }
    matrix.show();
    delay(100);
  }
  matrix.fillScreen(0);
}

uint16_t getPupilTop() {
  return PUPIL_TOP_AT_CENTER;
}

uint16_t getEyeWidthBounds(const int x, const int16_t atY, int16_t&x1, int16_t&x2) {
  if (atY == 0 || atY == 7) {
    x1 = x+2;
    x2 = x+5;
  }
  else if (atY == 1 || atY == 6) {
    x1 = x+1;
    x2 = x+6;
  }
  else {
    x1 = x;
    x2 = x + 7;
  }
}

void drawEyeOutline(const int x, const uint16_t color) {
  matrix.fillRect(x, 0, 8, 8, BLACK);

  // Framing "circle"
  matrix.drawLine(x+2, 0, x+5, 0, color);
  matrix.drawPixel(x+1, 1, color);
  matrix.drawPixel(x+6, 1, color);
  matrix.drawLine(x, 2, x, 5, color);
  matrix.drawLine(x+7, 2, x+7, 5, color);
  matrix.drawPixel(x+1, 6, color);
  matrix.drawLine(x+2, 7, x+5, 7, color);
  matrix.drawPixel(x+6, 6, color);
}

void drawPupil(const int x, const uint16_t color, PupilPlacement_t p) {
  const uint16_t kPupilTop = getPupilTop();
  switch(p) {
    case eCenter:
      matrix.drawRect(x+3, kPupilTop, 2, 2, color);
      break;
    case eNearLeft:
      matrix.drawRect(x+2, kPupilTop, 2, 2, color);
      break;
    case eLeft:
      matrix.drawRect(x+1, kPupilTop, 2, 2, color);
      break;
    case eFarLeft:
      matrix.drawRect(x+1, kPupilTop, 1, 2, color);
      break;
    case eNearRight:
      matrix.drawRect(x+4, kPupilTop, 2, 2, color);
      break;
    case eRight:
      matrix.drawRect(x+5, kPupilTop, 2, 2, color);
      break;
    case eFarRight:
      matrix.drawRect(x+6, kPupilTop, 1, 2, color);
      break;
  }
}

// Assumes an 8x8 eye
void drawEye(const int x, const uint16_t color, PupilPlacement_t p) {
  drawEyeOutline(x, color);
  drawPupil(x, color, p);
}

void drawEyes(uint16_t color, PupilPlacement_t p) {
  drawEye(LEFT_EYE_X, color, p);
  drawEye(RIGHT_EYE_X, color, p);
  matrix.show();
  delay(100);
}

void drawBlinkingEye(uint16_t x, uint16_t color, PupilPlacement_t p, LidHeight_t lidHeight) {
  const uint16_t kPupilTop = getPupilTop();
  if (lidHeight == eOpen) {
    drawEye(x, color, p);
  } else {
    drawEyeOutline(x, color);
    drawPupil(x, color, p);
    int16_t x1, x2;
    for(int16_t y = 1; y <= int16_t(lidHeight); ++y) {
      getEyeWidthBounds(x, y, x1, x2);
      matrix.drawLine(x1, y, x2, y, color);
    }
  }
}

void drawBlinkingEyes(uint16_t color, PupilPlacement_t p, LidHeight_t lidHeight) {
  drawBlinkingEye(LEFT_EYE_X, color, p, lidHeight);
  drawBlinkingEye(RIGHT_EYE_X, color, p, lidHeight);
  matrix.show();
}

void drawWinkingEyes(bool winkLeft, uint16_t color, PupilPlacement_t p, LidHeight_t lidHeight) {
  drawBlinkingEye(winkLeft ? LEFT_EYE_X : RIGHT_EYE_X, color, p, lidHeight);
  drawEye(!winkLeft ? LEFT_EYE_X : RIGHT_EYE_X, color, p);
  matrix.show();
}

void blinkingEyes(uint16_t color) {
  drawBlinkingEyes(color, eCenter, eOpen);
  delay(2000);
  drawBlinkingEyes(color, eCenter, eHigh);
  delay(250);
  drawBlinkingEyes(color, eCenter, eMiddle);
  delay(250);
  drawBlinkingEyes(color, eCenter, eLow);
  delay(250);
  drawBlinkingEyes(color, eCenter, eClosed);
  delay(500);
  drawBlinkingEyes(color, eCenter, eLow);
  delay(250);
  drawBlinkingEyes(color, eCenter, eMiddle);
  delay(250);
  drawBlinkingEyes(color, eCenter, eHigh);
  delay(250);
}

void winkingEyes(bool winkLeft, uint16_t color) {
  drawWinkingEyes(winkLeft, color, eCenter, eOpen);
  delay(2000);
  drawWinkingEyes(winkLeft, color, eCenter, eHigh);
  delay(250);
  drawWinkingEyes(winkLeft, color, eCenter, eMiddle);
  delay(250);
  drawWinkingEyes(winkLeft, color, eCenter, eLow);
  delay(250);
  drawWinkingEyes(winkLeft, color, eCenter, eClosed);
  delay(500);
  drawWinkingEyes(winkLeft, color, eCenter, eLow);
  delay(250);
  drawWinkingEyes(winkLeft, color, eCenter, eMiddle);
  delay(250);
  drawWinkingEyes(winkLeft, color, eCenter, eHigh);
  delay(250);
}

void shiftyEyes(uint16_t color) {
  drawEyes(color, eCenter);
  delay(1000);
  drawEyes(color, eNearLeft);
  drawEyes(color, eLeft);
  drawEyes(color, eFarLeft);
  delay(300);
  drawEyes(color, eLeft);
  drawEyes(color, eNearLeft);
  drawEyes(color, eCenter);
  delay(1000);
  drawEyes(color, eNearRight);
  drawEyes(color, eRight);
  drawEyes(color, eFarRight);
  delay(300);
  drawEyes(color, eRight);
  drawEyes(color, eNearRight);
  drawEyes(color, eCenter);
}

void loop() {
  howdy(GREEN);
  shiftyEyes(GREEN);
  // blinkingEyes(GREEN);
  winkingEyes(true, GREEN);
  shiftyEyes(GREEN);
}
