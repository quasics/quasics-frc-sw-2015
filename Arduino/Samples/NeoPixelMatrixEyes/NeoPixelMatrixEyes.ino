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

void howdy() {
  static int x    = matrix.width();
  static int pass = 0;

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

#define PUPIL_TOP_AT_CENTER 3

// Assumes an 8x8 eye
void drawEye(const int x, const uint16_t color, Placement_t p) {
  matrix.fillRect(x, 0, 8, 8, BLACK);

  // Framing "circle"
  matrix.drawLine(x+2, 0, x+5, 0, color);
  matrix.drawLine(x+2, 7, x+5, 7, color);
  matrix.drawLine(x, 2, x, 5, color);
  matrix.drawLine(x+7, 2, x+7, 5, color);
  matrix.drawPixel(x+1, 1, color);
  matrix.drawPixel(x+6, 1, color);
  matrix.drawPixel(x+1, 6, color);
  matrix.drawPixel(x+6, 6, color);

  switch(p) {
    case eCenter:
      matrix.drawRect(x+3, PUPIL_TOP_AT_CENTER, 2, 2, color);
      break;
    case eNearLeft:
      matrix.drawRect(x+2, PUPIL_TOP_AT_CENTER, 2, 2, color);
      break;
    case eLeft:
      matrix.drawRect(x+1, PUPIL_TOP_AT_CENTER, 2, 2, color);
      break;
    case eFarLeft:
      matrix.drawRect(x+1, PUPIL_TOP_AT_CENTER, 1, 2, color);
      break;
    case eNearRight:
      matrix.drawRect(x+4, PUPIL_TOP_AT_CENTER, 2, 2, color);
      break;
    case eRight:
      matrix.drawRect(x+5, PUPIL_TOP_AT_CENTER, 2, 2, color);
      break;
    case eFarRight:
      matrix.drawRect(x+6, PUPIL_TOP_AT_CENTER, 1, 2, color);
      break;
  }
}

void drawEyes(uint16_t color, Placement_t p) {
  drawEye(0, color, p);
  drawEye(24, color, p);
  matrix.show();
  delay(100);
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
  // howdy();
  shiftyEyes(GREEN);
}
