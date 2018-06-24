#include "TransitionButton.h"
#include <Adafruit_NeoPixel.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

const int NEOPIXEL_PIN = 6;           ///< Digital pin used to communicate with the NeoPixel strip.
const int PIXEL_STRIP_LENGTH = 14;    ///< Length of the NeoPixel strip.
const int BUTTON_PIN = 2;             ///< Pin connected to the button to cycle through lighting modes.

const uint32_t WHITE  = Adafruit_NeoPixel::Color(255, 255, 255, 0);
const uint32_t BLACK  = Adafruit_NeoPixel::Color(0, 0, 0, 0);
const uint32_t RED    = Adafruit_NeoPixel::Color(255, 0, 0, 0);
const uint32_t BLUE   = Adafruit_NeoPixel::Color(0, 0, 255, 0);
const uint32_t QUASICS_GREEN = Adafruit_NeoPixel::Color(40, 255, 0, 0);

/// NeoPixel strip control.
Adafruit_NeoPixel strip(PIXEL_STRIP_LENGTH, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/// Button to control stepping through the different lighting modes.
TransitionButton button(BUTTON_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Starting up!\n");

  strip.begin();
  strip.show();  
}

void loop() {
  static int counter = 0;
  if (button.advancePosition()) {
    Serial.println("Advancing...");
    Serial.print("Counter == ");
    Serial.print(counter);
    Serial.println();
  }

  const int standardDelay = 17;

  switch(counter) {
    case 0:
      simplePattern(BLUE, standardDelay);
      break;
    case 1:
      simplePattern(RED, standardDelay);
      break;
    case 2:
      simplePattern(WHITE, standardDelay);
      break;
    case 3:
      // Cycling red, blue, and white (for 4th of July)
      colorWipe(RED, standardDelay);
      colorWipe(BLUE, standardDelay);
      colorWipe(WHITE, standardDelay);
      colorWipe(BLACK, standardDelay);
      break;
    default:
      // If it's not a color being used, then reset back to 0.
      counter = 0;
      Serial.print("Resetting counter");
      break;
  }
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void simplePattern(uint32_t color, uint8_t wait) {
  colorWipe(color, wait);
  colorWipe(BLACK, wait);
}

