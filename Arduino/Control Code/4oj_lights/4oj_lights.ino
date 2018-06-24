#include "TransitionButton.h"
#include <Adafruit_NeoPixel.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

const int NEOPIXEL_PIN = 6;           ///< Digital pin used to communicate with the NeoPixel strip.
const int PIXEL_STRIP_LENGTH = 14;    ///< Length of the NeoPixel strip.
const int BUTTON_PIN = 2;             ///< Pin connected to the button to cycle through lighting modes.

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
    counter = (counter + 1) % 4;
    Serial.print("Counter == ");
    Serial.print(counter);
    Serial.println();
  }

  if (counter % 4 == 0){
    colorWipe(strip.Color(0, 0, 255), 17);
    colorWipe(strip.Color(0, 0, 0), 17);
  }
  else if (counter % 4 == 1){
    colorWipe(strip.Color(255, 0, 0), 17);
    colorWipe(strip.Color(0, 0, 0), 17);
  }
  else if (counter % 4 == 2){
    colorWipe(strip.Color(255, 255, 255), 17);
    colorWipe(strip.Color(0, 0, 0), 17);
  }
  else if (counter % 4 == 3){
    colorWipe(strip.Color(255, 0, 0), 17);
    colorWipe(strip.Color(0, 0, 255), 17);
    colorWipe(strip.Color(255, 255, 255), 17);
    colorWipe(strip.Color(0, 0, 0), 17);
  }

}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

