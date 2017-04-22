#include <Adafruit_NeoPixel.h>

const uint32_t stripLength = 15;
const uint32_t pin = 6;
const uint32_t lastPixel = stripLength - 1;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(stripLength, pin, NEO_GRB + NEO_KHZ800);


void setup() {
  // put your setup code here, to run once:
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

}

void loop() {
  // put your main code here, to run repeatedly:
  SetRangeRGB (0, lastPixel, 255, 0, 0);
  strip.show();
}

