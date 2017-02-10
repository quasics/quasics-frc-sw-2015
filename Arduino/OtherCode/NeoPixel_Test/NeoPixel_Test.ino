#include <Adafruit_NeoPixel.h>

#define STRIP_LENGTH 16
#define PIN 6

void SetStripColor (uint32_t red , uint32_t green , uint32_t blue, uint32_t white );
void SetStripColor (float hue, float saturation, float value);

void SetRangeColor (uint32_t first, uint32_t last, uint32_t red, uint32_t green, uint32_t blue, uint32_t white);
void SetRangeColor (uint32_t first, uint32_t last, float hue, float saturation, float value);


Adafruit_NeoPixel strip = Adafruit_NeoPixel(STRIP_LENGTH, PIN, NEO_GRBW + NEO_KHZ800);
const uint32_t lastPixel = STRIP_LENGTH-1;
const uint32_t halfPixel = uint32_t((STRIP_LENGTH - 1) /2 + .5);

void setup() {
  // put your setup code here, to run once:
  strip.begin();
  strip.show();
  strip.setBrightness(64);
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t iteration = 0;
  SetStripColor (float(iteration), 1, 1);
  for (int pixel = 0; pixel<= lastPixel; pixel++){
    SetRangeColor (pixel, pixel, float(int(iteration + 360/lastPixel * pixel) % 360)/360, 1, 1);
  }
  
  strip.show();
  iteration = ((iteration)%360) + 1;
}



