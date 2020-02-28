#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        12 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 14 // Popular NeoPixel ring size
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
void setup() {
  // put your setup code here, to run once:
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int b = 0; b < 360; b = b + 1) {
    for (int pixel = 0; pixel < NUMPIXELS; pixel++) {
      const float hue = (b + pixel * ((360/NUMPIXELS)*1)) % 360;
      const float saturation = 100;
      const float value = 100;
      byte red;
      byte green;
      byte blue;
      HSV_to_RGB(hue, saturation, value, &red, &green, &blue);
      pixels.setPixelColor(pixel, red, green, blue);
    }
    pixels.show();   // Send the updated pixel colors to the hardware.
    delay(1);
  }
}

void HSV_to_RGB(float h, float s, float v, byte *r, byte *g, byte *b)
{
  int i;
  float f, p, q, t;

  h = max(0.0, min(360.0, h));
  s = max(0.0, min(100.0, s));
  v = max(0.0, min(100.0, v));

  s /= 100;
  v /= 100;

  if (s == 0) {
    // Achromatic (grey)
    *r = *g = *b = round(v * 255);
    return;
  }

  h /= 60; // sector 0 to 5
  i = floor(h);
  f = h - i; // factorial part of h
  p = v * (1 - s);
  q = v * (1 - s * f);
  t = v * (1 - s * (1 - f));
  switch (i) {
    case 0:
      *r = round(255 * v);
      *g = round(255 * t);
      *b = round(255 * p);
      break;
    case 1:
      *r = round(255 * q);
      *g = round(255 * v);
      *b = round(255 * p);
      break;
    case 2:
      *r = round(255 * p);
      *g = round(255 * v);
      *b = round(255 * t);
      break;
    case 3:
      *r = round(255 * p);
      *g = round(255 * q);
      *b = round(255 * v);
      break;
    case 4:
      *r = round(255 * t);
      *g = round(255 * p);
      *b = round(255 * v);
      break;
    default: // case 5:
      *r = round(255 * v);
      *g = round(255 * p);
      *b = round(255 * q);
  }
}
