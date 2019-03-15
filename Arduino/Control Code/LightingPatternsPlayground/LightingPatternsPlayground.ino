#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#include "PatternColor.h"

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      12

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

const uint32_t BLACK = Adafruit_NeoPixel::Color(0, 0, 0);
const uint32_t RED = Adafruit_NeoPixel::Color(150, 0, 0);
const uint32_t BLUE = Adafruit_NeoPixel::Color(0, 0, 150);
const uint32_t GREEN = Adafruit_NeoPixel::Color(0, 150, 0);
const uint32_t YELLOW = Adafruit_NeoPixel::Color(150, 150, 0);

void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  pixels.begin(); // This initializes the NeoPixel library.

  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, BLACK);
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void loop() {
  chase(eRed);
  chase(eBlue);
  chase(eGreen);
  
  pulse(eRed);
  pulse(eBlue);
  pulse(eGreen);
  
  blinking(eRed);
  blinking(eBlue);
  blinking(eGreen);
}

uint32_t getColorValue(PatternColor patternColor) {
  if (patternColor == eRed)
    return RED;
  if (patternColor == eGreen)
    return GREEN;
  if (patternColor == eBlue)
    return BLUE;
  if (patternColor == eYellow)
    return YELLOW;

  // This shouldn't ever happen, but if it does, we'll be able to get some warning.
  Serial.print("Unrecognized PatternColor: ");
  Serial.print(int(patternColor));
  Serial.println(" --- Falling back on YELLOW.");
  return YELLOW;
}

uint32_t getColorValueUsingIntensity(PatternColor patternColor, int intensity) {
  if (patternColor == eGreen) {
    return pixels.Color(0, intensity, 0);
  }
  else if (patternColor == eRed) {
    return pixels.Color(intensity, 0, 0);
  }
  else if (patternColor == eBlue) {
    return pixels.Color(0, 0, intensity);
  }
  else {
    // Just assume it's yellow
    return pixels.Color(intensity, intensity, 0);
  }
}

void chase(PatternColor patternColor) {
  const uint32_t c = getColorValue(patternColor);
  const int delayval = 30; // delay for 3/100ths of a second

  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, c);
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval);
  }

  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, BLACK);
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval);
  }
}

void pulse(PatternColor patternColor) {
  const int delayval = 5; // delay for 0.005 seconds

  // Ramp the lights up
  for (int b = 0; b <= 150; b = b + 10) {
    const uint32_t newColor = getColorValueUsingIntensity(patternColor, b);
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, newColor);
      pixels.show();
      delay(delayval);
    }
  }

  // Ramp them back down
  for (int b = 150; b >= 0; b = b - 10) {
    const uint32_t newColor = getColorValueUsingIntensity(patternColor, b);
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, newColor);
      pixels.show();
      delay(delayval);
    }
  }
}

void blinking(PatternColor patternColor) {
  const uint32_t c = getColorValue(patternColor);
  const int delayval = 400;   // Delay time in milliseconds (4/10ths sec)

  // Turn them all on
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, c);
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
  delay(delayval);

  // Turn them all off
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, BLACK);
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
  delay(delayval);
}

