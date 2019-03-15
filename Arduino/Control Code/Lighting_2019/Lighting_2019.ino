#include <Adafruit_NeoPixel.h>
#include "PatternColor.h"

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      25

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);


//////////////////////////////////////////////////////////////////////////////////////////
// Color value definitions for functions from "LightingPatternsPlayground" code file

const uint32_t BLACK = Adafruit_NeoPixel::Color(0, 0, 0);
const uint32_t RED = Adafruit_NeoPixel::Color(150, 0, 0);
const uint32_t BLUE = Adafruit_NeoPixel::Color(0, 0, 150);
const uint32_t GREEN = Adafruit_NeoPixel::Color(0, 150, 0);
const uint32_t YELLOW = Adafruit_NeoPixel::Color(150, 150, 0);

//////////////////////////////////////////////////////////////////////////////////////////
// Utility functions from "LightingPatternsPlayground" code file
//
// These will allow us to turn a given PatternColor into a color value that can be used
// to control a NeoPixel strip.

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

//////////////////////////////////////////////////////////////////////////////////////////
// Lighting control functions from "LightingPatternsPlayground" code file
//
// These will allow us to display a given pattern (1x), using a given PatternColor.

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

void showOneCycle(uint32_t color) {
  int delayval = 43; // delay for 43/1000ths of a second.

  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, color);
    pixels.show();
    delay(delayval);
  }

  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
    delay(delayval); // Delay for a period of time (150 miliseconds).

  }
}

//////////////////////////////////////////////////////////////////////////////////////////
// Lighting control code for 2019 robot.

// Set up is called one time (at program start).
void setup() {
  pixels.begin(); // This initializes the NeoPixel library.
  Serial.begin(115200);
}

// Returns a command (i.e., a line of text entered by the user) obtained from the
// Serial Monitor, or the empty string if we don't have a command to process.
String getCommandFromSerialMonitor() {
  // Buffer used to hold input from the user that isn't fully usable as a command
  // yet (e.g., we haven't seen the "\n" or "\r" at the end of input from the
  // Serial Monitor window.
  //
  // This is "static", so it will retain its data between one call to this function
  // and the next.
  static String text;

  // Process any data from the serial monitor, and see if we have a full command yet.
  while (Serial.available() > 0) {
    char nextChar = Serial.read();
    if (nextChar != '\n' && nextChar != '\r') {
      text += nextChar;
    } else {
      String command = text;
      text = "";
      return command;
    }
  }

  // We don't seem to have a full command, so return an empty string.
  return "";
}

// Loop function is called over and over and over (after setup() completes).
void loop() {
  // Holds the current color value to be used by the LED strip.
  //
  // This is "static", so it will retain its data between one call to this function
  // and the next.  The assignment below will *only* happen the first time that the
  // function is executed; after that, it will keep the value it had the *last* time
  // that the function ran.  (So the color value will stick around between calls.)
  static uint32_t color = GREEN;

  // See if we have a command to be executed.
  String command = getCommandFromSerialMonitor();

  // If we have a command, do something with it.
  if (command != "") {
    if (command == "red")  {
      color = RED;
    } else if (command == "blue") {
      color = BLUE;
    } else if (command == "green") {
      color = GREEN;
    }
  }

  // Run through 1 cycle of the lighting pattern, using the current color.
  showOneCycle(color);
}

