#include "ConfigurationFlags.h"
#include "NeoPixelControl.h"
#include "Logging.h"

// The NeoPixel strip being controlled.
static Adafruit_NeoPixel * strip = 0;

// The current mode for the lights.
static NeoPixelMode mode = NeoPixelMode::eOff;

// Convenient color definitions.
const uint32_t QUASICS_GREEN = Adafruit_NeoPixel::Color(40, 255, 0, 0);
const uint32_t YELLOW = Adafruit_NeoPixel::Color(255, 255, 0, 0);


///////////////////////////////////////////////////////////////////////////
// Data used to keep track of whether or not we need to update the lights

// How often things should change.  (Too fast, and you may not see the lights
// actually turning off.)
static unsigned long STEP_INTERVAL_MSEC = 100;

// When is the next time (reported by millis()) that we should update the lights?
static unsigned long int nextTick = 0;


///////////////////////////////////////////////////////////////////////////
// Data used for the "on" mode.

// Was the first pixel lit up the last time that we changed the pattern?
static bool firstPixelWasLit = false;


///////////////////////////////////////////////////////////////////////////
// "Helper" functions, used only inside this file.

namespace {

// Turns all of the lights off on the LED strip.
void turnLightsOff() {
  for(uint16_t i = 0; i < strip->numPixels(); i++) {
    strip->setPixelColor(i, 0);
  }
}

// Turns every other light on, using the specified color.
void turnOnEveryOtherLight(uint32_t color, bool lightTheFirstPixel) {
  bool curPixelLit = lightTheFirstPixel;
  for(uint16_t i = 0; i < strip->numPixels(); i++) {
    strip->setPixelColor(i, curPixelLit ? color : 0);
    curPixelLit = !curPixelLit;
  }
}

} // end of anonymous namespace


///////////////////////////////////////////////////////////////////////////
// Lighting control functions (usable outside this file).

void initializeNeoPixels(int pin, int length, neoPixelType type) {
  LOG("Initializing NeoPixel strip...");
  if (strip != 0) {
    delete strip;
    strip = 0;
  }

  strip = new Adafruit_NeoPixel(length, pin, type);
  strip->setBrightness(255);
  strip->begin();

  // Initialize all pixels to 'off'.
  turnLightsOff();
  strip->show();
  nextTick = 0;
}

void setNeoPixelMode(NeoPixelMode newMode) {
  mode = newMode;
  nextTick = 0;
}

void stepNeoPixels() {
  if (strip == 0) {
    // We never called "initializeNeoPixels()" (or it failed): bail out.
    return;
  }
  unsigned long int curTime = millis();
  if (curTime < nextTick) {
    return;
  }

  switch(mode) {
    case NeoPixelMode::eOn:
      turnOnEveryOtherLight(QUASICS_GREEN, !firstPixelWasLit);
      firstPixelWasLit = !firstPixelWasLit;
      break;
    case NeoPixelMode::eOff:
      turnLightsOff();
      break;
    case NeoPixelMode::eError:
      turnOnEveryOtherLight(YELLOW, !firstPixelWasLit);
      firstPixelWasLit = !firstPixelWasLit;
      break;
  }
  strip->show();

  // Updates for next time
  nextTick = curTime + STEP_INTERVAL_MSEC;
}

