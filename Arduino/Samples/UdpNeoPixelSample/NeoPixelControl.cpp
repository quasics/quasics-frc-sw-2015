#include "ConfigurationFlags.h"
#include "NeoPixelControl.h"
#include "Logging.h"

// The NeoPixel strip being controlled.
static Adafruit_NeoPixel * strip = 0;

// The current mode for the lights.
static NeoPixelMode mode = NeoPixelMode::eOff;

// Convenient color definition.
const uint32_t QUASICS_GREEN = Adafruit_NeoPixel::Color(40, 255, 0, 0);


///////////////////////////////////////////////////////////////////////////
// Data used to keep track of whether or not we need to update the lights

// How often things should change
static unsigned long STEP_INTERVAL_MSEC = 100;

// When is the next time (reported by millis()) that we should update the lights?
static unsigned long int nextTick = 0;


///////////////////////////////////////////////////////////////////////////
// Data used for the "on" mode.

// Was the first pixel lit up the last time that we changed the pattern?
static bool firstPixelLit = false;


///////////////////////////////////////////////////////////////////////////
// Function definitions

void initializeNeoPixels(int pin, int length, neoPixelType type) {
  if (strip != 0) {
    delete strip;
    strip = 0;
  }

  strip = new Adafruit_NeoPixel(length, pin, type);
  strip->setBrightness(255);
  strip->begin();

  // Initialize all pixels to 'off'.
  for(uint16_t i = 0; i < strip->numPixels(); i++) {
    strip->setPixelColor(i, 0);
  }
  strip->show();
  nextTick = 0;
}

void setNeoPixelMode(NeoPixelMode newMode) {
  mode = newMode;
  nextTick = 0;
}

void stepNeoPixels() {
  unsigned long int curTime = millis();
  if (curTime < nextTick) {
    return;
  }

  if (mode == NeoPixelMode::eOn) {
    bool curPixelLit = !firstPixelLit;
    for(uint16_t i = 0; i < strip->numPixels(); i++) {
      strip->setPixelColor(i, curPixelLit ? QUASICS_GREEN : 0);
      curPixelLit = !curPixelLit;
    }
    firstPixelLit = !firstPixelLit;
  } else {
    // We must be off.
    for(uint16_t i = 0; i < strip->numPixels(); i++) {
      strip->setPixelColor(i, 0);
    }
  }
  strip->show();

  // Updates for next time
  nextTick = curTime + STEP_INTERVAL_MSEC;
}

