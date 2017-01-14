#include "Arduino.h"
#include "NeoPixelController.h"

//#define testingEvolution

NeoPixelController* strip;

const uint32_t stripLength = 16;
const uint32_t stripPin = 6;
const uint32_t stripBrightness = 16;

void setup() {
	strip = new NeoPixelController(stripLength, stripPin, stripBrightness);
#ifdef testingEvolution
	strip->SetEvolution(NeoPixelController::kRollingCycle);
#else
	strip->SetMode(NeoPixelController::kGreenToWhite);
	strip->SetPattern(NeoPixelController::kOn);
#endif
}

void loop() {
#ifdef testingEvolution
	strip->Evolve();
#else
	strip->Process();
#endif
}

