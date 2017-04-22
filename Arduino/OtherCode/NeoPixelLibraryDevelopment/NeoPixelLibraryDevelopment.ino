#include "Arduino.h"
#include "NeoPixelController.h"

//#define testingEvolution

NeoPixelController* strip;

const uint32_t stripLength = 16;
const uint32_t stripPin = 6;
const uint32_t stripBrightness = 16;

void setup() {
	strip = new NeoPixelController(stripLength, stripPin, stripBrightness);

	strip->SetMode(NeoPixelController::kGreen);
	strip->SetPattern(NeoPixelController::kOn);
}

void loop() {
	strip->Process();
}

