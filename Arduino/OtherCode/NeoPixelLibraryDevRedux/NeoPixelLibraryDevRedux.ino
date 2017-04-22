#include "Arduino.h"
#include "NeoPixelPatternManager.h"

NeoPixelPatternManager * strip;

//The setup function is called once at startup of the sketch
void setup()
{
  strip = new NeoPixelPatternManager(15, 6, 500);
  strip->SetMode(NeoPixelPatternManager::kGreen);
  strip->SetPattern(NeoPixelPatternManager::kOn);
}

// The loop function is called in an endless loop
void loop()
{
  strip->Process();
}
