#include "light_mgmt.h"
#include "color_def.h"

#define SKEW this->getClockSkew()
#define MED_BLINK_TIME_OFF_MS   (500 * SKEW)
#define SLOW_BLINK_TIME_OFF_MS  (1000 * SKEW)
#define BREATHE_PHASE_TIME_MS   (2000 * SKEW)

const ColorDef& LightManager::getColorForMode(Mode mode) {
  switch(mode) {
    case eRedAlliance:
      return ColorDef::RED;
    case eBlueAlliance:
      return ColorDef::BLUE;
    case eDemoMode:
      return ColorDef::GREEN;
    case eErrorMode:
      return ColorDef::YELLOW;
    case eUndefinedMode:
    default:
      return ColorDef::OFF;
  }
}

void LightManager::setMode(Mode mode) {
  currentMode_  = mode;
}
void LightManager::setState(State state) {
  currentState_ = state;
}

int LightManager::calculateIntensity() const {
  switch (currentMode_) {
    case eRedAlliance:
    case eBlueAlliance:
    case eDemoMode: {
      const unsigned long curTime = millis();
      switch(currentState_) {
        case eAuto:
          // solid
          return 100;
        case eTeleOp:
        {
          // slow blink
          const unsigned long count = curTime % (SLOW_BLINK_TIME_OFF_MS * 2);
          return (count < SLOW_BLINK_TIME_OFF_MS)
                    ? 100 : 0;
        }
        case eTest:
        {
          // medium blink
          const unsigned long count = curTime % (MED_BLINK_TIME_OFF_MS * 2);
          return (count < MED_BLINK_TIME_OFF_MS)
                    ? 100 : 0;
        }
        case eDisabled:
        {
          // breathing
          unsigned long count = curTime % (BREATHE_PHASE_TIME_MS * 2);
          if (count >= BREATHE_PHASE_TIME_MS) {
            count = BREATHE_PHASE_TIME_MS - (count - BREATHE_PHASE_TIME_MS);
          }
          return (count * 100 / BREATHE_PHASE_TIME_MS);
        }
      }
      // Fallback case
      return 100;
    }

    case eErrorMode:
    case eUndefinedMode:
      return 100;
  }

  return 0;
}

void LightManager::updateLights() {
  setLights(getColorForMode(currentMode_), calculateIntensity());
}

DummyLightManager::DummyLightManager()
  : lastColor_(nullptr), lastIntensity_(0) {
  pinMode(13, OUTPUT);
}

void DummyLightManager::setLights(const ColorDef& color, int intensityPercent) {
  const int intensityDelta = abs(intensityPercent - lastIntensity_);
  if (&color != lastColor_ || intensityDelta > 10) {
    p("Set lights to %s at %d%%\n", color.name_, intensityPercent);
    /*
    Serial.print("Set lights to ");
    Serial.print(color.name_);
    Serial.print(" at ");
    Serial.print(intensityPercent);
    Serial.println("%");
    */

    lastColor_ = &color;
    lastIntensity_ = intensityPercent;
  }
  
  digitalWrite(13, intensityPercent > 50 ? HIGH : LOW);
}

void PwmLightManager::setLights(const ColorDef& color, int intensityPercent) {
  const double percent = intensityPercent / 100.0;
  analogWrite(redPin_, int(color.red_ * percent));
  analogWrite(bluePin_, int(color.blue_ * percent));
  analogWrite(greenPin_, int(color.green_ * percent));
}

