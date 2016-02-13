#include "color_def.h"
#include "light_mgmt.h"

// #define USE_REAL_LEDS

/*************************************************************************/
// Output (PWM) pin definitions
#define RED_PIN   11    // R petal on RGB LED module connected to digital pin 11
#define GREEN_PIN 10    // G petal on RGB LED module connected to digital pin 10
#define BLUE_PIN   9    // B petal on RGB LED module connected to digital pin 9

LightManager* lightManager = nullptr;

/**************************************************************************/     
void setup()
{
  Serial.begin(115200);

#ifdef USE_REAL_LEDS
  pinMode(RED_PIN, OUTPUT); // sets the redPin to be an output
  pinMode(GREEN_PIN, OUTPUT); // sets the greenPin to be an output
  pinMode(BLUE_PIN, OUTPUT); // sets the bluePin to be an output
  lightManager = new PwmLightManager(RED_PIN, GREEN_PIN, BLUE_PIN);
#else
  lightManager = new DummyLightManager();
#endif  // USE_REAL_LEDS

  lightManager->setMode(LightManager::eDemoMode);
  lightManager->setState(LightManager::eDisabled);

  showCommands();
}

void showCommands() {
  p("Supported operations:\n");
  p("   Mode: red/blue/demo/error\n");
  p("   State: disabled/auto/teleop/test\n\n");
}

void executeCommand(String s) {
  s.trim();
  if (s.length() == 0) {
    // Empty command (e.g., user just hit <Return>)
    return;
  }

  s.toLowerCase();
  if (s == "blue") {
    lightManager->setMode(LightManager::eBlueAlliance);
  } else if (s == "red") {
    lightManager->setMode(LightManager::eRedAlliance);
  } else if (s == "demo") {
    lightManager->setMode(LightManager::eDemoMode);
  } else if (s == "error") {
    lightManager->setMode(LightManager::eErrorMode);
  } else if (s == "disabled") {
    lightManager->setState(LightManager::eDisabled);
  } else if (s == "test") {
    lightManager->setState(LightManager::eTest);
  } else if (s == "auto") {
    lightManager->setState(LightManager::eAuto);
  } else if (s == "tele" || s == "teleop") {
    lightManager->setState(LightManager::eTeleOp);
  } else {
    p("Unknown command: %s\n", s.c_str());
    showCommands();
  }
}

void processCommands() {
  static String text;

  while (Serial.available() > 0) {
    char rc = Serial.read();
    if (rc == '\n' || rc == '\r' || rc == ';') {
      executeCommand(text);
      text = "";
    } else {
      text += rc;
    }
  }
}

/***************************************************************************/
void loop()  // run over and over again 
{
  processCommands();
  lightManager->updateLights();
  delay(10);
}    

