#include "LEDSerialController.h"

LEDSerialController::LEDSerialController (unsigned int redPin, unsigned int greenPin, unsigned int bluePin, unsigned long heartRateSeconds):
  LEDController (redPin, greenPin, bluePin) 
  {
  isLowBatteryOverride = false;
  Serial.begin(115200);
  HeartRateSecs = heartRateSeconds;
  serialIn = "";
  activeMode = kDemo;
  activeState = kBreathing;
  activeBatteryLow = false;
  lastHeartbeat = 0;
}

void LEDSerialController::LEDSerialProcess () {
  while (Serial.available() > 0)
  {
    char c = char(Serial.read());
    if (c != ';') {
      serialIn += c;
    }
    else {
      Translator (serialIn.c_str(), activeMode, activeState, activeBatteryLow);
      serialIn = "";
    }
  }

  if (millis() - lastHeartbeat > HeartRateSecs * 2000) {
     activeMode = kError;
  }

  SetMode (activeMode);
  SetState (activeState);
  SetBatteryLow (activeBatteryLow);
}

void LEDSerialController::SetMode (Mode mode) {
  if (!isLowBatteryOverride) {
    switch (mode) {
      case kRedTeam:
        SetRed (255);
        SetGreen (0);
        SetBlue (0);
        break;
      case kBlueTeam:
        SetRed (0);
        SetGreen (0);
        SetBlue (255);
        break;
      case kDemo:
        SetRed (0);
        SetGreen (255);
        SetBlue (0);
        break;
      default:
        SetRed (255);
        SetGreen (255);
        SetBlue (0);
        break;
    }
  }
}

//kBreathing, kBlink, kSlowBlink, kQuickBlink, kSolid, kErrorState, kOff

void LEDSerialController::SetState (State state) {
  switch (state) {
    case kBreathing:
      {
        const float loopDurration = 4;
        const float halfDurration = loopDurration / 2;

        float timeLoop = float(millis() % int(loopDurration * 1000)) / 1000;
        float timeHalf = float(millis() % int(halfDurration * 1000)) / 1000;

        if (timeLoop <= halfDurration)
          SetBrightness((timeHalf * timeHalf) / (halfDurration * halfDurration));
        else
          SetBrightness(((halfDurration * halfDurration) - (timeHalf * timeHalf)) /
                                      (halfDurration * halfDurration));
      }
      break;

    case kBlink:
      {
        const float loopDurration = 2;
        const float halfDurration = loopDurration / 2;

        float timeLoop = float(millis() % int(loopDurration * 1000)) / 1000;

        if (timeLoop <= halfDurration)
          SetBrightness(1);
        else
          SetBrightness(0);
      }
      break;

    case kSlowBlink:
      {
        const float loopDurration = 3;
        const float halfDurration = loopDurration / 2;

        float timeLoop = float(millis() % int(loopDurration * 1000)) / 1000;

        if (timeLoop <= halfDurration)
          SetBrightness(1);
        else
          SetBrightness(0);
      }
      break;

    case kSolid:
      SetBrightness(1);
      break;

    case kOff:
      SetBrightness(0);
      break;

    default: //and quick blink are the same thing
      {
        const float loopDurration = 1;
        const float halfDurration = loopDurration / 2;

        float timeLoop = float(millis() % int(loopDurration * 1000)) / 1000;


        if (timeLoop <= halfDurration)
          SetBrightness(1);
        else
          SetBrightness(0);
      }
      break;
  }
}

void LEDSerialController::SetBatteryLow (bool isLow) {
  if (isLow && millis() % 5000 > 4000) {
    SetRed(255);
    SetGreen(255);
    SetBlue(0);
    isLowBatteryOverride = true;
  }
  else {
    isLowBatteryOverride = false;
  }
}

void LEDSerialController::Translator (const char * input, Mode & mode, State & state, bool & isBatteryLow) {
  static Mode localMode = kDemo;
  static State localState = kBreathing;
  static bool localBatteryLow = false;

  if (strcmp(input, "RedTeam") == 0)
    localMode = kRedTeam;
  else if (strcmp(input, "BlueTeam") == 0)
    localMode = kBlueTeam;
  else if (strcmp(input, "Demo") == 0)
    localMode = kDemo;
  else if (strcmp(input, "Disabled") == 0)
    localState = kBreathing;
  else if (strcmp(input, "Solid") == 0)
    localState = kSolid;
  else if (strcmp(input, "SlowBlink") == 0)
    localState = kSlowBlink;
  else if (strcmp(input, "MediumBlink") == 0)
    localState = kBlink;
  else if (strcmp(input, "Off") == 0)
    localState = kOff;
  else if (strcmp(input, "LowBattery") == 0)
    localBatteryLow = true;
  else if (strcmp(input, "GoodBattery") == 0)
    localBatteryLow = false;
  else if (strcmp(input, "Heartbeat") == 0){
    lastHeartbeat = millis();
    Serial.println("LubDub");
  }
  else  {
    localMode = kError;
    localState = kSolid;
    Serial.print("Don't understand: '");
    Serial.print(input);
    Serial.println("'");
    Serial.print(";");
  }



  mode = localMode;
  state = localState;
  isBatteryLow = localBatteryLow;
}
