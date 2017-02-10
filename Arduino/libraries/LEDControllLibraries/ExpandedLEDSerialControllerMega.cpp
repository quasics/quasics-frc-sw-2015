#include "ExpandedLEDSerialControllerMega.h"

ExpandedLEDSerialControllerMega::ExpandedLEDSerialControllerMega (unsigned int redPin, unsigned int greenPin, unsigned int bluePin, unsigned int whitePin, unsigned int baseLoopSeconds):
  LEDController (redPin, greenPin, bluePin, whitePin)
{
  isLowBatteryOverride = false;
  Serial1.begin(115200);
  serialIn = "";
  activeMode = kError;
  activeState = kOff;
  loopSeconds = baseLoopSeconds;
}

void ExpandedLEDSerialControllerMega::LEDSerialProcess () {
  while (Serial1.available() > 0)
  {
    char c = char(Serial1.read());
    if (c != ';') {
      serialIn += c;
    }
    else {
      Translator (serialIn.c_str(), activeMode, activeState);
      serialIn = "";
    }
  }

  SetMode (activeMode);
  SetState (activeState);
}

void ExpandedLEDSerialControllerMega::Translator (const char * input, Mode & mode, State & state) {
  static Mode localMode = kError;
  static State localState = kBreathing;

  if (strcmp(input, "Red") == 0) {
    localMode = kRed;
    Serial1.println("Red");
  }
  else if (strcmp(input, "Blue") == 0) {
    localMode = kBlue;
    Serial1.println("Blue");
  }
  else if (strcmp(input, "Green") == 0) {
    localMode = kGreen;
    Serial1.println("Green");
  }
  else if (strcmp(input, "Yellow") == 0) {
    localMode = kYellow;
    Serial1.println("Yellow");
  }
  else if (strcmp(input, "Magenta") == 0) {
    localMode = kMagenta;
    Serial1.println("Magenta");
  }
  else if (strcmp(input, "Cyan") == 0) {
    localMode = kCyan;
    Serial1.println("Cyan");
  }
  else if (strcmp(input, "Orange") == 0) {
    localMode = kOrange;
    Serial1.println("Orange");
  }
  else if (strcmp(input, "White") == 0) {
    localMode = kWhite;
    Serial1.println("White");
  }
  else if (strcmp(input, "Purple") == 0) {
    localMode = kPurple;
    Serial1.println("Purple");
  }

  else if (strcmp(input, "Rainbow") == 0) {
    localMode = kRainbow;
    Serial1.println("Rainbow");
  }
  else if (strcmp(input, "Cycle") == 0) {
    localMode = kCycle;
    Serial1.println("Cycle");
  }

  else if (strcmp(input, "Breathing") == 0) {
    localState = kBreathing;
    Serial1.println("Breathing");
  }
  else if (strcmp(input, "Solid") == 0) {
    localState = kSolid;
    Serial1.println("Solid");
  }
  else if (strcmp(input, "SlowBlink") == 0) {
    localState = kSlowBlink;
    Serial1.println("Slow Blink");
  }
  else if (strcmp(input, "Blink") == 0) {
    localState = kBlink;
    Serial1.println("Blink");
  }
  else if (strcmp(input, "QuickBlink") == 0) {
    localState = kQuickBlink;
    Serial1.println("Quick Blink");
  }
  else if (strcmp(input, "Off") == 0) {
    localState = kOff;
    Serial1.println("Off");
  }
  else  {
    localMode = kError;
    localState = kOff;
    Serial1.print("Don't understand: '");
    Serial1.print(input);
    Serial1.println("', Switching Off.");
  }

  mode = localMode;
  state = localState;
}

void ExpandedLEDSerialControllerMega::SetMode (Mode mode) {
  switch (mode) {
    case kRed:
      SetRed (255);
      SetGreen ();
      SetBlue ();
      SetWhite ();
      break;
    case kBlue:
      SetRed ();
      SetGreen ();
      SetBlue (255);
      SetWhite ();
      break;
    case kGreen:
      SetRed ();
      SetGreen (255);
      SetBlue ();
      SetWhite ();
      break;
    case kYellow:
      SetRed (255);
      SetGreen (255);
      SetBlue ();
      SetWhite ();
      break;
    case kMagenta:
      SetRed (255);
      SetGreen ();
      SetBlue (255);
      SetWhite ();
      break;
    case kCyan:
      SetRed ();
      SetGreen (255);
      SetBlue (255);
      SetWhite ();
      break;
    case kOrange:
      SetRed (255);
      SetGreen (63);
      SetBlue ();
      SetWhite ();
      break;
    case kPurple:
      SetRed (127);
      SetGreen ();
      SetBlue (255);
      SetWhite ();
      break;
    case kWhite:
      SetRed ();
      SetGreen ();
      SetBlue ();
      SetWhite (255);
      break;
    case kCycle:
      SetWhite();
      SetHSV ((double(millis() % (loopSeconds * 1000)) / double(loopSeconds * 1000)), 1, 1);
      break;
    case kRainbow: {
        unsigned long timeStamp = (millis() % 8000);
        if (timeStamp < 1000) {
          SetRed (255);
          SetGreen (0);
          SetBlue (0);
          SetWhite();

        }
        else if (timeStamp < 2000) {
          SetRed (255);
          SetGreen (31);
          SetBlue (0);
          SetWhite();

        }
        else if (timeStamp < 3000) {
          SetRed (255);
          SetGreen (150);
          SetBlue (0);
          SetWhite();

        }
        else if (timeStamp < 4000) {
          SetRed (0);
          SetGreen (255);
          SetBlue (0);
          SetWhite();

        }
        else if (timeStamp < 5000) {
          SetRed (0);
          SetGreen (255);
          SetBlue (255);
          SetWhite();

        }
        else if (timeStamp < 6000) {
          SetRed (0);
          SetGreen (0);
          SetBlue (255);
          SetWhite();

        }
        else if (timeStamp < 7000) {
          SetRed (127);
          SetGreen (0);
          SetBlue (255);
          SetWhite();

        }
        else {
          SetRed (255);
          SetGreen (0);
          SetBlue (255);
          SetWhite();
        }
      }
      break;
    default:
      SetRed (0);
      SetGreen (0);
      SetBlue (0);
      SetWhite();
      break;
  }
}

//kBreathing, kBlink, kSlowBlink, kQuickBlink, kSolid, kErrorState, kOff

void ExpandedLEDSerialControllerMega::SetState (State state) {
  switch (state) {
    case kBreathing:
      {
        const float loopDurration = loopSeconds;
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
        const float loopDurration = loopSeconds;
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
        const float loopDurration = loopSeconds * 2;
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

    case kQuickBlink: {
        const float loopDurration = loopSeconds / 2;
        const float halfDurration = loopDurration / 2;

        float timeLoop = float(millis() % int(loopDurration * 1000)) / 1000;


        if (timeLoop <= halfDurration)
          SetBrightness(1);
        else
          SetBrightness(0);
      }
      break;

    default:
      SetBrightness(0);
      break;
  }
}
