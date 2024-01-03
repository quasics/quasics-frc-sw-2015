#include "NeoPixelUSBController.h"

#define SERIAL_PORT Serial  // or Serial1 on a Mega

NeoPixelUSBController::NeoPixelUSBController (uint32_t pin, float loopSeconds, uint8_t brightness, uint32_t stripLength, neoPixelType type) {
  strip = new NeoPixelController (pin, loopSeconds, brightness, stripLength, type);
  SERIAL_PORT.begin(115200);
  serialIn = "";
}

void NeoPixelUSBController::NeoPixelSerialProcess () {
  while (SERIAL_PORT.available() > 0) {
    char c = char(SERIAL_PORT.read());
    if (c == ';') {
      Translator(serialIn.c_str());
      serialIn = "";
    } else {
      serialIn += c;
    }

  }
  strip->NeoPixelProcess();
}

inline bool equal(String s1, String s2) {
  return s1.equals(s2);
}

void NeoPixelUSBController::Translator (String input) {
  input.trim();
  SERIAL_PORT.println("--------------------------------------------------------------------");
  SERIAL_PORT.print("Input  : '");
  SERIAL_PORT.print(input);
  SERIAL_PORT.println(";'");
  Serial.println(input);
  input.toUpperCase();
  if (equal(input, "RED")) {
    strip->SetColorMode(NeoPixelController::kRed);
    SERIAL_PORT.println("     Color  : Red");
  } else if (equal(input, "PURPLE")) {
    strip->SetColorMode(NeoPixelController::kPurple);
    SERIAL_PORT.println("     Color  : Purple");
  } else if (equal(input, "ORANGE") || equal(input, "HALLOWEEN")) {
    strip->SetColorMode(NeoPixelController::kOrange);
    SERIAL_PORT.println("     Color  : Orange");
  } else if (equal(input, "HELP")) {
    PrintHelpInfo();
  } else if (equal(input, "GREEN")) {
    strip->SetColorMode(NeoPixelController::kGreen);
    SERIAL_PORT.println("     Color  : Green");
  } else if (equal(input, "BLUE")) {
    strip->SetColorMode(NeoPixelController::kBlue);
    SERIAL_PORT.println("     Color  : Blue");
  } else if (equal(input, "WHITE")) {
    strip->SetColorMode(NeoPixelController::kWhite);
    SERIAL_PORT.println("     Color  : White");
  } else if (equal(input, "RAINBOW")) {
    strip->SetColorMode(NeoPixelController::kRainbow);
    SERIAL_PORT.println("     Color  : Rainbow");
  } else if (equal(input, "OLDCYCLE")) {
    strip->SetColorMode(NeoPixelController::kOldCycle);
    SERIAL_PORT.println("     Color  : Old Cycle");
  } else if (equal(input, "YELLOW")) {
    strip->SetColorMode(NeoPixelController::kYellow);
    SERIAL_PORT.println("     Color  : Yellow");
  } else if (equal(input, "QUASICS")) {
    strip->SetColorMode(NeoPixelController::kQuasics);
    SERIAL_PORT.println("     Color  : Quasics");
  } else if (equal(input, "BROWN")) {
    strip->SetColorMode(NeoPixelController::kBrown);
    SERIAL_PORT.println("     Color  : Brown");
  } else if (equal(input, "ON")) {
    strip->SetBrightnessMode(NeoPixelController::kOn);
    SERIAL_PORT.println("     Dynamic: On");
  } else if (equal(input, "BREATHING")) {
    strip->SetBrightnessMode(NeoPixelController::kBreathing);
    SERIAL_PORT.println("     Dynamic: Breathing");
  } else if (equal(input, "BLINKING")) {
    strip->SetBrightnessMode(NeoPixelController::kBlinking);
    SERIAL_PORT.println("     Dynamic: Blinking");
  } else if (equal(input, "DASHED")) {
    strip->SetBrightnessMode(NeoPixelController::kDashed);
    SERIAL_PORT.println("     Dynamic: Dashed");
  } else if (equal(input, "ROLLIN")) {
    strip->SetBrightnessMode(NeoPixelController::kRollIn);
    SERIAL_PORT.println("     Dynamic: Roll In");
  } else if (equal(input, "ROLLOUT")) {
    strip->SetBrightnessMode(NeoPixelController::kRollOut);
    SERIAL_PORT.println("     Dynamic: Roll Out");
  } else if (equal(input, "ROLLING")) {
    strip->SetBrightnessMode(NeoPixelController::kRolling);
    SERIAL_PORT.println("     Dynamic: Rolling");
  } else if (equal(input, "OFF")) {
    strip->SetBrightnessMode(NeoPixelController::kOff);
    strip->SetColorMode(NeoPixelController::kNone);
    SERIAL_PORT.println("     Notice : Colors and dynamics reset");
  } else if (equal(input, "SPEEDUP")) {
    SERIAL_PORT.print("     Notice :Set loop time from ");
    SERIAL_PORT.print(strip->LoopTime());
    strip->SetLoopTime(strip->LoopTime() / 2);
    SERIAL_PORT.print(" seconds to ");
    SERIAL_PORT.print(strip->LoopTime());
    SERIAL_PORT.println(" seconds");
  } else if (equal(input, "SPEEDDOWN")) {
    SERIAL_PORT.print("     Notice :Set loop time from ");
    SERIAL_PORT.print(strip->LoopTime());
    strip->SetLoopTime(strip->LoopTime() * 2);
    SERIAL_PORT.print(" seconds to ");
    SERIAL_PORT.print(strip->LoopTime());
    SERIAL_PORT.println(" seconds");
  } else if (equal(input, "MAXUP")) {
    if (strip->GetMaxBrightness() * 2 >= 1) {
      strip->SetMaxBrightness(1);
      SERIAL_PORT.println("     Notice : Brightness At Max");
    } else {
      strip->SetMaxBrightness(strip->GetMaxBrightness() * 2);
      SERIAL_PORT.print("     Notice : Brightness at ");
      SERIAL_PORT.print(strip->GetMaxBrightness() * 100);
      SERIAL_PORT.println("% of absolute max");
    }
  } else if (equal(input, "MAXDOWN")) {
    strip->SetMaxBrightness(strip->GetMaxBrightness() / 2);
    SERIAL_PORT.print("     Notice : Brightness at ");
    SERIAL_PORT.print(strip->GetMaxBrightness() * 100);
    SERIAL_PORT.println("% of absolute max");
  } else if (equal(input, "SEGMENTUP")) {
    if (strip->GetPixelsPerSegment() < strip->GetStripLength() - 4) {
      strip->SetPixelsPerSegment(strip->GetPixelsPerSegment() + 2);
      SERIAL_PORT.print("     Notice : ");
      SERIAL_PORT.print(strip->GetPixelsPerSegment());
      SERIAL_PORT.println(" pixels per segment");
    } else {
      strip->SetPixelsPerSegment(strip->GetStripLength());
      SERIAL_PORT.println("     Notice : Segment size at max (strip length)");
    }
    strip->SetBrightnessMode(NeoPixelController::kDashed);
  }  else if (equal(input, "SEGMENTDOWN")) {
    if (strip->GetPixelsPerSegment() > 5) {
      strip->SetPixelsPerSegment(strip->GetPixelsPerSegment() - 2);
      SERIAL_PORT.print("     Notice : ");
      SERIAL_PORT.print(strip->GetPixelsPerSegment());
      SERIAL_PORT.println(" pixels per segment");
    } else {
      strip->SetPixelsPerSegment(2);
      SERIAL_PORT.println("     Notice : Segment size at min (2)");
    }
    strip->SetBrightnessMode(NeoPixelController::kDashed);
  } else if (equal(input, "SNAKEIN")) {
    strip->SetBrightnessMode(NeoPixelController::kSnakeIn);
    SERIAL_PORT.println("     Dynamic: Snake In");
  } else if (equal(input, "SNAKEOUT")) {
    strip->SetBrightnessMode(NeoPixelController::kSnakeOut);
    SERIAL_PORT.println("     Dynamic: Snake Out");
  } else if (equal(input, "SNAKE")) {
    strip->SetBrightnessMode(NeoPixelController::kSnake);
    SERIAL_PORT.println("     Dynamic: Snake");
  } else if (equal(input, "PSNAKE")) {
    strip->SetBrightnessMode(NeoPixelController::kPSnake);
    SERIAL_PORT.println("     Dynamic: Persistant Snake");
  } else if (equal(input, "RAINBOWREVERSE")) {
    strip->SetColorMode(NeoPixelController::kRainbowReverse);
    SERIAL_PORT.println("     Dynamic: Reverse Rainbow");
  } else {
    SERIAL_PORT.println("     ***ERROR***");
  }
  SERIAL_PORT.println("--------------------------------------------------------------------\n");
}

void NeoPixelUSBController::PrintHelpInfo() {
  SERIAL_PORT.println("Colors:");
  SERIAL_PORT.println("    Red;");
  SERIAL_PORT.println("    Green;");
  SERIAL_PORT.println("    Blue;");
  SERIAL_PORT.println("    White;");
  SERIAL_PORT.println("    Rainbow;");
  SERIAL_PORT.println("    RainbowReverse;");
  SERIAL_PORT.println("    OldCycle;");
  SERIAL_PORT.println("    Yellow;");
  SERIAL_PORT.println("    Quasics;");
  SERIAL_PORT.println("    Brown;");
  SERIAL_PORT.println("    Purple;");
  SERIAL_PORT.println("    Orange;");
  SERIAL_PORT.println("    Halloween;");
  SERIAL_PORT.println("Dynamics:");
  SERIAL_PORT.println("    On;");
  SERIAL_PORT.println("    Breathing;");
  SERIAL_PORT.println("    Blinking;");
  SERIAL_PORT.println("    Dashed;");
  SERIAL_PORT.println("    RollIn;");
  SERIAL_PORT.println("    RollOut;");
  SERIAL_PORT.println("    Rolling;");
  SERIAL_PORT.println("    SnakeIn;");
  SERIAL_PORT.println("    SnakeOut;");
  SERIAL_PORT.println("    Snake;");
  SERIAL_PORT.println("    PSnake;");
  SERIAL_PORT.println("    Off;");
  SERIAL_PORT.println("Settings:");
  SERIAL_PORT.println("    SpeedUp;");
  SERIAL_PORT.println("    SpeedDown;");
  SERIAL_PORT.println("    MaxUp;");
  SERIAL_PORT.println("        Increase max brightness");
  SERIAL_PORT.println("    MaxDown;");
  SERIAL_PORT.println("        Decrease max brightness");
  SERIAL_PORT.println("    SegmentUp;");
  SERIAL_PORT.println("        Increase size of dashed segments");
  SERIAL_PORT.println("    SegmentDown;");
  SERIAL_PORT.println("        Decrease size of dashed segments");
}
