#include "NeoPixelUSBController.h"
NeoPixelUSBController::NeoPixelUSBController (uint32_t pin, float loopSeconds, uint8_t brightness, uint32_t stripLength) {
  strip = new NeoPixelController (pin, loopSeconds, brightness, stripLength);
  Serial1.begin(115200);
  Serial.begin(115200);
  serialIn = "";
}

void NeoPixelUSBController::NeoPixelSerialProcess () {
  while (Serial1.available() > 0) {
    char c = char(Serial1.read());
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
  Serial1.println("--------------------------------------------------------------------");
  Serial1.print("Input  : '");
  Serial1.print(input);
  Serial1.println(";'");
  Serial.println(input);
  input.toUpperCase();
  if (equal(input, "RED")) {
    strip->SetColorMode(NeoPixelController::kRed);
    Serial1.println("     Color  : Red");
  } else if (equal(input, "PURPLE")) {
    strip->SetColorMode(NeoPixelController::kPurple);
    Serial1.println("     Color  : Purple");
  } else if (equal(input, "HELP")) {
    PrintHelpInfo();
  } else if (equal(input, "GREEN")) {
    strip->SetColorMode(NeoPixelController::kGreen);
    Serial1.println("     Color  : Green");
  } else if (equal(input, "BLUE")) {
    strip->SetColorMode(NeoPixelController::kBlue);
    Serial1.println("     Color  : Blue");
  } else if (equal(input, "WHITE")) {
    strip->SetColorMode(NeoPixelController::kWhite);
    Serial1.println("     Color  : White");
  } else if (equal(input, "RAINBOW")) {
    strip->SetColorMode(NeoPixelController::kRainbow);
    Serial1.println("     Color  : Rainbow");
  } else if (equal(input, "OLDCYCLE")) {
    strip->SetColorMode(NeoPixelController::kOldCycle);
    Serial1.println("     Color  : Old Cycle");
  } else if (equal(input, "YELLOW")) {
    strip->SetColorMode(NeoPixelController::kYellow);
    Serial1.println("     Color  : Yellow");
  } else if (equal(input, "QUASICS")) {
    strip->SetColorMode(NeoPixelController::kQuasics);
    Serial1.println("     Color  : Quasics");
  } else if (equal(input, "BROWN")) {
    strip->SetColorMode(NeoPixelController::kBrown);
    Serial1.println("     Color  : Brown");
  } else if (equal(input, "ON")) {
    strip->SetBrightnessMode(NeoPixelController::kOn);
    Serial1.println("     Dynamic: On");
  } else if (equal(input, "BREATHING")) {
    strip->SetBrightnessMode(NeoPixelController::kBreathing);
    Serial1.println("     Dynamic: Breathing");
  } else if (equal(input, "BLINKING")) {
    strip->SetBrightnessMode(NeoPixelController::kBlinking);
    Serial1.println("     Dynamic: Blinking");
  } else if (equal(input, "DASHED")) {
    strip->SetBrightnessMode(NeoPixelController::kDashed);
    Serial1.println("     Dynamic: Dashed");
  } else if (equal(input, "ROLLIN")) {
    strip->SetBrightnessMode(NeoPixelController::kRollIn);
    Serial1.println("     Dynamic: Roll In");
  } else if (equal(input, "ROLLOUT")) {
    strip->SetBrightnessMode(NeoPixelController::kRollOut);
    Serial1.println("     Dynamic: Roll Out");
  } else if (equal(input, "ROLLING")) {
    strip->SetBrightnessMode(NeoPixelController::kRolling);
    Serial1.println("     Dynamic: Rolling");
  } else if (equal(input, "OFF")) {
    strip->SetBrightnessMode(NeoPixelController::kOff);
    strip->SetColorMode(NeoPixelController::kNone);
    Serial1.println("     Notice : Colors and dynamics reset");
  } else if (equal(input, "SPEEDUP")) {
    Serial1.print("     Notice :Set loop time from ");
    Serial1.print(strip->LoopTime());
    strip->SetLoopTime(strip->LoopTime() / 2);
    Serial1.print(" seconds to ");
    Serial1.print(strip->LoopTime());
    Serial1.println(" seconds");
  } else if (equal(input, "SPEEDDOWN")) {
    Serial1.print("     Notice :Set loop time from ");
    Serial1.print(strip->LoopTime());
    strip->SetLoopTime(strip->LoopTime() * 2);
    Serial1.print(" seconds to ");
    Serial1.print(strip->LoopTime());
    Serial1.println(" seconds");
  } else if (equal(input, "MAXUP")) {
    if (strip->GetMaxBrightness() * 2 >= 1) {
      strip->SetMaxBrightness(1);
      Serial1.println("     Notice : Brightness At Max");
    } else {
      strip->SetMaxBrightness(strip->GetMaxBrightness() * 2);
      Serial1.print("     Notice : Brightness at ");
      Serial1.print(strip->GetMaxBrightness() * 100);
      Serial1.println("% of absolute max");
    }
  } else if (equal(input, "MAXDOWN")) {
    strip->SetMaxBrightness(strip->GetMaxBrightness() / 2);
    Serial1.print("     Notice : Brightness at ");
    Serial1.print(strip->GetMaxBrightness() * 100);
    Serial1.println("% of absolute max");
  } else if (equal(input, "SEGMENTUP")) {
    if (strip->GetPixelsPerSegment() < strip->GetStripLength() - 4) {
      strip->SetPixelsPerSegment(strip->GetPixelsPerSegment() + 2);
      Serial1.print("     Notice : ");
      Serial1.print(strip->GetPixelsPerSegment());
      Serial1.println(" pixels per segment");
    } else {
      strip->SetPixelsPerSegment(strip->GetStripLength());
      Serial1.println("     Notice : Segment size at max (strip length)");
    }
    strip->SetBrightnessMode(NeoPixelController::kDashed);
  }  else if (equal(input, "SEGMENTDOWN")) {
    if (strip->GetPixelsPerSegment() > 5) {
      strip->SetPixelsPerSegment(strip->GetPixelsPerSegment() - 2);
      Serial1.print("     Notice : ");
      Serial1.print(strip->GetPixelsPerSegment());
      Serial1.println(" pixels per segment");
    } else {
      strip->SetPixelsPerSegment(2);
      Serial1.println("     Notice : Segment size at min (2)");
    }
    strip->SetBrightnessMode(NeoPixelController::kDashed);
  } else if (equal(input, "SNAKEIN")) {
    strip->SetBrightnessMode(NeoPixelController::kSnakeIn);
    Serial1.println("     Dynamic: Snake In");
  } else if (equal(input, "SNAKEOUT")) {
    strip->SetBrightnessMode(NeoPixelController::kSnakeOut);
    Serial1.println("     Dynamic: Snake Out");
  } else if (equal(input, "SNAKE")) {
    strip->SetBrightnessMode(NeoPixelController::kSnake);
    Serial1.println("     Dynamic: Snake");
  } else if (equal(input, "PSNAKE")) {
    strip->SetBrightnessMode(NeoPixelController::kPSnake);
    Serial1.println("     Dynamic: Persistant Snake");
  } else if (equal(input, "RAINBOWREVERSE")) {
    strip->SetColorMode(NeoPixelController::kRainbowReverse);
    Serial1.println("     Dynamic: Reverse Rainbow");
  } else {
    Serial1.println("     ***ERROR***");
  }
  Serial1.println("--------------------------------------------------------------------\n");
}

void NeoPixelUSBController::PrintHelpInfo() {
  Serial1.println("Colors:");
  Serial1.println("    Red;");
  Serial1.println("    Green;");
  Serial1.println("    Blue;");
  Serial1.println("    White;");
  Serial1.println("    Rainbow;");
  Serial1.println("    RainbowReverse;");
  Serial1.println("    OldCycle;");
  Serial1.println("    Yellow;");
  Serial1.println("    Quasics;");
  Serial1.println("    Brown;");
  Serial1.println("    Purple;");
  Serial1.println("Dynamics:");
  Serial1.println("    On;");
  Serial1.println("    Breathing;");
  Serial1.println("    Blinking;");
  Serial1.println("    Dashed;");
  Serial1.println("    RollIn;");
  Serial1.println("    RollOut;");
  Serial1.println("    Rolling;");
  Serial1.println("    SnakeIn;");
  Serial1.println("    SnakeOut;");
  Serial1.println("    Snake;");
  Serial1.println("    PSnake;");
  Serial1.println("    Off;");
  Serial1.println("Settings:");
  Serial1.println("    SpeedUp;");
  Serial1.println("    SpeedDown;");
  Serial1.println("    MaxUp;");
  Serial1.println("        Increase max brightness");
  Serial1.println("    MaxDown;");
  Serial1.println("        Decrease max brightness");
  Serial1.println("    SegmentUp;");
  Serial1.println("        Increase size of dashed segments");
  Serial1.println("    SegmentDown;");
  Serial1.println("        Decrease size of dashed segments");
}
