#include "NeoPixelSerialController.h"

NeoPixelSerialController::NeoPixelSerialController (uint32_t pin, float loopSeconds, uint8_t brightness, uint32_t stripLength) {
  strip = new NeoPixelController (pin, loopSeconds, brightness, stripLength);
  Serial1.begin(115200);
  
}

void NeoPixelSerialController::NeoPixelSerialProcess () {
  if (Serial1.available() > 0) {
    String serialIn = "";
    while (Serial1.available() > 0) {
      char c = char(Serial1.read());
      if (c != ';' && c != '\n' && c != '\r') {
        serialIn += c;
      } else {
        Translator(serialIn.c_str());
        serialIn = "";
      }
    }
  }
  strip->NeoPixelProcess();
}

void NeoPixelSerialController::Translator (const char * input){
   if (strcmp(input, "Red") == 0) {
    strip->SetColorMode(NeoPixelController::kRed);
   } else if (strcmp(input, "Green") == 0){
    strip->SetColorMode(NeoPixelController::kGreen);
   } else if (strcmp(input, "Blue") == 0){
    strip->SetColorMode(NeoPixelController::kBlue);
   } else if (strcmp(input, "White") == 0){
    strip->SetColorMode(NeoPixelController::kWhite);
   } else if (strcmp(input, "Rainbow") == 0){
    strip->SetColorMode(NeoPixelController::kRainbow);
   } else if (strcmp(input, "OldCycle") == 0){
    strip->SetColorMode(NeoPixelController::kOldCycle);
   } else if (strcmp(input, "Yellow") == 0){
    strip->SetColorMode(NeoPixelController::kYellow);
   } else if (strcmp(input, "Quasics") == 0){
    strip->SetColorMode(NeoPixelController::kQuasics);
   } else if (strcmp(input, "Brown") == 0){
    strip->SetColorMode(NeoPixelController::kBrown);
   } else if (strcmp(input, "On") == 0){
    strip->SetBrightnessMode(NeoPixelController::kOn);
   } else if (strcmp(input, "Breathing") == 0){
    strip->SetBrightnessMode(NeoPixelController::kBreathing);
   } else if (strcmp(input, "Blinking") == 0){
    strip->SetBrightnessMode(NeoPixelController::kBlinking);
   } else if (strcmp(input, "Dashed") == 0){
    strip->SetBrightnessMode(NeoPixelController::kDashed);
   } else if (strcmp(input, "RollIn") == 0){
    strip->SetBrightnessMode(NeoPixelController::kRollIn);
   } else if (strcmp(input, "RollOut") == 0){
    strip->SetBrightnessMode(NeoPixelController::kRollOut);
   } else if (strcmp(input, "Rolling") == 0){
    strip->SetBrightnessMode(NeoPixelController::kRolling);
   } else {
    strip->SetBrightnessMode(NeoPixelController::kOff);
    strip->SetColorMode(NeoPixelController::kNone);
   }
}

