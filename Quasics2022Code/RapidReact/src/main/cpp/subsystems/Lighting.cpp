// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lighting.h"

#include <iostream>

Lighting::Lighting() {
  SetName("Lighting");

  m_ledStrip.SetLength(m_ledBuffer.size());
  SetAllToColor(StockColor::Green);

  m_ledStrip.Start();
}

void Lighting::SetAllToColor(StockColor c) {
  SetAllToColor(Translate(c));
}

void Lighting::SetAllToColor(int r, int g, int b) {
  SetAllToColor(frc::Color(r / 255.0, g / 255.0, b / 255.0));
}

void Lighting::SetAllToColor(frc::Color c) {
  for (int i = 0; i < int(m_ledBuffer.size()); i++) {
    m_ledBuffer[i].SetLED(c);
  }
  m_ledStrip.SetData(m_ledBuffer);
}

void Lighting::SetStripColors(
    std::function<frc::AddressableLED::LEDData(int pos)> colorFunction) {
  int pos = 0;
  std::generate(m_ledBuffer.begin(), m_ledBuffer.end(),
                [&] { return colorFunction(pos++); });

  m_ledStrip.SetData(m_ledBuffer);
}

frc::Color Lighting::Translate(StockColor c) {
  if (c == StockColor::Red) {
    return frc::Color(255, 0, 0);
  } else if (c == StockColor::Green) {
    return frc::Color(0, 255, 0);
  } else if (c == StockColor::Blue) {
    return frc::Color(0, 0, 255);
  } else if (c == StockColor::White) {
    return frc::Color(255, 255, 255);
  } else {
    // (Matthew): It's probably worth letting the user know if they "fell
    // through" into this case because they're providing a StockColor that
    // isn't in the list above (e.g., printing an error message).  That way,
    // if someone defines a new color tomorrow (e.g., "Orange" for Halloween),
    // but doesn't update this function, there's some sort of diagnostic
    // output that can signal the source of the problem (rather than, "The
    // lights are off, must be a loose wire....").(done)
    std::cout << "The StockColor Given is not defined, make sure to add it to "
                 "the Lighting.h file"
              << std::endl;
    return frc::Color(0, 0, 0);
  }
}

void Lighting::SetEachCellToColor(
    std::function<frc::Color(int pos)> colorFunction) {
  // Update each element in the buffer to hold the desired color.
  int pos = 0;
  std::generate(m_ledBuffer.begin(), m_ledBuffer.end(),
                [&pos, colorFunction]() {
                  auto color = colorFunction(pos++);
                  return TranslateToLEDData(color);
                });

  // Apply the new buffer full of colors to the strip.
  m_ledStrip.SetData(m_ledBuffer);
}

// This method will be called once per scheduler run
void Lighting::Periodic() {
}
