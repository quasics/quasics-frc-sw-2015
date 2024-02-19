// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc2/command/SubsystemBase.h>

#include <vector>

class Lighting : public frc2::SubsystemBase {
 public:
  static const frc::AddressableLED::LEDData BLACK;
  static const frc::AddressableLED::LEDData WHITE;
  static const frc::AddressableLED::LEDData GREEN;
  static const frc::AddressableLED::LEDData RED;
  static const frc::AddressableLED::LEDData BLUE;
  static const frc::AddressableLED::LEDData YELLOW;
  static const frc::AddressableLED::LEDData ORANGE;
  static const frc::AddressableLED::LEDData CYAN;
  static const frc::AddressableLED::LEDData MAGENTA;
  static const frc::AddressableLED::LEDData PINK;
  static const frc::AddressableLED::LEDData PURPLE;
  static const frc::AddressableLED::LEDData YELLOW_GREEN;

 public:
  Lighting(int pwmPort, int size) : m_ledBuffer(size, GREEN), m_led(pwmPort) {
    SetName("Lighting");
    m_led.SetLength(m_ledBuffer.size());
    m_led.SetData(m_ledBuffer);
    m_led.Start();
  }

  typedef std::function<frc::AddressableLED::LEDData(int)> ColorFunction;

  void setStripColor(ColorFunction f) {
    for (unsigned int i = 0; i < m_ledBuffer.size(); ++i) {
      m_ledBuffer[i] = f(i);
    }
    m_led.SetData(m_ledBuffer);
  }

  void setSolidStripColor(frc::AddressableLED::LEDData color) {
    for (unsigned int i = 0; i < m_ledBuffer.size(); ++i) {
      m_ledBuffer[i] = color;
    }
    m_led.SetData(m_ledBuffer);
  }

 private:
  // Buffer allocation is expensive: allocate it once and just use it over and
  // over.
  std::vector<frc::AddressableLED::LEDData> m_ledBuffer;
  frc::AddressableLED m_led;
};
