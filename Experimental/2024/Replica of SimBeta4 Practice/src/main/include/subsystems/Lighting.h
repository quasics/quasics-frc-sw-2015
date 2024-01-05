#pragma once

#include <frc/AddressableLED.h>
#include <frc2/command/SubsystemBase.h>

#include <vector>

//standard lighting subsystem. Just like the older codes

class Lighting : public frc2::SubsystemBase {
 public:
  static const frc::AddressableLED::LEDData BLACK;
  static const frc::AddressableLED::LEDData WHITE;
  static const frc::AddressableLED::LEDData GREEN;
  static const frc::AddressableLED::LEDData RED;
  static const frc::AddressableLED::LEDData BLUE;

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
  std::vector<frc::AddressableLED::LEDData> m_ledBuffer;
  frc::AddressableLED m_led;
};

