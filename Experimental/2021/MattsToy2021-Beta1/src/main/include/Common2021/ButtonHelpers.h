#pragma once

#include <frc/Joystick.h>

/**
 * Helper class for use in toggling some behavior on/off,
 * or move between discrete states (like changing channels)
 * in response to button presses on a joystick.
 *
 * This class monitors the state of the specified button
 * every time that "ShouldToggle()" is invoked, and will
 * indicate that we should make a switch whenever we see
 * the button go down (and remember what we saw last time,
 * so that we can leave the state alone until we see a
 * release and then a new button depress).
 */
class ButtonToggleMonitor {
 public:
  ButtonToggleMonitor(frc::Joystick& stick, int button)
      : m_stick(stick), m_button(button) {}

  void Reset() { m_wasDownLastTime = false; }

  bool ShouldToggle() {
    bool result = false;
    bool buttonCurrentlyDown = m_stick.GetRawButton(m_button);
    if (buttonCurrentlyDown && !m_wasDownLastTime) {
      result = true;
    }
    m_wasDownLastTime = buttonCurrentlyDown;
    return result;
  }

 private:
  frc::Joystick& m_stick;
  int m_button;
  bool m_wasDownLastTime{false};
};