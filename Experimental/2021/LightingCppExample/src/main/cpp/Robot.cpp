// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/AddressableLED.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <array>

class Robot : public frc::TimedRobot {
  /// Maximum # of lights that we'll control (and the expected length of the
  /// strip, though something shorter *will* still function).
  static constexpr int kLength = 60;

  /// PWM port to which the LED strip is connected.
  ///
  /// Note that this *must* be a PWM header, not MXP or DIO.
  static constexpr int kLedPwmPortNum = 7;

  /// Software controller for addressable/digital LEDs.
  frc::AddressableLED m_led{kLedPwmPortNum};

  /// Buffer used to specify light values.  (Reusing a single buffer every time
  /// that we modify the lights, rather than constantly creating/releasing
  /// them.)
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;

  /// Store what the last hue of the first pixel is.
  int firstPixelHue = 0;

 public:
  void RobotInit() override {
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
  }

  void RobotPeriodic() override {
    // Fill the buffer with a rainbow
    UpdateLedBuffer();

    // Set the LEDs
    m_led.SetData(m_ledBuffer);
  }

 private:
  void UpdateLedBuffer() {
    // For every pixel
    for (int i = 0; i < kLength; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
      // Set the value
      m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
    }

    // Increase by to make the rainbow "move"
    firstPixelHue += 3;

    // Check bounds
    firstPixelHue %= 180;
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
