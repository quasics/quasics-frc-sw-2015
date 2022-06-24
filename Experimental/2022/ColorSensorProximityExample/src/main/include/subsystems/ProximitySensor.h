// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rev/ColorSensorV3.h>
#include <iostream>

class ProximitySensor : public frc2::SubsystemBase {
 public:
  static constexpr auto BAD_PROXIMITY = static_cast<uint32_t>(-1);

  ProximitySensor();

  uint32_t GetProximity() { return m_colorSensor.IsConnected() ? m_colorSensor.GetProximity() : BAD_PROXIMITY; }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::ColorSensorV3 m_colorSensor{i2cPort};
};
