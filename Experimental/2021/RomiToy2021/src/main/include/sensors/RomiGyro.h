// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/interfaces/Gyro.h>
#include <hal/SimDevice.h>

class RomiGyro {
 public:
  RomiGyro();

  /**
   * Gets the rate of turn in degrees-per-second around the X-axis
   */
  double GetRateX();

  /**
   * Gets the rate of turn in degrees-per-second around the Y-axis
   */
  double GetRateY();

  /**
   * Gets the rate of turn in degrees-per-second around the Z-axis
   */
  double GetRateZ();

  /**
   * Gets the currently reported angle around the X-axis
   */
  double GetAngleX();

  /**
   * Gets the currently reported angle around the X-axis
   */
  double GetAngleY();

  /**
   * Gets the currently reported angle around the X-axis
   */
  double GetAngleZ();

  /**
   * Resets the gyro
   */
  void Reset();

  frc::Gyro &GetGyroX() { return m_simpleGyroX; }
  frc::Gyro &GetGyroY() { return m_simpleGyroY; }
  frc::Gyro &GetGyroZ() { return m_simpleGyroZ; }

private:
  hal::SimDevice m_simDevice;
  hal::SimDouble m_simRateX;
  hal::SimDouble m_simRateY;
  hal::SimDouble m_simRateZ;
  hal::SimDouble m_simAngleX;
  hal::SimDouble m_simAngleY;
  hal::SimDouble m_simAngleZ;

  double m_angleXOffset = 0;
  double m_angleYOffset = 0;
  double m_angleZOffset = 0;

  /**
   * Convenience class, used to expose each of the three axes on the Romi's
   * gyro as an independent entity.
   */
  class SimpleGyro : public frc::Gyro
  {
    friend class RomiGyro;

  private:
    SimpleGyro(std::function<void()> reset, std::function<double()> getAngle, std::function<double()> getRate)
        : m_reset(reset), m_getAngle(getAngle), m_getRate(getRate) {}

  public:
    // Functions overridden from the base type.
    Gyro &operator=(Gyro &&) = delete;
    void Reset() override { m_reset(); }
    double GetAngle() const override { return m_getAngle(); }
    double GetRate() const override { return m_getRate(); }

    // Stubbed only.
    void Calibrate() override {}

  private:
    std::function<void()> m_reset;
    std::function<double()> m_getAngle;
    std::function<double()> m_getRate;
  };

  SimpleGyro m_simpleGyroX;
  SimpleGyro m_simpleGyroY;
  SimpleGyro m_simpleGyroZ;
};
