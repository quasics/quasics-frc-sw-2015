// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SimulatedDriveBase.h"

#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

// Gains are for example purposes only - must be determined for your own
// robot!
constexpr auto kS = 1_V;
constexpr auto kV = 3 * (1_V * 1_s / 1_m);
constexpr auto kA = 0 * (1_V * 1_s * 1_s / 1_m);

constexpr double kP = 8.5;
constexpr double kI = 0;
constexpr double kD = 0;
constexpr units::length::meter_t SIMULATED_TRACK_WIDTH_METERS = 0.381_m * 2;

SimulatedDriveBase::SimulatedDriveBase()
    : IDrivebase(SIMULATED_TRACK_WIDTH_METERS, kP, kI, kD, kS, kV, kA),
      m_odometry{frc::Rotation2d(), units::meter_t(0), units::meter_t(0)} {
  SetName("SimulatedDriveBase");
  m_gyro.Reset();

  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  m_rightMotor.SetInverted(true);

  // Set the distance per pulse for the drive encoders. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_leftEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius /
                                    kEncoderResolution);
  m_rightEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius /
                                     kEncoderResolution);

  m_leftEncoder.Reset();
  m_rightEncoder.Reset();

  m_rightMotor.SetInverted(true);

  frc::SmartDashboard::PutData("Field", &m_fieldSim);
}

void SimulatedDriveBase::SimulationPeriodic() {
  // To update our simulation, we set motor voltage inputs, update the
  // simulation, and write the simulated positions and velocities to our
  // simulated encoder and gyro. We negate the right side so that positive
  // voltages make the right side move forward.
  m_drivetrainSimulator.SetInputs(units::volt_t{m_leftMotor.Get()} *
                                      frc::RobotController::GetInputVoltage(),
                                  units::volt_t{m_rightMotor.Get()} *
                                      frc::RobotController::GetInputVoltage());
  m_drivetrainSimulator.Update(20_ms);

  m_leftEncoderSim.SetDistance(m_drivetrainSimulator.GetLeftPosition().value());
  m_leftEncoderSim.SetRate(m_drivetrainSimulator.GetLeftVelocity().value());
  m_rightEncoderSim.SetDistance(
      m_drivetrainSimulator.GetRightPosition().value());
  m_rightEncoderSim.SetRate(m_drivetrainSimulator.GetRightVelocity().value());
  m_gyroSim.SetAngle(-m_drivetrainSimulator.GetHeading().Degrees().value());
}
