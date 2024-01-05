// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SimulatedDriveBase.h"

#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <numbers>

SimulatedDriveBase::SimulatedDriveBase() {
  SetName("SimulatedDriveBase");

  m_gyro.Reset();

  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  m_rightMotor.SetInverted(true);

  // Set the distance per pulse for the drive encoders. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_leftEncoder.SetDistancePerPulse(2 * std::numbers::pi *
                                    kWheelRadius.value() / kEncoderResolution);
  m_rightEncoder.SetDistancePerPulse(2 * std::numbers::pi *
                                     kWheelRadius.value() / kEncoderResolution);

  m_leftEncoder.Reset();
  m_rightEncoder.Reset();

  m_rightMotor.SetInverted(true);

  frc::SmartDashboard::PutData("Field", &m_fieldSim);
}

void SimulatedDriveBase::Periodic() {
  m_odometry.Update(m_gyro.GetRotation2d(), m_leftEncoder.GetDistance() * 1_m,
                    m_rightEncoder.GetDistance() * 1_m);
  m_fieldSim.SetRobotPose(m_odometry.GetPose());
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

void SimulatedDriveBase::setMotorSpeeds(double leftPercent,
                                        double rightPercent) {
  m_leftMotor.Set(leftPercent);
  m_rightMotor.Set(rightPercent);

  //   // The simulation stuff looks at motor *voltages*, so we need to convert
  //   // the "% speed" values to a corresponding reference voltage to make the
  //   // simulator work.
  //   auto leftVoltage =
  //       leftPercent * frc::RobotController::GetInputVoltage() * 1_V;
  //   auto rightVoltage =
  //       rightPercent * frc::RobotController::GetInputVoltage() * 1_V;
  //   m_leftMotor.SetVoltage(leftVoltage);
  //   m_rightMotor.SetVoltage(rightVoltage);
}