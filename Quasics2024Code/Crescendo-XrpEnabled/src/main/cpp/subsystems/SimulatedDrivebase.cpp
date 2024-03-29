// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SimulatedDrivebase.h"

#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <numbers>

#include "Constants.h"
#include "utils/SimulationSupport.h"

SimulatedDrivebase::SimulatedDrivebase()
    : m_leftMotor{PWMPorts::SIMULATED_LEFT_MOTOR_PORT},
      m_rightMotor{PWMPorts::SIMULATED_RIGHT_MOTOR_PORT},
      m_leftEncoder{DigitalInput::SIMULATED_LEFT_ENCODER_A_PORT,
                    DigitalInput::SIMULATED_LEFT_ENCODER_B_PORT},
      m_rightEncoder{DigitalInput::SIMULATED_RIGHT_ENCODER_A_PORT,
                     DigitalInput::SIMULATED_RIGHT_ENCODER_B_PORT} {
  SetName("SimulatedDrivebase");

  m_gyro.Reset();

  // motor needs to be inverted because when "mounted", they'll be facing
  // opposite directions
  m_rightMotor.SetInverted(true);

  // Sets distance per pulse of encoders.
  m_leftEncoder.SetDistancePerPulse(2 * std::numbers::pi *
                                    kWheelRadius.value() / kEncoderResolution);
  m_rightEncoder.SetDistancePerPulse(2 * kWheelRadius.value() /
                                     kEncoderResolution);

  m_leftEncoder.Reset();
  m_rightEncoder.Reset();

  frc::SmartDashboard::PutData("Field", &m_fieldSim);
}

// This method will be called once per scheduler run
void SimulatedDrivebase::Periodic() {
  IDrivebase::Periodic();
  m_fieldSim.SetRobotPose(m_odometry.GetPose());
}

void SimulatedDrivebase::SimulationPeriodic() {
  // Updating the simulation is setting the motor voltage inputs, updating the
  // simulation, and writing the simulated positions and velocities to the
  // simulated encoder and gyro.
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

  SimulationSupport::setSimulatedPose(m_drivetrainSimulator.GetPose());
}

void SimulatedDrivebase::setMotorSpeeds_HAL(double leftPercent,
                                            double rightPercent) {
  m_leftMotor.Set(leftPercent);
  m_rightMotor.Set(rightPercent);
}

units::volt_t SimulatedDrivebase::getLeftVoltage_HAL() {
  return m_leftMotor.Get() * frc::RobotController::GetBatteryVoltage();
}

units::volt_t SimulatedDrivebase::getRightVoltage_HAL() {
  return m_rightMotor.Get() * frc::RobotController::GetBatteryVoltage();
}
