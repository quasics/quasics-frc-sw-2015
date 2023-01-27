// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"


Drivebase::Drivebase() {
    SetName("Drivebase");

  m_leftSide.reset(new frc::MotorControllerGroup(m_leftFront, m_leftBack));
  m_rightSide.reset(new frc::MotorControllerGroup(m_rightFront, m_rightBack));

  m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));


}

void Drivebase::SetBrakingMode(bool enabled) {
  if (enabled) {
    m_leftFront.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightFront.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_leftBack.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightBack.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  } else {
    m_leftFront.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightFront.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_leftBack.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightBack.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  }
}

units::meter_t Drivebase::GetLeftDistance() {
  // Note that the conversion factor configured earlier means that we're getting
  // position in meters.
  return units::meter_t(m_leftFrontEncoder.GetPosition());
}

units::meter_t Drivebase::GetRightDistance() {
  // Note that the conversion factor configured earlier means that we're getting
  // position in meters.
  return units::meter_t(m_rightFrontEncoder.GetPosition());
}

units::meters_per_second_t Drivebase::GetLeftVelocity() {
  // Note that the conversion factor configured earlier means that we're getting
  // velocity in m/sec.
  return units::meters_per_second_t(m_leftFrontEncoder.GetVelocity());
}

units::meters_per_second_t Drivebase::GetRightVelocity() {
  // Note that the conversion factor configured earlier means that we're getting
  // velocity in m/sec.
  return units::meters_per_second_t(m_rightFrontEncoder.GetVelocity());
}

void Drivebase::ResetEncoders() {
  m_leftFrontEncoder.SetPosition(0);
  m_rightFrontEncoder.SetPosition(0);
  m_leftBackEncoder.SetPosition(0);
  m_rightBackEncoder.SetPosition(0);
}

units::degree_t Drivebase::GetAngle() {
  return m_gyro.GetRotation2d().Degrees();
}

// This method will be called once per scheduler run
void Drivebase::Periodic(){

}

void Drivebase::TankDrive(double leftPower, double rightPower) {
    m_drive->TankDrive(leftPower, rightPower);
}