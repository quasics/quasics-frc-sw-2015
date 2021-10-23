// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class BallIntake : public frc2::SubsystemBase {
 private:
  static constexpr double INTAKE_FORWARD_SPEED = .75;
  static constexpr double INTAKE_REVERSE_SPEED = -.75;
  static constexpr double CONVEYOR_FORWARD_SPEED = 1.0;
  static constexpr double CONVEYOR_REVERSE_SPEED = -0.25;

 public:
  BallIntake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void IntakeOn() {
    m_intakeMotor.Set(INTAKE_FORWARD_SPEED);
  }
  void IntakeOnReverse() {
    m_intakeMotor.Set(INTAKE_REVERSE_SPEED);
  }
  void IntakeOff() {
    m_intakeMotor.Set(0);
  }

  void ConveyorOn() {
    m_conveyorMotor.Set(CONVEYOR_FORWARD_SPEED);
  }
  void ConveyorOnReverse() {
    m_conveyorMotor.Set(CONVEYOR_REVERSE_SPEED);
  }
  void ConveyorOff() {
    m_intakeMotor.Set(0);
  }

 private:
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intakeMotor{
      CANBusIds::VictorSPXIds::IntakeMotor};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyorMotor{
      CANBusIds::VictorSPXIds::ConveyorMotor};
};
