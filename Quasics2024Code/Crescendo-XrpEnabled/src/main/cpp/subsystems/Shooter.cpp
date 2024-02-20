// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

#include <units/time.h>

#include "Constants.h"

Shooter::Shooter()
    : m_flyWheel{MotorIds::SparkMax::SHOOTER_FLYWHEEL_MOTOR_LEADER_ID,
                 rev::CANSparkMax::MotorType::kBrushless},
      m_flyWheelTwo{MotorIds::SparkMax::SHOOTER_FLYWHEEL_MOTOR_FOLLOWER_ID,
                    rev::CANSparkMax::MotorType::kBrushless} {
  SetName("Shooter");
}

void Shooter::SetFlywheelSpeed(double percentSpeed) {
  m_flyWheel.Set(percentSpeed);
  m_flyWheelTwo.Set(-percentSpeed);
}