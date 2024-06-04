// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TransitionRoller.h"

TransitionRoller::TransitionRoller()
    : m_transition{MotorIds::SparkMax::TRANSITION_MOTOR,
                   rev::CANSparkMax::MotorType::kBrushless} {
  SetName("Intake Roller");
}

void TransitionRoller::SetTransitionRollerSpeed(double percentSpeed) {
  m_transition.Set(-percentSpeed);
}

void TransitionRoller::Stop() {
  m_transition.Set(0);
}