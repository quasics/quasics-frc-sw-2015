// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PivotScorer.h"

PivotScorer::PivotScorer() = default;

// This method will be called once per scheduler run
void PivotScorer::Periodic() {
}

void PivotScorer::SetScorerSpeed(double percentSpeed) {
  m_leftScorer.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput,
      percentSpeed);
  m_rightScorer.Set(
      ctre::phoenix::motorcontrol::VictorSPXControlMode::PercentOutput,
      percentSpeed);
}