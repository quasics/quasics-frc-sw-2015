// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AmpScorer.h"

AmpScorer::AmpScorer()
    : m_ampScorer{MotorIds::SparkMax::AMP_MOTOR_ID,
                  rev::CANSparkMax::MotorType::kBrushless} {
  SetName("Amp Scorer");
}

// This method will be called once per scheduler run
void AmpScorer::SetAmpScorerSpeed(double percentSpeed) {
  m_ampScorer.Set(-percentSpeed);
}

void AmpScorer::Stop() {
  m_ampScorer.Set(0);
}