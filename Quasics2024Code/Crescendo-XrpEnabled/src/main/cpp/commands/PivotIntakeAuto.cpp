// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PivotIntakeAuto.h"

#include <iostream>

PivotIntakeAuto::PivotIntakeAuto(IntakeDeployment& IntakeDeployment,
                                 double speed, bool extend)
    : m_intakeDeployment(IntakeDeployment),
      m_intakeDeploymentSpeed(extend ? std::abs(speed) : -std::abs(speed)),
      m_extending(extend) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&m_intakeDeployment);
}

// Called when the command is initially scheduled.
void PivotIntakeAuto::Initialize() {
  if (m_extending) {
    m_intakeDeployment.ResetEncoders();
  }

  m_intakeSlewRateLimiter.Reset(0);
  double speed = m_intakeSlewRateLimiter.Calculate(m_intakeDeploymentSpeed);
  std::cout << "Speed sending: " << speed << std::endl;
  m_intakeDeployment.SetMotorSpeed(speed);
  m_intakeDeployment.EnableBraking(false);
}

// Called repeatedly when this Command is scheduled to run
void PivotIntakeAuto::Execute() {
  double speed = m_intakeSlewRateLimiter.Calculate(m_intakeDeploymentSpeed);
  std::cout << "Speed sending: " << speed << std::endl;
  m_intakeDeployment.SetMotorSpeed(speed);
}

// Called once the command ends or is interrupted.
void PivotIntakeAuto::End(bool interrupted) {
  m_intakeDeployment.Stop();
  m_intakeDeployment.EnableBraking(false);
}

// Returns true when the command should end.
bool PivotIntakeAuto::IsFinished() {
  if (m_extending) {
    if (m_intakeDeployment.ExtendedByRevolutions()) {
      return true;
    }
    return false;
  } else {
    if (m_intakeDeployment.RetractedByRevolutions()) {
      return true;
    }
    return false;
  }
  return false;
}
