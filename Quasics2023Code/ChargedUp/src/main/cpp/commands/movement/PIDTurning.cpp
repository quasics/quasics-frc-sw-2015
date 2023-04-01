// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/movement/PIDTurning.h"

PIDTurning::PIDTurning(Drivebase* drivebase, units::degree_t angle)
    : m_drivebase(drivebase), m_angle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void PIDTurning::Initialize() {
  feedForward = true;
  activatePID = false;
  pid.Reset();
  pid.SetTolerance(1.0, 0);
  startingAngle = m_drivebase->GetYaw();
  FeedForward();
}

// Called repeatedly when this Command is scheduled to run
void PIDTurning::Execute() {
  FeedForward();
  if (std::abs((startingAngle + m_angle - currentAngle).value()) < 10) {
    feedForward = false;
  }
  if (!feedForward) {
    rotationCorrection = pid.Calculate(currentAngle.value(),
                                       startingAngle.value() + m_angle.value());
  }
  m_drivebase->ArcadeDrive(0, rotationCorrection);
}

// Called once the command ends or is interrupted.
void PIDTurning::End(bool interrupted) {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
}

// Returns true when the command should end.
bool PIDTurning::IsFinished() {
  if (std::abs((startingAngle + m_angle - currentAngle).value()) < 1) {
    return true;
  }
  return false;
}

void PIDTurning::FeedForward() {
  currentAngle = m_drivebase->GetYaw();
  if (feedForward) {
    if (m_angle > 0_deg) {
      m_drivebase->ArcadeDrive(0, 0.5);
    } else {
      m_drivebase->ArcadeDrive(0, -0.5);
    }
  }
}