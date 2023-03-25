// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/ExtendIntakeAtSpeedForTime.h"

ExtendIntakeAtSpeedForTime::ExtendIntakeAtSpeedForTime(
    IntakeDeployment* IntakeDeployment, double speed, units::second_t time)
    : m_intakeDeployment(IntakeDeployment),
      intakeSpeed(std::abs(speed)),
      m_time(time) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_intakeDeployment);
  SetName("ExtendIntakeAtSpeedForTime");
}
// Called when the command is initially scheduled.
void ExtendIntakeAtSpeedForTime::Initialize() {
  m_stopWatch.Reset();
  m_stopWatch.Start();
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
}

// Called repeatedly when this Command is scheduled to run
void ExtendIntakeAtSpeedForTime::Execute() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
}

// Called once the command ends or is interrupted.
void ExtendIntakeAtSpeedForTime::End(bool interrupted) {
  m_intakeDeployment->Stop();
}

// Returns true when the command should end.
bool ExtendIntakeAtSpeedForTime::IsFinished() {
  if (m_stopWatch.HasElapsed(m_time)) {
    return true;
  }
  return false;
}
