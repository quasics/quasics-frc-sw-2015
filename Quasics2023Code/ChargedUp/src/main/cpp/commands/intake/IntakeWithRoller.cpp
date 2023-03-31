// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/IntakeWithRoller.h"

IntakeWithRoller::IntakeWithRoller(IntakeRoller* intakeRoller, double power)
    : m_intakeRoller(intakeRoller), m_power(-std::abs(power)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intakeRoller);
  SetName("IntakeWithRoller");
}

// Called when the command is initially scheduled.
void IntakeWithRoller::Initialize() {
  m_intakeRoller->SetRollerSpeed(m_power);
}

// Called repeatedly when this Command is scheduled to run
void IntakeWithRoller::Execute() {
  m_intakeRoller->SetRollerSpeed(m_power);
}

// Called once the command ends or is interrupted.
void IntakeWithRoller::End(bool interrupted) {
  m_intakeRoller->Stop();
}

// Returns true when the command should end.
bool IntakeWithRoller::IsFinished() {
  return false;
}
