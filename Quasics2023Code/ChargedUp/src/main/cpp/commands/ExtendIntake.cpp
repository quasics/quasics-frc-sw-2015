// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExtendIntake.h"

#include "Constants.h"

ExtendIntake::ExtendIntake(IntakeDeployment* IntakeDeployment, double speed)
    : m_intakeDeployment(IntakeDeployment), intakeSpeed(std::abs(speed)) {
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements(m_intakeDeployment);
}

// Called when the command is initially scheduled.
void ExtendIntake::Initialize() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
  m_intakeDeployment->EnableBraking(true);
}

// Called repeatedly when this Command is scheduled to run
void ExtendIntake::Execute() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
}

// Called once the command ends or is interrupted.
void ExtendIntake::End(bool interrupted) {
  m_intakeDeployment->Stop();
  m_intakeDeployment->EnableBraking(true);
}

// Returns true when the command should end.
bool ExtendIntake::IsFinished() {
  if (m_intakeDeployment->IsIntakeDeployed(
          IntakeDeployment::LimitSwitch::Extended)) {
    // Note: This can only happen if the intake limit switches are enabled.
    return true;
  }

#if defined(ENABLE_INTAKE_HARD_STOP_DETECTION)
  // CODE_REVIEW(ethan): Note that this is assuming that we instantly
  // exceed "STOP_VELOCITY" when we start moving.  It would probably be
  // better to do something a little more sophisticated, like see if we
  // get up to some velocity within X cycles (and stop if we don't), and
  // then look for when the velocity suddenly falls off/drops below some
  // value.
  if (m_intakeDeployment->GetLeftVelocity() < Intake::STOP_VELOCITY) {
    return true;
  }
#endif

  // Keep running until interrupted (e.g., control button released).
  return false;
}
