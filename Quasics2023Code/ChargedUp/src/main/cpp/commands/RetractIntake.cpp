// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RetractIntake.h"

#include "Constants.h"

RetractIntake::RetractIntake(IntakeDeployment* intake, double speed)
    : m_intakeDeployment(intake), intakeSpeed(-std::abs(speed)) {
  AddRequirements(m_intakeDeployment);
}

// Called when the command is initially scheduled.
void RetractIntake::Initialize() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
  m_intakeDeployment->EnableBraking(true);
}

// Called repeatedly when this Command is scheduled to run
void RetractIntake::Execute() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
}

// Called once the command ends or is interrupted.
void RetractIntake::End(bool interrupted) {
  m_intakeDeployment->Stop();
  m_intakeDeployment->EnableBraking(true);
}

// Returns true when the command should end.
bool RetractIntake::IsFinished() {
#ifdef ENABLE_INTAKE_LIMIT_SWITCH
  return m_intakeDeployment->IsIntakeDeployed(false);
#elif defined(ENABLE_INTAKE_HARD_STOP_DETECTION)
  // CODE_REVIEW(ethan): Note that this is assuming that we instantly
  // exceed "STOP_VELOCITY" when we start moving.  It would probably be
  // better to do something a little more sophisticated, like see if we
  // get up to some velocity within X cycles (and stop if we don't), and
  // then look for when the velocity suddenly falls off/drops below some
  // value.
  return m_intakeDeployment->GetLeftVelocity() < Intake::STOP_VELOCITY;
#else
  // Keep running until interrupted (e.g., control button released).
  return false;
#endif
}
