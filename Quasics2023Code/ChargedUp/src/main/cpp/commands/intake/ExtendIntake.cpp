// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/ExtendIntake.h"

#include "Constants.h"

ExtendIntake::ExtendIntake(IntakeDeployment* IntakeDeployment, double speed)
    : m_intakeDeployment(IntakeDeployment), intakeSpeed(std::abs(speed)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_intakeDeployment);
  SetName("ExtendIntake");
}

// Called when the command is initially scheduled.
void ExtendIntake::Initialize() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
  m_intakeDeployment->EnableBraking(true);
  m_clocks = 0;
}

// Called repeatedly when this Command is scheduled to run
void ExtendIntake::Execute() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
  m_clocks++;
}

// Called once the command ends or is interrupted.
void ExtendIntake::End(bool interrupted) {
  m_intakeDeployment->Stop();
  m_intakeDeployment->EnableBraking(true);
}

// Returns true when the command should end.
bool ExtendIntake::IsFinished() {
  /*if (m_intakeDeployment->IsIntakeDeployed(
          IntakeDeployment::LimitSwitch::Extended)) {
    // Note: This can only happen if the intake limit switches are enabled.
    return true;
  }*/

#if defined(ENABLE_INTAKE_HARD_STOP_DETECTION)

  if (m_intakeDeployment->GetLeftVelocity() < Intake::STOP_VELOCITY &&
      m_clocks > Intake::CLOCKS_UNTIL_ABOVE_STOP_VELOCITY) {
    return true;
  }
#endif

  // Keep running until interrupted (e.g., control button released).
  return false;
}
