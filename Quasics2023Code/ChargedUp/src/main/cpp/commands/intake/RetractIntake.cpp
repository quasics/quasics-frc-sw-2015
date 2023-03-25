// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/RetractIntake.h"

#include "Constants.h"

RetractIntake::RetractIntake(IntakeDeployment* intake, double speed)
    : m_intakeDeployment(intake), intakeSpeed(-std::abs(speed)) {
  AddRequirements(m_intakeDeployment);
}

// Called when the command is initially scheduled.
void RetractIntake::Initialize() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
  m_intakeDeployment->EnableBraking(true);
  m_clocks = 0;
}

// Called repeatedly when this Command is scheduled to run
void RetractIntake::Execute() {
  m_intakeDeployment->SetMotorSpeed(intakeSpeed);
  m_clocks++;
}

// Called once the command ends or is interrupted.
void RetractIntake::End(bool interrupted) {
  m_intakeDeployment->Stop();
  m_intakeDeployment->EnableBraking(true);
}

// Returns true when the command should end.
bool RetractIntake::IsFinished() {
  if (m_intakeDeployment->IsIntakeDeployed(
          IntakeDeployment::LimitSwitch::Retracted)) {
    // Note: This can only happen if the intake limit switches are enabled.
    return true;
  }

#if defined(ENABLE_INTAKE_HARD_STOP_DETECTION)
  if (m_intakeDeployment->GetLeftVelocity() < Intake::STOP_VELOCITY &&
      m_clocks > Intake::CLOCKS_UNTIL_ABOVE_STOP_VELOCITY) {
    return true;
  }
#endif

  // Keep running until interrupted (e.g., control button released).
  return false;
}
