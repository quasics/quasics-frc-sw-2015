// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/TriggerBasedRollerCommand.h"

TriggerBasedRollerCommand::TriggerBasedRollerCommand(
    IntakeRoller* intakeRoller, ConfigSettings* settings,
    frc::XboxController* xboxController)
    : m_intakeRoller(intakeRoller),
      m_settings(settings),
      m_controller(xboxController) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_intakeRoller);
}

// Called repeatedly when this Command is scheduled to run
void TriggerBasedRollerCommand::Execute() {
  bool intakingCubes = m_settings->intakingCubes;
  if (IsLeftTriggerPressed()) {
    if (intakingCubes) {
      m_intakeRoller->SetRollerSpeed(IntakeConstants::RollerSpeeds::CUBES);
    } else {
      m_intakeRoller->SetRollerSpeed(IntakeConstants::RollerSpeeds::CONES);
    }
  } else if (IsRightTriggerPressed()) {
    if (intakingCubes) {
      m_intakeRoller->SetRollerSpeed(-1 * IntakeConstants::RollerSpeeds::CUBES);
    } else {
      m_intakeRoller->SetRollerSpeed(-1 * IntakeConstants::RollerSpeeds::CONES);
    }
  } else {
    m_intakeRoller->Stop();
  }
}

// Called once the command ends or is interrupted.
void TriggerBasedRollerCommand::End(bool interrupted) {
  m_intakeRoller->Stop();
}