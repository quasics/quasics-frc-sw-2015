// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/RunIntakeCubeOrConeToggleCommand.h"

// NOT NEEDED ANYMORE
RunIntakeCubeOrConeToggleCommand::RunIntakeCubeOrConeToggleCommand(
    IntakeRoller* intakeRoller, ConfigSettings* settings)
    : m_intakeRoller(intakeRoller), m_settings(settings) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intakeRoller);
}

// Called when the command is initially scheduled.
void RunIntakeCubeOrConeToggleCommand::Initialize() {
  UpdatingIntakeChoice();
}

// Called repeatedly when this Command is scheduled to run
void RunIntakeCubeOrConeToggleCommand::Execute() {
  UpdatingIntakeChoice();
}

// Called once the command ends or is interrupted.
void RunIntakeCubeOrConeToggleCommand::End(bool interrupted) {
  m_intakeRoller->Stop();
}

// Returns true when the command should end.
bool RunIntakeCubeOrConeToggleCommand::IsFinished() {
  return false;
}

void RunIntakeCubeOrConeToggleCommand::UpdatingIntakeChoice() {
  bool intakingCubes = m_settings->intakingCubes;
  if (intakingCubes) {
    m_intakeRoller->SetRollerSpeed(0.5);
  } else {
    m_intakeRoller->SetRollerSpeed(1.00);
  }
}
