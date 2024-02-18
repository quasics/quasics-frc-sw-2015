// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TriggerBasedIntaking.h"

TriggerBasedIntaking::TriggerBasedIntaking(IntakeRoller& intake,
                                           frc::Joystick* driverController)
    : m_intake(intake), m_driverController(driverController) {
  AddRequirements(&m_intake);
}

// Called repeatedly when this Command is scheduled to run
void TriggerBasedIntaking::Execute() {
  if (m_driverController->GetRawAxis(
          OperatorConstants::LogitechGamePad::RightTriggerAxis > 0.5)) {
    m_intake.SetRollerSpeed(1.00);
  } else if (m_driverController->GetRawAxis(
                 OperatorConstants::LogitechGamePad::LeftTriggerAxis > 0.5)) {
    m_intake.SetRollerSpeed(-1.00);
  } else {
    m_intake.Stop();
  }
}

// Called once the command ends or is interrupted.
void TriggerBasedIntaking::End(bool interrupted) {
  m_intake.Stop();
}
