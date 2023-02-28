// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TriggerBasedRollerCommand.h"

TriggerBasedRollerCommand::TriggerBasedRollerCommand(
    IntakeRoller* intakeRoller, frc::XboxController* xboxController)
    : m_intakeRoller(intakeRoller), m_controller(xboxController) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_intakeRoller);
}

// Called repeatedly when this Command is scheduled to run
void TriggerBasedRollerCommand::Execute() {
  if (m_controller->GetRawAxis(frc::XboxController::Axis::kLeftTrigger) > 0.5) {
    m_intakeRoller->SetRollerSpeed(-0.47);
    // TODO GO TO CLASSES AND FINED NAMED WAYS OF JOYSTICK
    // WEIRD BUG THIS ONE READS FROM DIFFERENT TRIGGERS
  } /*else if (m_controller->GetRawAxis(frc::XboxController::Axis::kLeftTrigger)
       < -0.5) {*/
  else if (m_controller->GetRawAxis(frc::XboxController::Axis::kRightTrigger) >
           0.5) {
    m_intakeRoller->SetRollerSpeed(0.47);
  } else {
    m_intakeRoller->Stop();
  }
}

// Called once the command ends or is interrupted.
void TriggerBasedRollerCommand::End(bool interrupted) {
  m_intakeRoller->Stop();
}
