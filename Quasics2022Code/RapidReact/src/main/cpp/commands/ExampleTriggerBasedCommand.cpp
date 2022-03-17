// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExampleTriggerBasedCommand.h"

#include "Constants.h"
#include "subsystems/Lighting.h"

ExampleTriggerBasedCommand::ExampleTriggerBasedCommand(
    Lighting* lighting, frc::XboxController* xboxController)
    : m_lighting(lighting), m_controller(xboxController) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_lighting);
}

// Called when the command is initially scheduled.
void ExampleTriggerBasedCommand::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void ExampleTriggerBasedCommand::Execute() {
  if (m_controller->GetRawAxis(frc::XboxController::Axis::kLeftTrigger) > 0.5) {
    // When left trigger is triggered, set the color to red
    m_lighting->SetAllToColor(frc::Color(1, 0, 0));
  } else if (m_controller->GetRawAxis(
                 frc::XboxController::Axis::kRightTrigger) > 0.5) {
    // When right trigger is triggered, set the color to blue
    m_lighting->SetAllToColor(frc::Color(0, 0, 1));
  } else {
    m_lighting->SetAllToColor(frc::Color(0, 0, 0));
  }
}

// Called once the command ends or is interrupted.
void ExampleTriggerBasedCommand::End(bool interrupted) {
  m_lighting->SetAllToColor(frc::Color(0, 0, 0));
}

// Returns true when the command should end.
bool ExampleTriggerBasedCommand::IsFinished() {
  // This command will never end.
  return false;
}
