// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TriggerBasedShooterCommand.h"

#include "FORCOMPETITIONRobotConstants.h"

TriggerBasedShooterCommand::TriggerBasedShooterCommand(
    Shooter* shooter, frc::XboxController* xboxController)
    : m_shooter(shooter), m_controller(xboxController) {
  AddRequirements(m_shooter);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called repeatedly when this Command is scheduled to run
void TriggerBasedShooterCommand::Execute() {
  if (m_controller->GetRawAxis(frc::XboxController::Axis::kLeftTrigger) > 0.5) {
    m_shooter->SetFlywheelSpeed(RobotValues::SLOW_SHOOTER_SPEED);
    m_shooter->SetRollerSpeed(RobotValues::SLOW_SHOOTER_BACKROLLER_SPEED);
  } else if (m_controller->GetRawAxis(
                 frc::XboxController::Axis::kRightTrigger) > 0.5) {
    m_shooter->SetFlywheelSpeed(RobotValues::FAST_SHOOTER_SPEED);
    m_shooter->SetRollerSpeed(RobotValues::FAST_SHOOTER_BACKROLLER_SPEED);
  } else {
    m_shooter->Stop();
  }
}

// Called once the command ends or is interrupted.
void TriggerBasedShooterCommand::End(bool interrupted) {
  m_shooter->Stop();
}
