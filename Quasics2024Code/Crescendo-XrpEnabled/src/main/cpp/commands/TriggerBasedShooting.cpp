// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TriggerBasedShooting.h"

TriggerBasedShooting::TriggerBasedShooting(Shooter& shooter,
                                           frc::XboxController* xboxController)
    : m_shooter(shooter), m_controller(xboxController) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&m_shooter);
}

// Called repeatedly when this Command is scheduled to run
void TriggerBasedShooting::Execute() {
  if (m_controller->GetRawAxis(frc::XboxController::Axis::kLeftTrigger) > 0.5) {
    m_shooter.SetFlywheelSpeed(ShooterSpeeds::amp);
  } else if (m_controller->GetRawAxis(
                 frc::XboxController::Axis::kRightTrigger) > 0.5) {
    m_shooter.SetFlywheelSpeed(ShooterSpeeds::speaker);
  } else {
    m_shooter.Stop();
  }
}

// Called once the command ends or is interrupted.
void TriggerBasedShooting::End(bool interrupted) {
  m_shooter.Stop();
}
