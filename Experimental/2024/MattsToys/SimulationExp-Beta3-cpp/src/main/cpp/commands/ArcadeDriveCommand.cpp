// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArcadeDriveCommand.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

ArcadeDriveCommand::ArcadeDriveCommand(IDrivebase& drivebase,
                                       frc::XboxController& controller)
    : m_drivebase(drivebase), m_controller(controller) {
  AddRequirements(&m_drivebase.asFrcSubsystem());
}

// Called when the command is initially scheduled.
void ArcadeDriveCommand::Initialize() {
  updateSpeeds();
}

// Called repeatedly when this Command is scheduled to run
void ArcadeDriveCommand::Execute() {
  updateSpeeds();
}

// Called once the command ends or is interrupted.
void ArcadeDriveCommand::End(bool interrupted) {
  m_drivebase.stop();
}

// Returns true when the command should end.
bool ArcadeDriveCommand::IsFinished() {
  return false;
}

void ArcadeDriveCommand::updateSpeeds() {
  const bool isReal = frc::RobotBase::IsReal();
  // TODO: add deadband code.
  const double leftStickValue =
      isReal ? m_controller.GetLeftX() : m_controller.GetRawAxis(0);
  const double rightStickValue =
      isReal ? m_controller.GetRightY() : m_controller.GetRawAxis(1);
  
  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  const units::meters_per_second_t xSpeed =
      -m_speedLimiter.Calculate(leftStickValue) * IDrivebase::MAX_SPEED;

  // Get the rate of angular rotation. We are inverting this because we want a
  // positive value when we pull to the left (remember, CCW is positive in
  // mathematics). Xbox controllers return positive values when you pull to
  // the right by default.
  const units::radians_per_second_t rot =
      -m_rotLimiter.Calculate(rightStickValue) * IDrivebase::MAX_ANGULAR_SPEED;

  frc::SmartDashboard::PutNumber("Left stick", leftStickValue);
  frc::SmartDashboard::PutNumber("Right stick", rightStickValue);
  frc::SmartDashboard::PutNumber("Forward speed", xSpeed.value());
  frc::SmartDashboard::PutNumber("Rotation speed", rot.value());

  m_drivebase.arcadeDrive(xSpeed, rot);
}