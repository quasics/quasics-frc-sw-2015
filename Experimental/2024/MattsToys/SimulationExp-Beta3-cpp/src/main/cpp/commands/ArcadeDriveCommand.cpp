// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArcadeDriveCommand.h"

ArcadeDriveCommand::ArcadeDriveCommand(IDrivebase& drivebase,
                                       frc::XboxController& controller)
    : m_drivebase(drivebase), m_controller(controller) {
  AddRequirements(dynamic_cast<frc2::Subsystem*>(&m_drivebase));
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
  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  const units::meters_per_second_t xSpeed =
      -m_speedLimiter.Calculate(m_controller.GetRawAxis(0)) *
      IDrivebase::MAX_SPEED;

  // Get the rate of angular rotation. We are inverting this because we want a
  // positive value when we pull to the left (remember, CCW is positive in
  // mathematics). Xbox controllers return positive values when you pull to
  // the right by default.
  const units::radians_per_second_t rot =
      -m_rotLimiter.Calculate(m_controller.GetRawAxis(1)) *
      IDrivebase::MAX_ANGULAR_SPEED;

  m_drivebase.arcadeDrive(xSpeed, rot);
}