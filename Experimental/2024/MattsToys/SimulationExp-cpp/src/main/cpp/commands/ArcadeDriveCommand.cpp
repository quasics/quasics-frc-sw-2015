// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArcadeDriveCommand.h"

ArcadeDriveCommand::ArcadeDriveCommand(IDrivebase& drivebase,
                                       PercentSupplier forwardSupplier,
                                       PercentSupplier rotationSupplier)
    : m_drivebase(drivebase),
      m_forwardSupplier(forwardSupplier),
      m_rotationSupplier(rotationSupplier) {
  AddRequirements(&m_drivebase);
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

void ArcadeDriveCommand::updateSpeeds() {
  const double forwardValue = m_forwardSupplier();
  const double rotationValue = m_rotationSupplier();

  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  const units::meters_per_second_t xSpeed =
      -m_speedLimiter.Calculate(forwardValue) * IDrivebase::MAX_SPEED;

  // Get the rate of angular rotation. We are inverting this because we want a
  // positive value when we pull to the left (remember, CCW is positive in
  // mathematics). Xbox controllers return positive values when you pull to
  // the right by default.
  const units::radians_per_second_t rot =
      -m_rotLimiter.Calculate(rotationValue) * IDrivebase::MAX_ANGULAR_SPEED;

  IDrivebase::logValue("Forward stick", forwardValue);
  IDrivebase::logValue("Rotation stick", rotationValue);

  m_drivebase.arcadeDrive(xSpeed, rot);
}