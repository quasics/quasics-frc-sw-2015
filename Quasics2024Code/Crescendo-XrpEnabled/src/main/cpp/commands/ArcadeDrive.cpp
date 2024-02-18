// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArcadeDrive.h"

#include "Constants.h"

ArcadeDrive::ArcadeDrive(IDrivebase& drivebase, PercentSupplier forwardSupplier,
                         PercentSupplier rotationSupplier)
    : m_drivebase(drivebase),
      m_forwardSupplier(forwardSupplier),
      m_rotationSupplier(rotationSupplier) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&m_drivebase);
}

// Called when the command is initially scheduled.
void ArcadeDrive::Initialize() {
  updateSpeeds();
}

// Called repeatedly when this Command is scheduled to run
void ArcadeDrive::Execute() {
  updateSpeeds();
}

// Called once the command ends or is interrupted.
void ArcadeDrive::End(bool interrupted) {
  m_drivebase.stop();
}

void ArcadeDrive::updateSpeeds() {
  const double forwardVal = m_forwardSupplier();
  const double rotationVal = m_rotationSupplier();

  const units::meters_per_second_t fSpeed =
      forwardVal * RobotConstants::MAX_SPEED;
  const units::radians_per_second_t rSpeed =
      rotationVal * RobotConstants::MAX_ANGULAR_SPEED;

  m_drivebase.arcadeDrive(fSpeed, rSpeed);
}
