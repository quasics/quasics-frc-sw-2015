// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDrive.h"

TankDrive::TankDrive(IDrivebase& driveBase, PercentSupplier leftSupplier,
                     PercentSupplier rightSupplier)
    : m_driveBase(driveBase),
      m_leftSupplier(leftSupplier),
      m_rightSupplier(rightSupplier) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&m_driveBase);
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {
  updateSpeed();
}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
  updateSpeed();
}

void TankDrive::updateSpeed() {
  m_driveBase.tankDrive(m_leftSupplier(), m_rightSupplier());
}

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {
  m_driveBase.stop();
}