// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDrive.h"

TankDrive::TankDrive(DriveBase* driveBase,
                     std::function<double()> leftPowerSupplier,
                     std::function<double()> rightPowerSupplier)
    : m_driveBase(driveBase),
      m_leftPowerSupplier(leftPowerSupplier),
      m_rightPowerSupplier(rightPowerSupplier) {
  AddRequirements(driveBase);
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {
  m_driveBase->TankDrive(m_leftPowerSupplier(), m_rightPowerSupplier());
}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
  m_driveBase->TankDrive(m_leftPowerSupplier(), m_rightPowerSupplier());
}

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {
  m_driveBase->Stop();
}
