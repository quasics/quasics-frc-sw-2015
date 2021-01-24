// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopTankDrive.h"

// Called repeatedly when this Command is scheduled to run
void TeleopTankDrive::Execute() {
  m_driveBase->TankDrive(m_leftSpeedSupplier(), m_rightSpeedSupplier());
}
