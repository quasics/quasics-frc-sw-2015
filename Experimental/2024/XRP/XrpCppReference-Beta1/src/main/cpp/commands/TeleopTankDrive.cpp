// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopTankDrive.h"

TeleopTankDrive::TeleopTankDrive(Drivetrain* subsystem,
                                 std::function<double()> leftSpeedSupplier,
                                 std::function<double()> rightSpeedSupplier)
    : m_drive(subsystem),
      m_leftSpeedSupplier(leftSpeedSupplier),
      m_rightSpeedSupplier(rightSpeedSupplier) {
  AddRequirements(m_drive);
}

void TeleopTankDrive::Execute() {
  m_drive->TankDrive(m_leftSpeedSupplier(), m_rightSpeedSupplier());
}
