// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDrive.h"

TankDrive::TankDrive(Drivetrain *driveTrain,
                     std::function<double()> leftSpeedSupplier,
                     std::function<double()> rightRotateSupplier)
    : m_driveTrain(driveTrain), m_leftSpeedSupplier(leftSpeedSupplier), m_rightSpeedSupplier(rightRotateSupplier)
{
  AddRequirements(driveTrain);
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {
  m_driveTrain->TankDrive(m_leftSpeedSupplier(), m_rightSpeedSupplier());
}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
  m_driveTrain->TankDrive(m_leftSpeedSupplier(), m_rightSpeedSupplier());
}

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {
  m_driveTrain->Stop();
}
