// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopTankDrive.h"

TeleopTankDrive::TeleopTankDrive(Drivetrain *driveTrain,
                                 std::function<double()> leftSpeedSupplier,
                                 std::function<double()> rightSpeedSupplier)
    : m_driveTrain(driveTrain), m_leftSpeedSupplier(leftSpeedSupplier), m_rightSpeedSupplier(rightSpeedSupplier)
{
  AddRequirements({driveTrain});
}

// Called repeatedly when this Command is scheduled to run
void TeleopTankDrive::Execute()
{
  m_driveTrain->TankDrive(m_leftSpeedSupplier(), m_rightSpeedSupplier());
}

// Called once the command ends or is interrupted.
void TeleopTankDrive::End(bool interrupted)
{
  m_driveTrain->Stop();
}
