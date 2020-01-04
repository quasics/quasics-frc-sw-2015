/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TankDriveCommand.h"

TankDriveCommand::TankDriveCommand(DriveBase* driveBase,
                                   std::function<double()> leftPower,
                                   std::function<double()> rightPower)
    : driveBase(driveBase), m_leftPower(leftPower), m_rightPower(rightPower) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void TankDriveCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TankDriveCommand::Execute() {
  driveBase->SetMotorPower(m_leftPower(), m_rightPower());
}

// Called once the command ends or is interrupted.
void TankDriveCommand::End(bool interrupted) { driveBase->Stop(); }

// Returns true when the command should end.
bool TankDriveCommand::IsFinished() { return false; }
