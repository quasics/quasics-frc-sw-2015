/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TankDrive.h"
#include "subsystems/DriveBase.h"
#include "Robot.h"
#include "OI.h"

const float kDeadBandSize = 0.015;

TankDrive::TankDrive() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::driveBase.get());
}

// Called just before this Command runs the first time
void TankDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
  double leftStickReading = Robot::m_oi.GetLeftJoystick();
  double rightStickReading = Robot::m_oi.GetRightJoystick();

  // Software dead-band handling.
  if (std::abs(leftStickReading) < kDeadBandSize) {
    leftStickReading = 0.0;
  }
  if (std::abs(rightStickReading) < kDeadBandSize) {
    rightStickReading = 0.0;
  }

  Robot::driveBase->SetMotorSpeed(
      leftStickReading * mult,
      rightStickReading * mult);
}

// Make this return true when this Command no longer needs to run execute()
bool TankDrive::IsFinished() { return false; }

// Called once after isFinished returns true
void TankDrive::End() {
  Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TankDrive::Interrupted() {
  End();
}
