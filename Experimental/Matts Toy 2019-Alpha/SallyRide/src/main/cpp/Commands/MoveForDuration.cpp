/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/MoveForDuration.h"
#include "Robot.h"

MoveForDuration::MoveForDuration(double timeout, double power)
    : TimedCommand(timeout), m_power(power) {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::m_driveBase);
}

// Called just before this Command runs the first time
void MoveForDuration::Initialize() {
  // Set power for both sides to 25%.
  Robot::m_driveBase.SetPowerToMotors(m_power, m_power);
}

// Called repeatedly when this Command is scheduled to run
void MoveForDuration::Execute() {}

// Called once after command times out
void MoveForDuration::End() {
  Robot::m_driveBase.Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveForDuration::Interrupted() {
  End();
}
