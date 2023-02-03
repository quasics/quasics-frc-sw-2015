// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveForDistance.h"

#include <iostream>

#undef CAN_SET_BREAKING_MODE  // Only supported on the big bot

constexpr double MOTOR_SPEED = 0.5;

// Called when the command is initially scheduled.
void DriveForDistance::Initialize()
{
  m_startingPosition = m_driveBase->GetRightDistance();
  m_endPosition = m_startingPosition + m_distance;
#ifdef CAN_SET_BREAKING_MODE
  m_driveBase->EnableBreakingMode(true);
#endif

  m_driveBase->TankDrive(MOTOR_SPEED, MOTOR_SPEED);
}

// Called repeatedly when this Command is scheduled to run
void DriveForDistance::Execute()
{
  m_driveBase->TankDrive(MOTOR_SPEED, MOTOR_SPEED);
}

// Called once the command ends or is interrupted.
void DriveForDistance::End(bool interrupted)
{
  m_driveBase->Stop();
}

// Returns true when the command should end.
bool DriveForDistance::IsFinished()
{
  units::inch_t current = m_driveBase->GetRightDistance();

  std::cerr << "Driving for distance: current = " << current.value() << ", end = " << m_endPosition.value() << std::endl;

  return m_endPosition <= current;
}
