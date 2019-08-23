/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/TestHardStops.h"
#include "Robot.h"
#include <iostream>

TestHardStops::TestHardStops() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::newElevator.get());
}

// Called just before this Command runs the first time
void TestHardStops::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TestHardStops::Execute() {
  std::string topStop = "off";
  std::string bottomStop = "off";
  std::string lowHallEffect = "off";
  std::string mediumHallEffect = "off";

  if(Robot::newElevator->atTop()) {
    topStop = "on";
  }
  if(Robot::newElevator->atBottom()) {
    bottomStop = "on";
  }
  if(Robot::newElevator->atLow()) {
    lowHallEffect = "on";
  }
  if(Robot::newElevator->atMedium()) {
    mediumHallEffect = "on";
  }

  std::cerr << "Top stop: " << topStop << "Medium hall effect: " << mediumHallEffect << "Low hall effect: " << lowHallEffect << "Bottom stop: " << bottomStop << std::endl;
}

// Make this return true when this Command no longer needs to run execute()
bool TestHardStops::IsFinished() { return false; }

// Called once after isFinished returns true
void TestHardStops::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TestHardStops::Interrupted() {}
