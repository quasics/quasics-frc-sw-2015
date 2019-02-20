/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/TestLimitSwitches.h"
#include "Robot.h"
#include <iostream>

TestLimitSwitches::TestLimitSwitches() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::elevator.get());
  Requires(Robot::lifter.get());
}

// Called just before this Command runs the first time
void TestLimitSwitches::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TestLimitSwitches::Execute() {
  std::string elevatorTop = "off";
  std::string elevatorBottom = "off";
  std::string lifterTop = "off";
  std::string lifterBottom = "off";
  if(Robot::elevator->atTop()){
    elevatorTop = "on ";
  }
  if(Robot::elevator->atBottom()){
    elevatorBottom = "on ";
  }
  if(Robot::lifter->atTop()){
    lifterTop = "on ";
  }
  if(Robot::lifter->atBottom()){
    lifterBottom = "on ";
  }

  std::cerr << "Elevator top: " << elevatorTop << " bottom: " << elevatorBottom << " Lifter top: " << lifterTop << " bottom: " << lifterBottom
            << std::endl;
  
}

// Make this return true when this Command no longer needs to run execute()
bool TestLimitSwitches::IsFinished() { return false; }

// Called once after isFinished returns true
void TestLimitSwitches::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TestLimitSwitches::Interrupted() {}
