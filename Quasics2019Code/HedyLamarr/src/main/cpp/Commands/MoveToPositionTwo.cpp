/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/MoveToPositionTwo.h"
#include "Commands/ElevatorToPositionTwo.h"
#include "Commands/LifterToPositionTwo.h"
#include "Robot.h"
MoveToPositionTwo::MoveToPositionTwo() {
  Requires(Robot::elevator.get());
  Requires(Robot::lifter.get());
  AddParallel(new ElevatorToPositionTwo);
  AddParallel(new LifterToPositionTwo);
}

