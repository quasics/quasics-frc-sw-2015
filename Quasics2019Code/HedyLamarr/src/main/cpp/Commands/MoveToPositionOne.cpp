/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/MoveToPositionOne.h"
#include "Commands/ElevatorToPositionOne.h"
#include "Commands/LifterToPositionOne.h"
#include "Robot.h"

MoveToPositionOne::MoveToPositionOne() {
  Requires(Robot::elevator.get());
  Requires(Robot::lifter.get());
  AddParallel(new ElevatorToPositionOne);
  AddParallel(new LifterToPositionOne);
}
