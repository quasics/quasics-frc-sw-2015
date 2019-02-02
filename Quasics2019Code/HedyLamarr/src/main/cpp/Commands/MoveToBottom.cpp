/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/MoveToBottom.h"
#include "Commands/ElevatorToBottom.h"
#include "Commands/LifterToBottom.h"
#include "Robot.h"

MoveToBottom::MoveToBottom() {
  Requires(Robot::elevator.get());
  Requires(Robot::lifter.get());

  AddParallel(new ElevatorToBottom);
  AddParallel(new LifterToBottom);
}
