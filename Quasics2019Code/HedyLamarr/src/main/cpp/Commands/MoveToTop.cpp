/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/MoveToTop.h"
#include "Commands/ElevatorToTop.h"
#include "Commands/LifterToTop.h"
#include "Robot.h"

MoveToTop::MoveToTop() {
  Requires(Robot::elevator.get());
  Requires(Robot::lifter.get());

  AddParallel(new ElevatorToTop);
  AddParallel(new LifterToTop);
}