/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#ifdef ENABLE_OLD_ELEVATOR
#include "Commands/MoveToPositionTwo.h"
#include "Commands/ElevatorToPositionTwo.h"
#include "Commands/LifterToPositionTwo.h"
#include "Robot.h"

MoveToPositionTwo::MoveToPositionTwo() {
  AddParallel(new ElevatorToPositionTwo);
  AddParallel(new LifterToPositionTwo);
}
#endif // ENABLE_OLD_ELEVATOR
