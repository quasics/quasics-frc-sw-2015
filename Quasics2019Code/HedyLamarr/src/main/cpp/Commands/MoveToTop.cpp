/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#ifdef ENABLE_OLD_ELEVATOR
#include "Commands/MoveToTop.h"
#include "Commands/ElevatorToTop.h"
#include "Commands/LifterToTop.h"
#include "Robot.h"

MoveToTop::MoveToTop() {
  AddParallel(new ElevatorToTop);
  AddParallel(new LifterToTop);
}
#endif // ENABLE_OLD_ELEVATOR