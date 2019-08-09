/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#ifdef ENABLE_OLD_ELEVATOR
#include "Commands/MoveToBottom.h"
#include "Commands/ElevatorToBottom.h"
#include "Commands/LifterToBottom.h"
#include "Robot.h"

MoveToBottom::MoveToBottom() {
  AddParallel(new ElevatorToBottom);
  AddParallel(new LifterToBottom);
}
#endif // ENABLE_OLD_ELEVATOR