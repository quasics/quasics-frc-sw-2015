/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ExampleSubsystem.h"

ExampleSubsystem::ExampleSubsystem() {}

// This method will be called once per scheduler run
void ExampleSubsystem::Periodic() {}

void ExampleSubsystem::RotateShoulderDown() { shoulder.Set(0.25); }

void ExampleSubsystem::RotateShoulderUp() { shoulder.Set(-0.25); }

void ExampleSubsystem::StopMovingShoulder() { shoulder.Set(0); }
