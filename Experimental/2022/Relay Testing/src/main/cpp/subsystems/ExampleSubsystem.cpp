// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExampleSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ExampleSubsystem::ExampleSubsystem() {
  // Implementation of subsystem constructor goes here.
}

void ExampleSubsystem::Periodic() {
  frc::SmartDashboard::PutString("Limit switch",
                                 intakeLimitSwitch.Get() ? "open" : "closed");
  // Implementation of subsystem periodic method goes here.
}

void ExampleSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}


