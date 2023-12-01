// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>

#include "Robot.h"
#include "subsystems/SimulatedDriveBase.h"

RobotContainer::RobotContainer() {
  if (Robot::IsReal()) {
    // Configure real drive base
  } else {
    // Configure simulation drive base
    m_drivebase.reset(new SimulatedDriveBase);
  }
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  frc2::FunctionalCommand lightingExample(
      // onInit
      [&]() { m_lighting.setSolidStripColor(Lighting::GREEN); },
      // onExecute
      []() {},
      // onEnd
      [&](bool) { m_lighting.setSolidStripColor(Lighting::WHITE); },
      // isFinished
      []() { return false; },
      // Requirements
      {&m_lighting});
  m_lighting.SetDefaultCommand(std::move(lightingExample));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
