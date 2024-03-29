// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Button.h>

#include "commands/TeleopArcadeDrive.h"
#include "commands/TankDrive.h"

RobotContainer::RobotContainer() {
  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Also set default commands here
#ifdef USE_ARCADE_DRIVE
  m_drive.SetDefaultCommand(TankDrive(
      &m_drive, [this] { return -m_controller.GetRawAxis(1); },
      [this] { return -m_controller.GetRawAxis(2); }));
#else  // USE_ARCADE_DRIVE
  // Note that the axis #s are from Mr. Healy's wireless controller.
  // Values for Quasics *wired* controllers may be different.
  m_drive.SetDefaultCommand(TankDrive(
      &m_drive, [this] { return -m_controller.GetRawAxis(1); },
      [this] { return -m_controller.GetRawAxis(5); }));
#endif  // USE_ARCADE_DRIVE

  // Example of how to use the onboard IO
  m_onboardButtonA.OnTrue(frc2::cmd::Print("Button A Pressed"))
      .OnFalse(frc2::cmd::Print("Button A Released"));

  // Setup SmartDashboard options.
  m_chooser.SetDefaultOption("Auto Routine Distance", &m_autoDistance);
  m_chooser.AddOption("Auto Routine Time", &m_autoTime);
  frc::SmartDashboard::PutData("Auto Selector", &m_chooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_chooser.GetSelected();
}
