// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "Constants.h"
#include "commands/TeleopArcadeDrive.h"
#include "commands/TeleopTankDrive.h"

#undef DRIVE_ARCADE_STYLE

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here
#ifdef DRIVE_ARCADE_STYLE
  m_drive.SetDefaultCommand(TeleopArcadeDrive(
      &m_drive, [this] { return -m_controller.GetRawAxis(JoystickDefinitions::GameSirPro::LeftVertical); },
      [this] { return m_controller.GetRawAxis(JoystickDefinitions::GameSirPro::LeftHorizontal); }));
#else
  m_drive.SetDefaultCommand(TeleopTankDrive(
      &m_drive,
      [this] { return -m_controller.GetRawAxis(JoystickDefinitions::GameSirPro::LeftVertical); },
      [this] { return -m_controller.GetRawAxis(JoystickDefinitions::GameSirPro::RightVertical); }));
#endif

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
