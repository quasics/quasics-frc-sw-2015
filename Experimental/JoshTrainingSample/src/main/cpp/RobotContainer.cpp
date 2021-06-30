// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/Button.h>

#include "commands/TeleopArcadeDrive.h"

RobotContainer::RobotContainer() {
  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Also set default commands here
  m_drive.SetDefaultCommand(TeleopArcadeDrive(
      &m_drive, [this] { return -m_controller.GetRawAxis(1); },
      [this] { return m_controller.GetRawAxis(2); }));

  // Example of how to use the onboard IO
  m_onboardButtonA.WhenPressed(frc2::PrintCommand("Button A Pressed"))
      .WhenReleased(frc2::PrintCommand("Button A Released"));

  // Setup SmartDashboard options.
  m_chooser.SetDefaultOption("Drive a cicle" , &m_driveCircle);
  m_chooser.AddOption("Drive a small circle" , &m_driveSmallCircle);
  m_chooser.AddOption("Auto Routine Distance", &m_autoDistance);
  m_chooser.AddOption("Auto Routine Time", &m_autoTime);
  m_chooser.AddOption("Forward 1m, 50% speed", &m_forward1Meter);
  frc::SmartDashboard::PutData("Auto Selector", &m_chooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_chooser.GetSelected();
}

// Sample usage:
//    frc2::Command* group = BuildSequentialCommandGroup(new CommandOne(.....), new CommandTwo(.....));
frc2::SequentialCommandGroup* RobotContainer::BuildSequentialCommandGroup(
  frc2::Command* commandOne, frc2::Command* commandTwo
) {
  std::vector<std::unique_ptr<frc2::Command>> commandPieces;

  commandPieces.push_back(std::move(std::unique_ptr<frc2::Command>(commandOne)));
  commandPieces.push_back(std::move(std::unique_ptr<frc2::Command>(commandTwo)));

  return new frc2::SequentialCommandGroup(std::move(commandPieces));
}
