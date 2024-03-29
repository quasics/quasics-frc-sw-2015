// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/Button.h>

#include "commands/TeleopArcadeDrive.h"
#include "commands/DriveDistance.h"
#include "commands/TurnDegrees.h"
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
  m_chooser.SetDefaultOption("Draw Square", BuildSquareCommand());
  m_chooser.AddOption("Auto Routine Distance", &m_autoDistance);
  m_chooser.AddOption("Auto Routine Time", &m_autoTime);
  frc::SmartDashboard::PutData("Auto Selector", &m_chooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_chooser.GetSelected();
}

frc2::SequentialCommandGroup* RobotContainer::BuildSquareCommand(){
  std::vector<std::unique_ptr<frc2::Command>> squarePieces;

  for(int i = 0; i < 4; i++){
    frc2::Command * piece1 = new DriveDistance(0.6, 10_in, &m_drive);
    frc2::Command * piece2 = new TurnDegrees(0.5,75_deg, &m_drive);
    squarePieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      piece1)));
    squarePieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      piece2)));
  }
  return new frc2::SequentialCommandGroup(std::move(squarePieces));
}
