/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include "Constants.h"
#include "commands/ExampleCommand.h"
#include "commands/FieldSensitiveLightingCommand.h"
#include "commands/LowerElevatorCommand.h"
#include "commands/MoveForTimeCommand.h"
#include "commands/RaiseElevatorCommand.h"
#include "commands/TankDriveCommand.h"
#include "utils/DeadBandLimiter.h"

const DeadBandLimiter<double> JoystickDeadbandLimiter(
    OIConstants::kDriveControllerDeadBandSize);

// Initialize all of your commands and subsystems here.
RobotContainer::RobotContainer() {
  m_driveBase.SetDefaultCommand(TankDriveCommand(
      &m_driveBase,
      [this] {
        double stickValue = m_logitechController.GetRawAxis(
            OIConstants::LogitechGamePad::LeftYAxis);
        return JoystickDeadbandLimiter(stickValue);
      },
      [this] {
        double stickValue = m_logitechController.GetRawAxis(
            OIConstants::LogitechGamePad::RightYAxis);
        return JoystickDeadbandLimiter(stickValue);
      }));

  lightingSubsystem.SetDefaultCommand(
      FieldSensitiveLightingCommand(&lightingSubsystem));

  // Configure the button bindings
  ConfigureButtonBindings();

  // Configure the smart dashboard/shuffleboard
  ConfigureSmartDashboard();
}

void RobotContainer::ConfigureSmartDashboard() {
  // Configure the set of commands we want to choose from in autonomous mode.
  autoChooser.SetDefaultOption("Do nothing", new DoNothingCommand);
  autoChooser.AddOption("Do something", new ExampleCommand("Do something"));
  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

  frc::SmartDashboard::PutData("Forward 1s/25%",
                               new MoveForTimeCommand(&m_driveBase, 1, 0.25));
}

void RobotContainer::ConfigureButtonBindings() {
  // Trivial example, just for demo purposes.
  frc2::JoystickButton(&m_xboxController, int(frc::XboxController::Button::kA))
      .WhenPressed(frc2::PrintCommand("Button 'A' on XBox was pressed"));

  // While the left shoulder button is held down, enable turbo mode.
  //
  // (Another option would be to have a non-instant command that would enable
  // turbo in Initialize(), disable it in End(), and then bind it "WhileHeld",
  // but this lets me show a couple of InstantCommands being used.)
  frc2::JoystickButton(&m_logitechController,
                       OIConstants::LogitechGamePad::LeftShoulder)
      .WhenPressed(&m_enableTurbo)
      .WhenReleased(&m_disableTurbo);

  // While holding the right shoulder button, move the elevator up.
  frc2::JoystickButton(&m_logitechController,
                       OIConstants::LogitechGamePad::YButton)
      .WhileHeld(RaiseElevatorCommand(&swissArmySubsystem));

  // While holding the left shoulder button, move the elevator down.
  frc2::JoystickButton(&m_logitechController,
                       OIConstants::LogitechGamePad::AButton)
      .WhileHeld(LowerElevatorCommand(&swissArmySubsystem));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autoChooser.GetSelected();
}
