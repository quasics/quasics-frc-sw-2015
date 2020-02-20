/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include <iostream>

#include "Constants.h"
#include "commands/ClimberDownCommand.h"
#include "commands/ClimberUpCommand.h"
#include "commands/DeliverToLowGoalCommand.h"
#include "commands/IntakeBallsCommand.h"
#include "commands/IntakeBallsReverseCommand.h"
#include "commands/LoadFromWindowCommand.h"
#include "commands/MoveForTimeCommand.h"
#include "commands/ShootBallsCommand.h"
#include "commands/ShootBallsReverseCommand.h"
#include "commands/ShoulderControlCommand.h"
#include "commands/SpinTheWheelCommand.h"
#include "commands/TankDriveCommand.h"
#include "commands/TurnControlPanel4TimesCommand.h"
#include "commands/TurnControlPanelToTargetColorCommand.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Intake.h"

inline double DeadBand(double stickValue) {
  if (stickValue > OIConstants::DeadBand_LowValue &&
      stickValue < OIConstants::DeadBand_HighValue) {
    return 0;
  }
  return (stickValue);
}

RobotContainer::RobotContainer()
    : drivebase(
#ifdef DISABLE_DRIVE_BASE
          nullptr
#else
          new Drivebase
#endif  // DISABLE_DRIVE_BASE
      ) {
  // Initialize all of your commands and subsystems here
  if (drivebase) {
    drivebase->SetDefaultCommand(TankDriveCommand(
        drivebase.get(),
        [this] {
          double stickValue = driverJoystick.GetRawAxis(
              OIConstants::LogitechGamePad::RightYAxis);
          return DeadBand(stickValue);
        },
        [this] {
          double stickValue = driverJoystick.GetRawAxis(
              OIConstants::LogitechGamePad::LeftYAxis);
          return DeadBand(stickValue);
        }));
  }

  intake.SetDefaultCommand(ShoulderControlCommand(&intake, [this] {
    double stickValue =
        operatorController.GetRawAxis(OIConstants::XBox::LeftYAxis);
    return DeadBand(stickValue);
  }));

  // Configure the operator interface
  ConfigureButtonBindings();
  ConfigureSmartDashboard();
}

void RobotContainer::ConfigureButtonBindings() {
  std::cout << "Beginning button binding configuration" << std::endl;

#ifndef DISABLE_DRIVE_BASE
  frc2::JoystickButton(&driverJoystick,
                       OIConstants::LogitechGamePad::LeftShoulder)
      .WhenPressed(enableTurboMode)
      .WhenReleased(disableTurboMode);

  frc2::JoystickButton(&driverJoystick,
                       OIConstants::LogitechGamePad::RightShoulder)
      .WhenPressed(frontIsForward);
#endif  // DISABLE_DRIVE_BASE

  // frc2::JoystickButton(&driverJoystick,
  //                      OIConstants::LogitechGamePad::LeftShoulder)
  //     .WhileHeld(IntakeBallsCommand(&intake));

  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kBumperLeft))
      .WhenPressed(TurnControlPanel4TimesCommand(&commandPanel));

  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kA))
      .WhenPressed(frc2::PrintCommand("Button 'A' on XBox was pressed"));

  // Exhaust
  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kY))
      .WhileHeld(LoadFromWindowCommand(&exhaust));
  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kB))
      .WhileHeld(DeliverToLowGoalCommand(&exhaust));
  //

  // Intake
  // frc2::JoystickButton(
  //&operatorController,
  // int(frc::XboxController::Button::kA)
  //).WhileHeld(IntakeBallsCommand(&intake));
  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kX))
      .WhileHeld(IntakeBallsReverseCommand(&intake));
  //

  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kStart))
      .WhileHeld(SpinTheWheelCommand(&commandPanel, true));
  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kBumperLeft))
      .WhileHeld(ClimberUpCommand(&climber));
  frc2::JoystickButton(&operatorController,
                       int(frc::XboxController::Button::kBumperRight))
      .WhileHeld(ClimberDownCommand(&climber));

  // frc2::JoystickButton(&operatorController,
  // int(frc::XboxController::Button::kBumperRight))
  //.WhileHeld(TurnControlPanelToTargetColorCommand(&commandPanel));

  // Going to map the following buttons:
  //    * left bumper --> push the ball up (TODO) fu=ind other binding
  //    * B button    --> run the exhaust to push out a ball
  //    * X button    --> intake balls
  //    * back button --> ????

  std::cout << "Completed button binding configuration" << std::endl;
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
#ifndef DISABLE_DRIVE_BASE
  return &m_trivialAutonmousCommand;
#else
  return &m_autonomousCommand;
#endif  // DISABLE_DRIVE_BASE
}

void RobotContainer::ConfigureSmartDashboard() {
  std::cout << "Beginning smart dashboard configuration" << std::endl;
  if (drivebase) {
    frc::SmartDashboard::PutData(
        "Move off the line", new MoveForTimeCommand(drivebase.get(), 3, .4));
  }
  frc::SmartDashboard::PutData(
      "Turn Four Times", new TurnControlPanel4TimesCommand(&commandPanel));
  std::cout << "Completed smart dashboard configuration" << std::endl;

  frc::SmartDashboard::PutData(
      "Turn To Color", new TurnControlPanelToTargetColorCommand(&commandPanel));
}
