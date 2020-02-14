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
#include "commands/ClimberDown.h"
#include "commands/ClimberUp.h"
#include "commands/IntakeBallsCommand.h"
#include "commands/MoveForTime.h"
#include "commands/IntakeBallsReverseCommand.h"
#include "commands/TankDrive.h"
#include "commands/Turn4Times.h"
#include "commands/TurnToColor.h"
#include "commands/SpinTheWheel.h"
#include "subsystems/Drivebase.h"
#include "commands/ShoulderControl.h"
#include "subsystems/Intake.h"
#include "commands/ShootBallsCommand.h"
#include "commands/ShootBallsReverseCommand.h"
#include "commands/IntakeBallsCommand.h"
#include "commands/IntakeBallsReverseCommand.h"

inline double DeadBand(double stickValue) {
  if (stickValue > OIConstants::DeadBand_LowValue &&
      stickValue < OIConstants::DeadBand_HighValue) {
    return 0;
  }
  return (stickValue);
}

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
#ifndef DISABLE_DRIVE_BASE
  drivebase.SetDefaultCommand(TankDrive(
      &drivebase,
      [this] {
        double stickValue =
            driverJoystick.GetRawAxis(OIConstants::LogitechGamePad::RightYAxis);
        return DeadBand(stickValue);
      },
      [this] {
        double stickValue =
            driverJoystick.GetRawAxis(OIConstants::LogitechGamePad::LeftYAxis);
        return DeadBand(stickValue);
      }));
#endif  // DISABLE_DRIVE_BASE

  intake.SetDefaultCommand(ShoulderControl(
      &intake,
      [this] {
        double stickValue =
            operatorController.GetRawAxis(OIConstants::XBox::LeftYAxis);
        return DeadBand(stickValue);
            
      }
  ));

  // Configure the operator interface
  ConfigureButtonBindings();
  ConfigureSmartDashboard();
}

void RobotContainer::ConfigureButtonBindings() {
  std::cout << "Configuring button bindings for Driver controls" << std::endl;

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

  //frc2::JoystickButton(&operatorController,
                 //int(frc::XboxController::Button::kBumperLeft))
      //.WhileHeld(Turn4Times(&commandPanel));

  // frc2::JoystickButton(&m_xboxController,
  // int(frc::XboxController::Button::kA))
  //     .WhenPressed(frc2::PrintCommand("Button 'A' on XBox was pressed"));


//Exhaust
  frc2::JoystickButton(
      &operatorController,
      int(frc::XboxController::Button::kB)
    ).WhileHeld(ShootBallsCommand(&exhaust));
  // frc2::JoystickButton(
  //     &operatorController,
  //     int(frc::XboxController::Button::kB)
  //   ).WhileHeld(ShootBallsReverseCommand(&exhaust));
//

//Intake
  frc2::JoystickButton(
      &operatorController,
      int(frc::XboxController::Button::kX)
    ).WhileHeld(IntakeBallsCommand(&intake));
  // frc2::JoystickButton(
    //   &operatorController,
    //   int(frc::XboxController::Button::kBumperRight)
    // ).WhileHeld(IntakeBallsReverseCommand(&intake));
//

  frc2::JoystickButton(
      &operatorController,
      int(frc::XboxController::Button::kBack)
    ).WhileHeld(SpinTheWheel(&commandPanel, false));
  frc2::JoystickButton(
      &operatorController,
      int(frc::XboxController::Button::kStart)
    ).WhileHeld(SpinTheWheel(&commandPanel, true));
  frc2::JoystickButton(
      &operatorController,
      int(frc::XboxController::Button::kA)
    ).WhileHeld(ClimberUp(&climber));
  frc2::JoystickButton(
      &operatorController,
      int(frc::XboxController::Button::kY)
    ).WhileHeld(ClimberDown(&climber));
  

  //frc2::JoystickButton(&operatorController,
    //                   int(frc::XboxController::Button::kBumperRight))
      //.WhileHeld(TurnToColor(&commandPanel));

  // Going to map the following buttons:
  //    * left bumper --> push the ball up (TODO) fu=ind other binding
  //    * B button    --> run the exhaust to push out a ball
  //    * X button    --> intake balls
  //    * back button --> ????

  std::cout << "Done configuring button bindings" << std::endl;
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}

void RobotContainer::ConfigureSmartDashboard() {
#ifndef DISABLE_DRIVE_BASE
  frc::SmartDashboard::PutData("Move off the line",
                               new MoveForTime(&drivebase, 3, .4));
   std::cout << "Done configuring smart dashboard" << std::endl;
#endif  // DISABLE_DRIVE_BASE
}
