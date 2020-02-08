/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>

#include "Constants.h"
#include "commands/ClimberDown.h"
#include "commands/ClimberUp.h"
#include "commands/IntakeBallsCommand.h"
#include "commands/PushBallUpCommand.h"
#include "commands/TankDrive.h"
#include "commands/Turn4Times.h"
#include "commands/TurnToColor.h"
#include "subsystems/Drivebase.h"
#include <frc2/command/button/Trigger.h>

inline double DeadBand(double stickValue) {
  if (stickValue > OIConstants::DeadBand_LowValue &&
      stickValue < OIConstants::DeadBand_HighValue) {
    return 0;
  }
  return (stickValue);
}
RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here
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

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  // frc2::JoystickButton(&driverJoystick,
  //                      OIConstants::LogitechGamePad::LeftShoulder)
  //     .WhileHeld(IntakeBallsCommand(&intake));

  frc2::JoystickButton(&driverJoystick,
                       OIConstants::LogitechGamePad::LeftShoulder)
      .WhenPressed(enableTurboMode)
      .WhenReleased(disableTurboMode);

  frc2::JoystickButton(&driverJoystick,
                       OIConstants::LogitechGamePad::RightShoulder)
      .WhenPressed(frontIsForward);
  frc2::JoystickButton(&operatorController, OIConstants::XBox::LeftButton)
  .WhileHeld(Turn4Times(&commandPanel));

   frc2::JoystickButton(&operatorController, OIConstants::XBox::LeftTrigger)
  .WhileHeld(TurnToColor(&commandPanel));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
