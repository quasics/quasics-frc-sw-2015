// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/Drivebase.h"
#include "commands/DoASpin.h"
#include "commands/RunShootingMotor.h"
#include "Constants.h"
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include "commands/TankDrive.h"

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  ConfigureSmartDashboard();
    drivebase.SetDefaultCommand(TankDrive(
      &drivebase,
      [this] {
        double stickValue =
            driverJoystick.GetRawAxis(OIConstants::LogitechGamePad::RightYAxis);
        return stickValue;
      },
      [this] {
        double stickValue =
            driverJoystick.GetRawAxis(OIConstants::LogitechGamePad::LeftYAxis);
        return stickValue;
      }));
}

void RobotContainer::RunCommandWhenOperatorButtonIsHeld(
    frc::XboxController::Button buttonId, frc2::Command* command) {
  frc2::JoystickButton(&operatorController, int(buttonId)).WhileHeld(command); //see last year's code
}

void RobotContainer::ConfigureButtonBindings() {
  static RunShootingMotor runshootingmotor(&shooter);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kA, &runshootingmotor); //see last year's code

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}

void RobotContainer::ConfigureSmartDashboard() {
  frc::SmartDashboard::PutData("Do those spinnin",
                               new DoASpin(&drivebase));
  frc::SmartDashboard::PutData("Run shooter at 75% power", new RunShootingMotor(&shooter));
}