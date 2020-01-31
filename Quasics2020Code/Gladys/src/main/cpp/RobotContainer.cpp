/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"
#include "commands/TankDrive.h"
#include "Constants.h"

inline double DeadBand (double stickValue) {
if (stickValue > -.1 && stickValue < .1) {
           return 0;
}
    return (stickValue);
}
RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here
drivebase.SetDefaultCommand(TankDrive(&drivebase, 
 [this] {

        double stickValue = driverJoystick.GetRawAxis(OIConstants::LogitechGamePad_RightYAxis);
        return DeadBand(stickValue);
 
 } 
      ,[this] {

        double stickValue = driverJoystick.GetRawAxis(OIConstants::LogitechGamePad_LeftYAxis);
        return DeadBand (stickValue);
        

      }));
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
