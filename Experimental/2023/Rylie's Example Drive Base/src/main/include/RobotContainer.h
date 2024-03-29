// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/Joystick.h>



#include "Constants.h"
#include "subsystems/DriveBase.h"
#include "subsystems/ExampleSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void SetDefaultTankDrive();
  void SetDefaultArcadeDrive();
 void ConfigureBindings();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // Driver's controller.
  frc::Joystick m_driverStick{OperatorInterface::DRIVER_JOYSTICK};

  
  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  DriveBase m_drivebase;

  // Supporting tank drive-related stuff.
  frc::SlewRateLimiter<units::scalar> m_leftSlewRateLimiter;
  frc::SlewRateLimiter<units::scalar> m_rightSlewRateLimiter;
 
};
