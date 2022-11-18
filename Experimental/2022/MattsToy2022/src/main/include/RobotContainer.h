// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/Command.h>
#include <frc2/command/PrintCommand.h>
#include <units/dimensionless.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/DriveBase.h"
#include "subsystems/Lighting.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  frc2::SequentialCommandGroup* Eight();


 private:
  void ConfigureButtonBindings();



 private:
  // Slew rate limiters to make joystick inputs more gentle.  (See example at
  // https://tinyurl.com/52k4cpsn.)
  frc::SlewRateLimiter<units::scalar> m_leftSpeedLimiter;
  frc::SlewRateLimiter<units::scalar> m_rightSpeedLimiter;

  frc::Joystick m_driverStick{OperatorInterface::DRIVER_JOYSTICK};



  DriveBase m_driveBase;
  Lighting m_lighting;

  frc2::PrintCommand m_autonomousCommand{"Do something autonomous...."};
};
