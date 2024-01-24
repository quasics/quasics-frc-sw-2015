// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <list>

#include "Constants.h"
#include "subsystems/IDrivebase.h"
#include "utils/DeadBandEnforcer.h"

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
  void AddTestButtonsOnSmartDashboard();
  void AddButtonToSmartDashboardTestingRetainedCommands();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc::Joystick m_driverController{OperatorConstants::kDriverControllerPort};

  void allocateDriveBase();

  void setUpTankDrive();

  void setUpArcadeDrive();

  double GetDriveSpeedScalingFactor();

  // The robot's subsystems are defined here...
  std::unique_ptr<IDrivebase> m_drivebase;

  // These are commands generated "on the fly" by functions like
  // GetCommandForTrajectory(), which we're attaching to things like buttons on
  // the smart dashboard, and thus need to outlive the function where they were
  // created.
  std::list<frc2::CommandPtr> retainedCommands;

  // void ConfigureBindings();

  const DeadBandEnforcer m_joystickDeadbandEnforcer{0.03};

  frc::SlewRateLimiter<units::scalar> m_leftSlewRateLimiter{
      DRIVER_JOYSTICK_RATE_LIMIT};
  frc::SlewRateLimiter<units::scalar> m_rightSlewRateLimiter{
      DRIVER_JOYSTICK_RATE_LIMIT};
};
