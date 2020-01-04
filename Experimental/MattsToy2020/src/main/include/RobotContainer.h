/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include "Constants.h"
#include "commands/ExampleCommand.h"
#include "subsystems/DriveBase.h"

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

 private:
  void ConfigureButtonBindings();

  // The robot's subsystems and commands are defined here...
  DriveBase m_driveBase;

  frc2::InstantCommand m_enableTurbo{[this] { m_driveBase.EnableTurboMode(); },
                                     {}};
  frc2::InstantCommand m_disableTurbo{
      [this] { m_driveBase.DisableTurboMode(); }, {}};

  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

  ExampleCommand m_autonomousCommand;
};
