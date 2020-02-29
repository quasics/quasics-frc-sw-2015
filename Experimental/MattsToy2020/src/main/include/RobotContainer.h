/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include "Constants.h"
#include "subsystems/DriveBase.h"
#include "subsystems/LightingSubsystem.h"
#include "subsystems/SwissArmySubsystem.h"

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
  void ConfigureSmartDashboard();

  // Robot subsystems and commands
 private:
  // Drive base stuff, allowing us to move around.
  DriveBase m_driveBase;

  frc2::InstantCommand m_enableTurbo{[this] { m_driveBase.EnableTurboMode(); },
                                     {}};
  frc2::InstantCommand m_disableTurbo{
      [this] { m_driveBase.DisableTurboMode(); }, {}};

  // The "Swiss Army Subsystem"
  SwissArmySubsystem swissArmySubsystem;

  // Lighting subsystem
  LightingSubsystem lightingSubsystem;

  // The driver's controller
  frc::Joystick m_logitechController{OIConstants::kDriverControllerPort};

  frc::XboxController m_xboxController{OIConstants::kOperatorControllerPort};

  frc::SendableChooser<frc2::Command*> autoChooser;
};
