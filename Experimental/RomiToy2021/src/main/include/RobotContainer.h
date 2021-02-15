// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/Command.h>

#include "../../../../Common2021/TurnToTargetCommand.h"
#include "../../../../Common2021/VisionSettingsHelper.h"
#include "commands/ExampleCommand.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/OnBoardIO.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
public:
  RobotContainer();

  frc2::Command *GetAutonomousCommand();

private:
  void ConfigureButtonBindings();
  void ConfigureDrivingCommand();

private:
  // The robot's subsystems and commands are defined here...
  ExampleSubsystem m_subsystem;
  ExampleCommand m_autonomousCommand;

  VisionSettingsHelper m_helper{"visionSettings.dat"};

  // Assumes a gamepad plugged into channnel 0
  frc::Joystick m_controller{0};

  // The robot's subsystems
  Drivetrain m_drive;
  OnBoardIO m_onboardIO{OnBoardIO::ChannelMode::INPUT,
                        OnBoardIO::ChannelMode::INPUT};
  TurnToTargetCommand m_turnToTargetCommand{&m_drive, 0.350};
};
