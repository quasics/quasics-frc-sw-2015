// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc/Joystick.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/DriveBase.h"

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

  // "Helper functions".  
 private:
  // Sample function, showing how you can build a sequence of simple commands that
  // will perform some more complex operation.
  frc2::SequentialCommandGroup* BuildSampleCommandSequence();

  void ConfigureBindings();

  // Private data for the class.
 private:
  // Things the drive team uses to control the robot.
  frc2::CommandJoystick m_driverController{
      OperatorConstants::kDriverControllerPort};
  frc2::CommandXboxController m_operatorController{
      OperatorConstants::kOperatorControllerPort};

  // The robot's subsystems.
  ExampleSubsystem m_subsystem;
  DriveBase m_driveBase;
};
