// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <iostream>

#include "TrajectoryGenerator.h"
#include "commands/DriveFromVoltageForTime.h"
#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand(frc2::InstantCommand([this]{m_drive.TankDriveVolts(0_V, 0_V);}, {&m_drive}) );

  // Configure the button bindings
  ConfigureBindings();
  AddTestButtonsOnSmartDashboard();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return GetCommandForTrajectory("LoopyPath.wpilib.json", &m_drive);
}

void RobotContainer::AddTestButtonsOnSmartDashboard(){
  frc::SmartDashboard::PutData("3volts left, -3Volts right, 3 sec", new DriveFromVoltageForTime(&m_drive, 3_V, -3_V, 3_s));
  frc::SmartDashboard::PutData("3volts both, 3 sec", new DriveFromVoltageForTime(&m_drive, 3_V, 3_V, 3_s));
  frc::SmartDashboard::PutData("Reset Encoders", new frc2::InstantCommand([this]() { m_drive.ResetEncoders(); }, {&m_drive}));
}
