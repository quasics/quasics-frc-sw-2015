// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/WidgetType.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/Trigger.h>
#include <units/time.h>

#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/DriveFromVoltageForTime.h"
#include "TrajectoryGenerator.h"

RobotContainer::RobotContainer() : m_drive() {
  // Initialize all of your commands and subsystems here
  /*units::volt_t forwardVoltage = PathPlannerConstants::kS + (.3 * PathPlannerConstants::kV);
  std::shared_ptr<frc2::Command> pathPlannerCommand(new DriveFromVoltageForTime(&m_drive,forwardVoltage, forwardVoltage, 1_s));
pathplanner::NamedCommands::registerCommand("Forward", pathPlannerCommand);
*/
  // Configure the button bindings
  ConfigureBindings();
  AddSmartBoardTestButtons();
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return GetCommandForTrajectory("ContinuosPath.wpilib.json", &m_drive);
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
}

void RobotContainer::AddSmartBoardTestButtons() {
  /*frc::SmartDashboard::PutData(
    "Forward 2m",
  )*/
}

/*frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}*/
