// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/PrintCommand.h>

#include "commands/Autos.h"
#include "commands/DriveDistance.h"
#include "commands/TankDrive.h"
#include "commands/TurnToAngle.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();

  frc::SmartDashboard::PutData("Sample cmd", new frc2::PrintCommand("Do something!"));
  frc::SmartDashboard::PutData("1m @ 30%", new DriveDistance(&m_driveBase, 1_m, 0.3));
  frc::SmartDashboard::PutData("+45 degree turn", new TurnToAngle(&m_driveBase, 45_deg, 0.3));
  frc::SmartDashboard::PutData("-90 degree turn", new TurnToAngle(&m_driveBase, -90_deg, 0.3));
}

void RobotContainer::ConfigureBindings()
{
  using namespace OperatorConstants::LogitechGamePad;

  //
  // Configure your trigger bindings here.

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this]
                { return m_subsystem.ExampleCondition(); })
      .OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_operatorController.B().WhileTrue(m_subsystem.ExampleMethodCommand());

  //
  // Configure default commands for subsystems.

  // Default for drive base is (of course) tank driving.
  TankDrive tankDrive(
      &m_driveBase, // The drive base it works with
      // provides left speed control
      [this]()
      { return m_driverController.GetRawAxis(LEFT_Y_AXIS); },
      // provides right speed control
      [this]()
      { return m_driverController.GetRawAxis(RIGHT_Y_AXIS); });
  m_driveBase.SetDefaultCommand(tankDrive);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
