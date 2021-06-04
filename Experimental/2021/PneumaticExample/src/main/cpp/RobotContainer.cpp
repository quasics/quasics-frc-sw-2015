// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  ConfigurePneumaticDashboardCommands();
}

void RobotContainer::ConfigurePneumaticDashboardCommands() {
  frc::SmartDashboard::PutData(
      "Extend solenoid",
      new frc2::InstantCommand([this]() { m_pneumaticsBoard.ExtendSolenoid(); },
                               {&m_pneumaticsBoard}));
  frc::SmartDashboard::PutData(
      "Retract solenoid", new frc2::InstantCommand(
                              [this]() { m_pneumaticsBoard.RetractSolenoid(); },
                              {&m_pneumaticsBoard}));
  frc::SmartDashboard::PutData(
      "Stop solenoid",
      new frc2::InstantCommand([this]() { m_pneumaticsBoard.StopSolenoid(); },
                               {&m_pneumaticsBoard}));
  frc::SmartDashboard::PutData(
      "Enable compressor",
      new frc2::InstantCommand(
          [this]() { m_pneumaticsBoard.SetCompressorEnabled(true); },
          {&m_pneumaticsBoard}));
  frc::SmartDashboard::PutData(
      "Disable compressor",
      new frc2::InstantCommand(
          [this]() { m_pneumaticsBoard.SetCompressorEnabled(false); },
          {&m_pneumaticsBoard}));
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
