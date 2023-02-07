// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/PrintCommand.h>
#include <iostream>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/Trigger.h>

#include "commands/SelfBalancing.h"
#include "commands/DriveUntilPitchAngleChange.h"
#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/TankDrive.h"
#include "commands/DriveAtPowerForMeters.h"
#include "commands/RotateAtAngle.h"
#include "Constants.h"

RobotContainer::RobotContainer() :
  m_leftSpeedLimiter{OperatorInterface::DRIVER_JOYSTICK_RATE_LIMIT},
  m_rightSpeedLimiter{OperatorInterface::DRIVER_JOYSTICK_RATE_LIMIT} {

  TankDrive tankDrive {
    &m_drivebase,
    [this] {

      // Figure out scaling for the current driving mode
      const double scalingFactor = GetDriveSpeedScalingFactor();

      double joystickValue;

      if(isInverted) {
        joystickValue = -1 * scalingFactor *
                              m_driverController.GetRawAxis(
            OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        
      }
     else {
      joystickValue = +1 * scalingFactor *
                            m_driverController.GetRawAxis(
            OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
     }
     return m_leftSpeedLimiter.Calculate(joystickValue);
    },
    [this] {

      const double scalingFactor = GetDriveSpeedScalingFactor();

      double joystickValue;

      if(isInverted) {
        joystickValue = -1 * scalingFactor *
                              m_driverController.GetRawAxis(
            OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
        
      }
      else {
        joystickValue = +1 * scalingFactor *
                            m_driverController.GetRawAxis(
        OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
      }
    return m_rightSpeedLimiter.Calculate(joystickValue);
    }
  };
  
  

  m_drivebase.SetDefaultCommand(tankDrive);

  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
  AddAutonomousCommandsToSmartDashboard();
  AddTestButtonsToSmartDashboard();  
}

double RobotContainer::GetDriveSpeedScalingFactor() {
  const bool isTurbo = m_driverStick.GetRawButton(
      OperatorInterface::LogitechGamePad::RIGHTSHOULDER);
  const bool isTurtle = m_driverStick.GetRawButton(
      OperatorInterface::LogitechGamePad::LEFTSHOULDER);

  if (isTurbo) {
    return RobotValues::TURBO_MODE_SPEED_SCALING;
  } else if (isTurtle) {
    return RobotValues::TURTLE_MODE_SPEED_SCALING;
  } else {
    return RobotValues::NORMAL_MODE_SPEED_SCALING;
  }
}


void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}

void RobotContainer::AddTestButtonsToSmartDashboard() {
  frc::SmartDashboard::PutData("Drive 1m at 70%", 
                        new DriveAtPowerForMeters(&m_drivebase, 0.70, 1_m));
  frc::SmartDashboard::PutData("Drive 1m at 20%",
                        new DriveAtPowerForMeters(&m_drivebase, 0.20, 1_m));

  frc::SmartDashboard::PutData("Rotate 90 degrees at 70%",
                        new RotateAtAngle(&m_drivebase, 0.70, 90_deg) );

  frc::SmartDashboard::PutData("Set Coasting Mode",
                       new frc2::InstantCommand([this]() { m_drivebase.SetBrakingMode(false); },
                               {&m_drivebase}));

    frc::SmartDashboard::PutData("Set Breaking Mode",
                       new frc2::InstantCommand([this]() { m_drivebase.SetBrakingMode(true); },
                               {&m_drivebase}));
}

void RobotContainer::AddAutonomousCommandsToSmartDashboard(){
  m_autonomousOptions.SetDefaultOption(
      "Do Nothing", new frc2::PrintCommand("I decline to do anything."));
  frc::SmartDashboard::PutData("Auto mode", &m_autonomousOptions);
}

frc2::SequentialCommandGroup* RobotContainer::RedAndBlueDriveStation2GTFO(){
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<DriveAtPowerForMeters>(&m_drivebase, -0.5, 4_m));
    commands.push_back(
      std::make_unique<DriveUntilPitchAngleChange>(&m_drivebase, 0.5));
  commands.push_back(
      std::make_unique<SelfBalancing>(&m_drivebase));
  return new frc2::SequentialCommandGroup(std::move(commands));
}