// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include "Constants.h"
#include "commands/BreathingAllianceLights.h"
#include "commands/BreathingLights.h"
#include "commands/DriveAtPowerForMeters.h"
#include "commands/ExtendClimber.h"
#include "commands/MoveRobotTestCommand.h"
#include "commands/RetractClimber.h"
#include "commands/RunConveyorAtSpeed.h"
#include "commands/RunIntakeAtSpeed.h"
#include "commands/RunShooterAtSpeed.h"
#include "commands/SetLightsToColor.h"
#include "commands/ShootForTime.h"
#include "commands/TankDrive.h"

RobotContainer::RobotContainer() {
  frc2::InstantCommand switchControls([this] { isSwitched = !(isSwitched); });

  frc2::JoystickButton(&m_driverStick,
                       OperatorInterface::LogitechGamePad::YButton)
      .WhenPressed(&switchControls);

  TankDrive tankDrive{
      &m_drivebase,
      [this] {
        if (!isSwitched) {
          bool isTurbo = m_driverStick.GetRawButton(
              OperatorInterface::LogitechGamePad::LEFTSHOULDER);
          bool isTurtle = m_driverStick.GetRawButton(
              OperatorInterface::LogitechGamePad::RIGHTSHOULDER);
          if (isTurbo) {
            return 0.80 * m_driverStick.GetRawAxis(
                              OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
          } else if (isTurtle) {
            return 0.40 * m_driverStick.GetRawAxis(
                              OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
          } else {
            return 0.60 * m_driverStick.GetRawAxis(
                              OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
          }
        } else {
          bool isTurbo = m_driverStick.GetRawButton(
              OperatorInterface::LogitechGamePad::LEFTSHOULDER);
          bool isTurtle = m_driverStick.GetRawButton(
              OperatorInterface::LogitechGamePad::RIGHTSHOULDER);
          if (isTurbo) {
            return -1 * 0.80 *
                   m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
          } else if (isTurtle) {
            return -1 * 0.40 *
                   m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
          } else {
            return -1 * 0.60 *
                   m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
          }
        }
      },
      [this] {
        if (!isSwitched) {
          bool isTurbo = m_driverStick.GetRawButton(
              OperatorInterface::LogitechGamePad::LEFTSHOULDER);
          bool isTurtle = m_driverStick.GetRawButton(
              OperatorInterface::LogitechGamePad::RIGHTSHOULDER);
          if (isTurbo) {
            return 0.80 * m_driverStick.GetRawAxis(
                              OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
          } else if (isTurtle) {
            return 0.40 * m_driverStick.GetRawAxis(
                              OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
          } else {
            return 0.60 * m_driverStick.GetRawAxis(
                              OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
          }
        } else {
          bool isTurbo = m_driverStick.GetRawButton(
              OperatorInterface::LogitechGamePad::LEFTSHOULDER);
          bool isTurtle = m_driverStick.GetRawButton(
              OperatorInterface::LogitechGamePad::RIGHTSHOULDER);
          if (isTurbo) {
            return -1 * 0.80 *
                   m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
          } else if (isTurtle) {
            return -1 * 0.40 *
                   m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
          } else {
            return -1 * 0.60 *
                   m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
          }
        }
      }};

  // Initialize all of your commands and subsystems here
  m_drivebase.SetDefaultCommand(tankDrive);

  // Configure the button bindings
  ConfigureJoystickButtonBindings();
  AddTestButtonsToSmartDashboard();
  AddAutonomousCommandsToSmartDashboard();
}

void RobotContainer::ConfigureJoystickButtonBindings() {
  // Configure your button bindings here
}

// Note: 0.65 seems to be reasonable power for the high goal.
void RobotContainer::AddTestButtonsToSmartDashboard() {
  frc::SmartDashboard::PutData("Test Button Do Something",
                               new MoveRobotTestCommand(&m_drivebase, 0.2));
  frc::SmartDashboard::PutData("Shoot @ 65%",
                               new RunShooterAtSpeed(&m_shooter, 0.65));
  frc::SmartDashboard::PutData("Intake: 50% forward",
                               new RunIntakeAtSpeed(&m_intake, 0.50));
  frc::SmartDashboard::PutData("Intake: 30% backward",
                               new RunIntakeAtSpeed(&m_intake, -0.3));
  frc::SmartDashboard::PutData("Conveyor: 20% forward",
                               new RunConveyorAtSpeed(&m_conveyor, 0.2));
  frc::SmartDashboard::PutData("Conveyor: 30% backward",
                               new RunConveyorAtSpeed(&m_conveyor, -0.3));
  frc::SmartDashboard::PutData("Extend Climber", new ExtendClimber(&m_climber));
  frc::SmartDashboard::PutData("Retract Climber",
                               new RetractClimber(&m_climber));
  frc::SmartDashboard::PutData(
      "Set All ligths to Red",
      new SetLightsToColor(&m_lighting, Lighting::StockColor::Red));
  frc::SmartDashboard::PutData(
      "Breathing Lights", new BreathingLights(&m_lighting, 0, 255, 0, 0.75));
  frc::SmartDashboard::PutData("Alliance Breathing Lights",
                               new BreathingAllianceLights(&m_lighting, 0.75));
}

void RobotContainer::AddAutonomousCommandsToSmartDashboard() {
  m_autonomousOptions.SetDefaultOption(
      "Do Nothing", new frc2::PrintCommand("I decline to do anything."));

  m_autonomousOptions.AddOption(
      "Move Forward 1m at 50% power",
      new DriveAtPowerForMeters(&m_drivebase, 0.5, 1_m));

  m_autonomousOptions.AddOption("Shoot @ 20% for 3 seconds",
                                new ShootForTime(&m_shooter, 0.2, 3_s));

  m_autonomousOptions.AddOption("Shoot @ 20% for 2sec, move @ 20% for 1",
                                BuildShootAndMoveCommand(0.2, 2_s, 0.2, 1_m));

  frc::SmartDashboard::PutData("Auto mode", &m_autonomousOptions);
}

frc2::SequentialCommandGroup* RobotContainer::BuildShootAndMoveCommand(
    double powerShoot, units::second_t timeShoot, double powerMove,
    units::meter_t distanceToMove) {
  // Holds the sequence of the commands to be executed as a group.
  std::vector<std::unique_ptr<frc2::Command>> commands;

  // Add each of the individual commands to the sequence.
  commands.push_back(std::make_unique<frc2::PrintCommand>(
      "Starting 'shoot and move' sequence"));
  commands.push_back(
      std::make_unique<ShootForTime>(&m_shooter, powerShoot, timeShoot));
  commands.push_back(std::make_unique<frc2::PrintCommand>("Moving away"));
  commands.push_back(std::make_unique<DriveAtPowerForMeters>(
      &m_drivebase, powerMove, distanceToMove));
  commands.push_back(
      std::make_unique<frc2::PrintCommand>("Sequence completed"));

  // Builds the command group object.
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autonomousOptions.GetSelected();
}
