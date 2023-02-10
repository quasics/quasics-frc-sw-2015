// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/PrintCommand.h>
#include <iostream>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/ConditionalCommand.h>

#include "commands/SelfBalancing.h"
#include "commands/DriveUntilPitchAngleChange.h"
#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/TankDrive.h"
#include "commands/DriveAtPowerForMeters.h"
#include "commands/RotateAtAngle.h"
#include "Constants.h"

RobotContainer::RobotContainer() : m_leftSpeedLimiter{OperatorInterface::DRIVER_JOYSTICK_RATE_LIMIT},
                                   m_rightSpeedLimiter{OperatorInterface::DRIVER_JOYSTICK_RATE_LIMIT}
{
  TankDrive tankDrive{
      &m_drivebase,
      [this]
      {
        // Figure out scaling for the current driving mode
        const double scalingFactor = GetDriveSpeedScalingFactor();
        double joystickValue;

        if (isInverted)
        {
          joystickValue = -1 * scalingFactor *
                          m_driverController.GetRawAxis(
                              OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        }
        else
        {
          joystickValue = +1 * scalingFactor *
                          m_driverController.GetRawAxis(
                              OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        }
        return m_leftSpeedLimiter.Calculate(joystickValue);
      },
      [this]
      {
        const double scalingFactor = GetDriveSpeedScalingFactor();

        double joystickValue;

        if (isInverted)
        {
          joystickValue = -1 * scalingFactor *
                          m_driverController.GetRawAxis(
                              OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
        }
        else
        {
          joystickValue = +1 * scalingFactor *
                          m_driverController.GetRawAxis(
                              OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
        }
        return m_rightSpeedLimiter.Calculate(joystickValue);
      }};

  m_drivebase.SetDefaultCommand(tankDrive);

  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
  AddTestButtonsToSmartDashboard();
}

double RobotContainer::GetDriveSpeedScalingFactor()
{
  const bool isTurbo = m_driverStick.GetRawButton(
      OperatorInterface::LogitechGamePad::RIGHTSHOULDER);
  const bool isTurtle = m_driverStick.GetRawButton(
      OperatorInterface::LogitechGamePad::LEFTSHOULDER);

  if (isTurbo)
  {
    return RobotValues::TURBO_MODE_SPEED_SCALING;
  }
  else if (isTurtle)
  {
    return RobotValues::TURTLE_MODE_SPEED_SCALING;
  }
  else
  {
    return RobotValues::NORMAL_MODE_SPEED_SCALING;
  }
}

void RobotContainer::ConfigureBindings()
{
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this]
                { return m_subsystem.ExampleCondition(); })
      .OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}
/*







*/

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  // frc2::Command *
  frc2::Command *selectedOperation = m_RobotSequenceAutonomousOptions.GetSelected();
  frc2::Command *teamAndPosCmd = m_TeamAndStationAutonomousOptions.GetSelected();
  if (selectedOperation == nullptr || teamAndPosCmd == nullptr)
  {
    // This shouldn't happen if things were set up right.  But it did.  So they weren't.
    // We'll bail out, but at least return a valid pointer that will tell us something went wrong when it's run.
    static frc2::PrintCommand somethingIsScrewyCommand("Can't decide what to do");
    return &somethingIsScrewyCommand;
  }

  std::string operationName = selectedOperation->GetName();
  std::string teamAndPosName = teamAndPosCmd->GetName();

  if (operationName == AutonomousSelectedOperation::DoNothing)
  {
    return new frc2::PrintCommand("Doing nothing, as instructed");
  }
  else if (operationName == AutonomousSelectedOperation::GTFO)
  {
    if (teamAndPosName == AutonmousTeamAndStationPositions::Blue2 || teamAndPosName == AutonmousTeamAndStationPositions::Red2)
    {
      std::vector<std::unique_ptr<frc2::Command>> commands;
      commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{&m_drivebase, -0.5, 4.5_m}));
      return new frc2::SequentialCommandGroup(std::move(commands));
    }
    else{
      std::vector<std::unique_ptr<frc2::Command>> commands;
      commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{&m_drivebase, -0.5, 4.0_m}));
      return new frc2::SequentialCommandGroup(std::move(commands));
    }
  }
  else if (operationName == AutonomousSelectedOperation::GTFODock)
  {
    if (teamAndPosName == AutonmousTeamAndStationPositions::Blue2 || teamAndPosName == AutonmousTeamAndStationPositions::Red2)
    {
      std::vector<std::unique_ptr<frc2::Command>> commands;
      commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{&m_drivebase, -0.5, 4.5_m}));
      commands.push_back(std::unique_ptr<frc2::Command>(new DriveUntilPitchAngleChange{&m_drivebase, 0.5})); // LOOK INTO HOW TO DO OR
      commands.push_back(std::unique_ptr<frc2::Command>(new SelfBalancing{&m_drivebase}));
      return new frc2::SequentialCommandGroup(std::move(commands));
    }
    else
    {
      const bool firstTurnIsClockwise = (teamAndPosName == AutonmousTeamAndStationPositions::Blue3 || teamAndPosName == AutonmousTeamAndStationPositions::Red1);
      // Build a sequential command
      std::vector<std::unique_ptr<frc2::Command>> commands; 

      commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{&m_drivebase, -0.5, 4.0_m}));
      commands.push_back(std::unique_ptr<frc2::Command>(
          new frc2::ConditionalCommand(RotateAtAngle{&m_drivebase, 0.5, 90_deg}, RotateAtAngle{&m_drivebase, 0.5, -90_deg}, [firstTurnIsClockwise]()
                                       { return firstTurnIsClockwise; })));
      commands.push_back(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters{&m_drivebase, 0.5, 1.889_m}));
      commands.push_back(std::unique_ptr<frc2::Command>(
          new frc2::ConditionalCommand(RotateAtAngle{&m_drivebase, 0.5, 90_deg}, RotateAtAngle{&m_drivebase, 0.5, -90_deg}, [firstTurnIsClockwise]()
                                       { return firstTurnIsClockwise; })));
      commands.push_back(std::unique_ptr<frc2::Command>(new DriveUntilPitchAngleChange{&m_drivebase, 0.5})); // LOOK INTO HOW TO DO OR
      commands.push_back(std::unique_ptr<frc2::Command>(new SelfBalancing{&m_drivebase}));
      return new frc2::SequentialCommandGroup(std::move(commands));

      // Add commands to move forward until we hit the ramp (or decide we're not going to), and to then balance

      // Build and return the sequence to run.
      return new frc2::PrintCommand("Invalid Input, Failed, Doing Nothing"); 
    }
  }

  /*
  TAKE A LOOK AT CONDITIONAL COMMANDS MIGHT BE AN ALT WAY TO DO THIS
  OR JUST DO THESE BASIC DECISIONS IF
  DONE THIS WAY THEN IT SHOULD GO COMMAND THEN STATION AND COLOR

  */

  return m_RobotSequenceAutonomousOptions.GetSelected(); //CHANGE THIS
}

/*







*/

void RobotContainer::AddTestButtonsToSmartDashboard()
{
  frc::SmartDashboard::PutData("Drive 1m at 70%",
                               new DriveAtPowerForMeters(&m_drivebase, 0.70, 1_m));
  frc::SmartDashboard::PutData("Drive 1m at 20%",
                               new DriveAtPowerForMeters(&m_drivebase, 0.20, 1_m));

  frc::SmartDashboard::PutData("Rotate 90 degrees at 70%",
                               new RotateAtAngle(&m_drivebase, 0.70, 90_deg));

  frc::SmartDashboard::PutData("Set Coasting Mode",
                               new frc2::InstantCommand([this]()
                                                        { m_drivebase.SetBrakingMode(false); },
                                                        {&m_drivebase}));

  frc::SmartDashboard::PutData("Set Breaking Mode",
                               new frc2::InstantCommand([this]()
                                                        { m_drivebase.SetBrakingMode(true); },
                                                        {&m_drivebase}));
}
/*





*/

void RobotContainer::AddTeamAndStationSelectorToSmartDashboard()
{
  frc2::Command *blue1 = new frc2::PrintCommand("Blue 1");
  blue1->SetName(AutonmousTeamAndStationPositions::Blue1);
  m_TeamAndStationAutonomousOptions.SetDefaultOption(AutonmousTeamAndStationPositions::Blue1, blue1);

  frc2::Command *blue2 = new frc2::PrintCommand("Blue 2");
  blue2->SetName(AutonmousTeamAndStationPositions::Blue2);
  m_TeamAndStationAutonomousOptions.AddOption(AutonmousTeamAndStationPositions::Blue2, blue2);

  frc2::Command *blue3 = new frc2::PrintCommand("Blue 3");
  blue3->SetName(AutonmousTeamAndStationPositions::Blue3);
  m_TeamAndStationAutonomousOptions.AddOption(AutonmousTeamAndStationPositions::Blue3, blue3);

  frc2::Command *red1 = new frc2::PrintCommand("Red 1");
  red1->SetName(AutonmousTeamAndStationPositions::Red1);
  m_TeamAndStationAutonomousOptions.AddOption(AutonmousTeamAndStationPositions::Red1, red1);

  frc2::Command *red2 = new frc2::PrintCommand("Red 2");
  red2->SetName(AutonmousTeamAndStationPositions::Red2);
  m_TeamAndStationAutonomousOptions.AddOption(AutonmousTeamAndStationPositions::Red2, red2);

  frc2::Command *red3 = new frc2::PrintCommand("Red 3");
  red3->SetName(AutonmousTeamAndStationPositions::Red3);
  m_TeamAndStationAutonomousOptions.AddOption(AutonmousTeamAndStationPositions::Red3, red3);

  frc::SmartDashboard::PutData("Team and Station Auto Selector", &m_TeamAndStationAutonomousOptions);
}

void RobotContainer::AddRobotSequenceSelectorToSmartDashboard()
{
  frc2::Command *doNothingCommand = new frc2::PrintCommand("Do Nothing");
  doNothingCommand->SetName(AutonomousSelectedOperation::DoNothing);
  m_RobotSequenceAutonomousOptions.SetDefaultOption(AutonomousSelectedOperation::DoNothing, doNothingCommand);

  frc2::Command *GTFODockCommand = new frc2::PrintCommand("GTFO and Dock");
  GTFODockCommand->SetName(AutonomousSelectedOperation::GTFODock);
  m_RobotSequenceAutonomousOptions.AddOption(AutonomousSelectedOperation::GTFODock, GTFODockCommand);

  frc::SmartDashboard::PutData("Robot Sequence Auto Selector", &m_RobotSequenceAutonomousOptions);
}
/*







*/
frc2::SequentialCommandGroup *RobotContainer::RedAndBlueDriveStation2GTFOAndBalance()
{
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<DriveAtPowerForMeters>(&m_drivebase, -0.5, 4_m));
  commands.push_back(
      std::make_unique<DriveUntilPitchAngleChange>(&m_drivebase, 0.5));
  commands.push_back(
      std::make_unique<SelfBalancing>(&m_drivebase));
  return new frc2::SequentialCommandGroup(std::move(commands));
}