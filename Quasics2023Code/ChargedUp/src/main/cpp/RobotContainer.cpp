// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/Trigger.h>

#include <iostream>
#include <list>

#include "Constants.h"
#include "commands/Autos.h"
#include "commands/DriveAtPowerForMeters.h"
#include "commands/DriveUntilPitchAngleChange.h"
#include "commands/ExampleCommand.h"
#include "commands/RotateAtAngle.h"
#include "commands/SelfBalancing.h"
#include "commands/TankDrive.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  //
  // TODO(josh/ethan): Y'all have a subtle bug in here, possibly based on
  // limited testing while actually driving the robot.  I'd suggest doing some
  // more testing, and come see me (or maybe get Meg to help you spot it in
  // action) if you can't find it.
  TankDrive tankDrive{
      &m_drivebase,
      [this] {
        // Figure out scaling for the current driving mode
        const double scalingFactor = GetDriveSpeedScalingFactor();
        double joystickValue;

        if (isInverted) {
          joystickValue = -1 * scalingFactor *
                          m_driverController.GetRawAxis(
                              OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        } else {
          joystickValue = +1 * scalingFactor *
                          m_driverController.GetRawAxis(
                              OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        }
        return m_leftSpeedLimiter.Calculate(joystickValue);
      },
      [this] {
        const double scalingFactor = GetDriveSpeedScalingFactor();
        double joystickValue;

        if (isInverted) {
          joystickValue = -1 * scalingFactor *
                          m_driverController.GetRawAxis(
                              OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
        } else {
          joystickValue = +1 * scalingFactor *
                          m_driverController.GetRawAxis(
                              OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
        }
        return m_rightSpeedLimiter.Calculate(joystickValue);
      }};

  m_drivebase.SetDefaultCommand(tankDrive);

  // Configure the button bindings
  ConfigureBindings();
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

frc2::Command *RobotContainer::GetAutonomousCommand() {
  frc2::Command *selectedOperation =
      m_RobotSequenceAutonomousOptions.GetSelected();
  frc2::Command *teamAndPosCmd =
      m_TeamAndStationAutonomousOptions.GetSelected();
  if (selectedOperation == nullptr || teamAndPosCmd == nullptr) {
    // This shouldn't happen if things were set up right.  But it did.  So they
    // weren't. We'll bail out, but at least return a valid pointer that will
    // tell us something went wrong when it's run.
    static frc2::PrintCommand somethingIsScrewyCommand(
        "Selection error: can't decide what to do");
    return &somethingIsScrewyCommand;
  }

  std::string operationName = selectedOperation->GetName();
  std::string teamAndPosName = teamAndPosCmd->GetName();

  if (operationName == AutonomousSelectedOperation::DoNothing) {
    static frc2::PrintCommand doNothing("Doing nothing, as instructed");
    return &doNothing;
  } else if (operationName == AutonomousSelectedOperation::GTFO) {
    if (teamAndPosName == AutonmousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonmousTeamAndStationPositions::Red2) {
      // TODO(matthew): Why are you returning a command group (that will be
      // leaked) that contains only a single command?
      //
      // Why not just have a simple staticly declared command (like in the
      // "somethingIsScrewyCommand" case above), and return its address?  This
      // will both simplify the code, and remove the memory leak in this case.
      // DONE

      static DriveAtPowerForMeters JustDriving{&m_drivebase, -0.5, 4.5_m};
      return &JustDriving;
    } else {
      // TODO(matthew): As noted above, why are you returning a command group
      // (that will be leaked) that contains only a single command? DONE

      static DriveAtPowerForMeters JustDriving{&m_drivebase, -0.5, 4.0_m};
      return &JustDriving;
    }
  } else if (operationName == AutonomousSelectedOperation::GTFODock) {
    // TODO(matthew): This block is getting pretty long.  It might be worth
    // turning it into a separate member function, similar to your initial
    // RedAndBlueDriveStation2GTFOAndBalance() function, that is invoked from
    // here to do the work.
    std::vector<std::unique_ptr<frc2::Command>> commands;
    if (teamAndPosName == AutonmousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonmousTeamAndStationPositions::Red2) {
      // In this case, we need to move back out of the community area (for the
      // mobility points), and then move forward and balance on the charging
      // station.
      std::vector<std::unique_ptr<frc2::Command>> commands;
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveAtPowerForMeters{&m_drivebase, -0.5, 4.5_m}));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveUntilPitchAngleChange{
              &m_drivebase, 0.5}));  // LOOK INTO HOW TO DO OR
      commands.push_back(
          std::unique_ptr<frc2::Command>(new SelfBalancing{&m_drivebase}));
    } else {
      // In this case, we need to move back out of the community area (for the
      // mobility points), then turn and drive until we're in line with the
      // middle of the charging station, and then move forward and balance on
      // the charging station.
      const bool firstTurnIsClockwise =
          (teamAndPosName == AutonmousTeamAndStationPositions::Blue3 ||
           teamAndPosName == AutonmousTeamAndStationPositions::Red1);
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveAtPowerForMeters{&m_drivebase, -0.5, 4.0_m}));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              RotateAtAngle{&m_drivebase, 0.5, 90_deg},
              RotateAtAngle{&m_drivebase, 0.5, -90_deg},
              [firstTurnIsClockwise]() { return firstTurnIsClockwise; })));
      commands.push_back(std::unique_ptr<frc2::Command>(
          new DriveAtPowerForMeters{&m_drivebase, 0.5, 1.889_m}));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
              RotateAtAngle{&m_drivebase, 0.5, 90_deg},
              RotateAtAngle{&m_drivebase, 0.5, -90_deg},
              [firstTurnIsClockwise]() { return firstTurnIsClockwise; })));
      commands.push_back(
          std::unique_ptr<frc2::Command>(new DriveUntilPitchAngleChange{
              &m_drivebase, 0.5}));  // LOOK INTO HOW TO DO OR
      commands.push_back(
          std::unique_ptr<frc2::Command>(new SelfBalancing{&m_drivebase}));
      // Add commands to move forward until we hit the ramp (or decide we're not
      // going to), and to then balance
    }
    return new frc2::SequentialCommandGroup(std::move(commands));
  }

  /*
  TAKE A LOOK AT CONDITIONAL COMMANDS MIGHT BE AN ALT WAY TO DO THIS
  OR JUST DO THESE BASIC DECISIONS IF
  DONE THIS WAY THEN IT SHOULD GO COMMAND THEN STATION AND COLOR
  */

  return m_RobotSequenceAutonomousOptions.GetSelected();  // CHANGE THIS
}

void RobotContainer::AddTestButtonsToSmartDashboard() {
  frc::SmartDashboard::PutData(
      "Drive 1m at 45%", new DriveAtPowerForMeters(&m_drivebase, 0.45, 1_m));
  frc::SmartDashboard::PutData(
      "Drive 1m at 70%", new DriveAtPowerForMeters(&m_drivebase, 0.70, 1_m));
  frc::SmartDashboard::PutData(
      "Drive 1m at 100%", new DriveAtPowerForMeters(&m_drivebase, 1.00, 1_m));

  frc::SmartDashboard::PutData(
      "Drive -1m at 45%", new DriveAtPowerForMeters(&m_drivebase, 0.45, -1_m));
  frc::SmartDashboard::PutData(
      "Drive -1m at 70%", new DriveAtPowerForMeters(&m_drivebase, 0.70, -1_m));
  frc::SmartDashboard::PutData(
      "Drive -1m at 100%", new DriveAtPowerForMeters(&m_drivebase, 1.00, -1_m));

  frc::SmartDashboard::PutData(
      "Drive 3m at 45%", new DriveAtPowerForMeters(&m_drivebase, 0.45, 3_m));
  frc::SmartDashboard::PutData(
      "Drive 3m at 70%", new DriveAtPowerForMeters(&m_drivebase, 0.70, 3_m));
  frc::SmartDashboard::PutData(
      "Drive 3m at 100%", new DriveAtPowerForMeters(&m_drivebase, 1.00, 3_m));

  frc::SmartDashboard::PutData(
      "Drive 0.3_m at 45%",
      new DriveAtPowerForMeters(&m_drivebase, 0.45, 0.3_m));
  frc::SmartDashboard::PutData(
      "Drive 0.3_m at 70%",
      new DriveAtPowerForMeters(&m_drivebase, 0.70, 0.3_m));
  frc::SmartDashboard::PutData(
      "Drive 0.3_m at 100%",
      new DriveAtPowerForMeters(&m_drivebase, 1.00, 0.3_m));

  frc::SmartDashboard::PutData("Rotate 90 degrees at 45%",
                               new RotateAtAngle(&m_drivebase, 0.45, 90_deg));
  frc::SmartDashboard::PutData("Rotate 90 degrees at 70%",
                               new RotateAtAngle(&m_drivebase, 0.70, 90_deg));
  frc::SmartDashboard::PutData("Rotate 90 degrees at 100%",
                               new RotateAtAngle(&m_drivebase, 1.00, 90_deg));

  frc::SmartDashboard::PutData("Rotate -90 degrees at 45%",
                               new RotateAtAngle(&m_drivebase, 0.45, -90_deg));
  frc::SmartDashboard::PutData("Rotate -90 degrees at 70%",
                               new RotateAtAngle(&m_drivebase, 0.70, -90_deg));
  frc::SmartDashboard::PutData("Rotate -90 degrees at 100%",
                               new RotateAtAngle(&m_drivebase, 1.00, -90_deg));

  frc::SmartDashboard::PutData("Rotate 180 degrees at 45%",
                               new RotateAtAngle(&m_drivebase, 0.45, 180_deg));
  frc::SmartDashboard::PutData("Rotate 180 degrees at 70%",
                               new RotateAtAngle(&m_drivebase, 0.70, 180_deg));
  frc::SmartDashboard::PutData("Rotate 180 degrees at 100%",
                               new RotateAtAngle(&m_drivebase, 1.00, 180_deg));

  frc::SmartDashboard::PutData("Rotate 45 degrees at 45%",
                               new RotateAtAngle(&m_drivebase, 0.45, 45_deg));
  frc::SmartDashboard::PutData("Rotate 45 degrees at 70%",
                               new RotateAtAngle(&m_drivebase, 0.70, 45_deg));
  frc::SmartDashboard::PutData("Rotate 45 degrees at 100%",
                               new RotateAtAngle(&m_drivebase, 1.00, 45_deg));

  frc::SmartDashboard::PutData(
      "Set Coasting Mode",
      new frc2::InstantCommand([this]() { m_drivebase.SetBrakingMode(false); },
                               {&m_drivebase}));

  frc::SmartDashboard::PutData(
      "Set Breaking Mode",
      new frc2::InstantCommand([this]() { m_drivebase.SetBrakingMode(true); },
                               {&m_drivebase}));
}

frc2::Command *BuildNamedPrintCommand(std::string name, std::string text = "") {
  if (text.empty()) {
    text = name;
  }
  frc2::Command *cmd = new frc2::PrintCommand(text);
  cmd->SetName(name);
  return cmd;
}

// Questions on if such a structure would work

/*
frc2::Command *BuildNamedPrintCommand(std::string name) {
  frc2::Command *cmd = new frc2::PrintCommand(name);
  cmd->SetName(name);
  return cmd;
}

void AddNamedCommandToSelector(frc::SendableChooser<frc2::Command *> &selector,
                               std::string name) {
  selector.AddOption(name, BuildNamedPrintCommand(name));
}

const std::list<std::string>
    nonDefaultTeamsAndPositionsList{
          {AutonmousTeamAndStationPositions::Blue2},
          {AutonmousTeamAndStationPositions::Blue3},
          {AutonmousTeamAndStationPositions::Red1},
          {AutonmousTeamAndStationPositions::Red2},
          {AutonmousTeamAndStationPositions::Red3},
    };

for (const auto &element : nonDefaultTeamsAndPositionsList) {
  AddNamedCommandToSelector(m_TeamAndStationAutonomousOptions, element);
}
*/

void RobotContainer::AddTeamAndStationSelectorToSmartDashboard() {
  m_TeamAndStationAutonomousOptions.SetDefaultOption(
      AutonmousTeamAndStationPositions::Blue1,
      BuildNamedPrintCommand(AutonmousTeamAndStationPositions::Blue1,
                             "Blue 1"));

  m_TeamAndStationAutonomousOptions.AddOption(
      AutonmousTeamAndStationPositions::Blue2,
      BuildNamedPrintCommand(AutonmousTeamAndStationPositions::Blue2,
                             "Blue 2"));

  m_TeamAndStationAutonomousOptions.AddOption(
      AutonmousTeamAndStationPositions::Blue3,
      BuildNamedPrintCommand(AutonmousTeamAndStationPositions::Blue3,
                             "Blue 3"));

  m_TeamAndStationAutonomousOptions.AddOption(
      AutonmousTeamAndStationPositions::Red1,
      BuildNamedPrintCommand(AutonmousTeamAndStationPositions::Red1, "Red 1"));

  m_TeamAndStationAutonomousOptions.AddOption(
      AutonmousTeamAndStationPositions::Red2,
      BuildNamedPrintCommand(AutonmousTeamAndStationPositions::Red2, "Red 2"));

  m_TeamAndStationAutonomousOptions.AddOption(
      AutonmousTeamAndStationPositions::Red3,
      BuildNamedPrintCommand(AutonmousTeamAndStationPositions::Red3, "Red 3"));

  frc::SmartDashboard::PutData("Team and Station Auto Selector",
                               &m_TeamAndStationAutonomousOptions);
}

void RobotContainer::AddRobotSequenceSelectorToSmartDashboard() {
  m_RobotSequenceAutonomousOptions.SetDefaultOption(
      AutonomousSelectedOperation::DoNothing,
      BuildNamedPrintCommand(AutonomousSelectedOperation::DoNothing,
                             "Do Nothing"));

  m_RobotSequenceAutonomousOptions.AddOption(
      AutonomousSelectedOperation::GTFODock,
      BuildNamedPrintCommand(AutonomousSelectedOperation::GTFODock,
                             "GTFO and Dock"));

  frc::SmartDashboard::PutData("Robot Sequence Auto Selector",
                               &m_RobotSequenceAutonomousOptions);
}

// TODO(matthew): Is this function still needed?  (You seem to have replaced it
// with the logic now in GetAutonomousCommand().) DELETED DONE