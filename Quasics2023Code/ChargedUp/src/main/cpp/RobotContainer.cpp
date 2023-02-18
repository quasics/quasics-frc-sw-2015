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
#include "commands/ClampWithIntakeAtSpeedForTime.h"
#include "commands/DriveAtPowerForMeters.h"
#include "commands/DriveUntilPitchAngleChange.h"
#include "commands/ExampleCommand.h"
#include "commands/ExtendIntakeAtSpeedForTime.h"
#include "commands/ReleaseWithIntakeAtSpeedForTime.h"
#include "commands/RetractIntakeAtSpeedForTime.h"
#include "commands/RotateAtAngle.h"
#include "commands/SelfBalancing.h"
#include "commands/TankDrive.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

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
                              OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
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
                              OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        }
        return m_rightSpeedLimiter.Calculate(joystickValue);
      }};

  m_drivebase.SetDefaultCommand(tankDrive);

  // Configure the button bindings
  ConfigureBindings();
  AddTestButtonsToSmartDashboard();
}

void RobotContainer::setInverted(bool invert) { isInverted = invert; }

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
    if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
        teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
      static DriveAtPowerForMeters JustDriving{&m_drivebase, -0.5, 4.5_m};
      return &JustDriving;
    } else {
      static DriveAtPowerForMeters JustDriving{&m_drivebase, -0.5, 4.0_m};
      return &JustDriving;
    }
  } else if (operationName == AutonomousSelectedOperation::GTFODock) {
    return GTFODOCK(teamAndPosName, &m_drivebase);
  } else if (operationName ==
             AutonomousSelectedOperation::moveToDefenseAgainstScoringWall) {
    return moveToDefenseAgainstScoringWall(teamAndPosName, &m_drivebase);
  } else if (operationName ==
             AutonomousSelectedOperation::moveToDefenseAgainstOuterWall) {
    return moveToDefenseAgainstOuterWall(teamAndPosName, &m_drivebase);
  } else if (operationName == AutonomousSelectedOperation::ScoreAndLeave) {
    return ScoreAndLeave(teamAndPosName, &m_drivebase, &m_intakeDeployment,
                         &m_intakeClamp);
    static frc2::PrintCommand doNothing("Doing nothing, as instructed");
    return &doNothing;
  } else if (operationName == AutonomousSelectedOperation::ScorePiece) {
    return ScoreGamePieceHelperCommand(&m_drivebase, &m_intakeDeployment,
                                       &m_intakeClamp);
  } else if (operationName == AutonomousSelectedOperation::JustCharge) {
    return JustCharge(teamAndPosName, &m_drivebase);
  } else if (operationName == AutonomousSelectedOperation::ScoreThenCharge) {
    return ScoreThenCharge(teamAndPosName, &m_drivebase, &m_intakeDeployment,
                           &m_intakeClamp);
  } else if (operationName ==
             AutonomousSelectedOperation::ScoreThenEndNearGamePiece) {
    return ScoreThenEndNearGamePieceCommand(
        teamAndPosName, &m_drivebase, &m_intakeDeployment, &m_intakeClamp);
  } else if (operationName == AutonomousSelectedOperation::DropGamePiece) {
    return DropGamePieceHelperCommand(&m_intakeDeployment, &m_intakeClamp);
  } else if (operationName == AutonomousSelectedOperation::DropAndGTFO) {
    return DropGamePieceThenGTFOCommand(teamAndPosName, &m_drivebase,
                                        &m_intakeDeployment, &m_intakeClamp);
  } else if (operationName == AutonomousSelectedOperation::DropAndCharge) {
    return DropGamePieceThenChargeCommand(teamAndPosName, &m_drivebase,
                                          &m_intakeDeployment, &m_intakeClamp);
  }

  return m_RobotSequenceAutonomousOptions.GetSelected();  // CHANGE THIS
}

// The functions that go into the autonomous chooser thingy above

frc2::Command *RobotContainer::GTFODOCK(std::string teamAndPosName,
                                        Drivebase *m_drivebase) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
      teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
    // In this case, we need to move back out of the community area (for the
    // mobility points), and then move forward and balance on the charging
    // station.
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters{m_drivebase, -0.5, 4.5_m}));
    commands.push_back(
        std::unique_ptr<frc2::Command>(new DriveUntilPitchAngleChange{
            m_drivebase, 0.5}));  // LOOK INTO HOW TO DO OR
    commands.push_back(
        std::unique_ptr<frc2::Command>(new SelfBalancing{m_drivebase}));
  } else {
    // In this case, we need to move back out of the community area (for the
    // mobility points), then turn and drive until we're in line with the
    // middle of the charging station, and then move forward and balance on
    // the charging station.
    const bool firstTurnIsClockwise =
        (teamAndPosName == AutonomousTeamAndStationPositions::Blue3 ||
         teamAndPosName == AutonomousTeamAndStationPositions::Red1);
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters{m_drivebase, -0.5, 4.0_m}));
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            RotateAtAngle{m_drivebase, 0.5, 90_deg},
            RotateAtAngle{m_drivebase, 0.5, -90_deg},
            [firstTurnIsClockwise]() { return firstTurnIsClockwise; })));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters{m_drivebase, 0.5, 1.889_m}));
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            RotateAtAngle{m_drivebase, 0.5, 90_deg},
            RotateAtAngle{m_drivebase, 0.5, -90_deg},
            [firstTurnIsClockwise]() { return firstTurnIsClockwise; })));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveUntilPitchAngleChange{m_drivebase, 0.5}));
    commands.push_back(
        std::unique_ptr<frc2::Command>(new SelfBalancing{m_drivebase}));
  }
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::Command *RobotContainer::moveToDefenseAgainstScoringWall(
    std::string teamAndPosName, Drivebase *m_drivebase) {
  std::vector<std::unique_ptr<frc2::Command>> commands;

  if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
      teamAndPosName == AutonomousTeamAndStationPositions::Blue3) {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new RotateAtAngle{m_drivebase, 0.5, 90_deg}));
  }

  if (teamAndPosName == AutonomousTeamAndStationPositions::Red2 ||
      teamAndPosName == AutonomousTeamAndStationPositions::Red1) {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new RotateAtAngle{m_drivebase, 0.5, -90_deg}));
  }

  if (teamAndPosName == AutonomousTeamAndStationPositions::Red1 ||
      teamAndPosName == AutonomousTeamAndStationPositions::Blue3) {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters{m_drivebase, 0.5, 3.793_m}));
  }

  if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
      teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters{m_drivebase, 0.5, 1.708_m}));  // placeholder
  }

  if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
      teamAndPosName == AutonomousTeamAndStationPositions::Blue3) {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new RotateAtAngle{m_drivebase, 0.5, -90_deg}));
  }

  if (teamAndPosName == AutonomousTeamAndStationPositions::Red2 ||
      teamAndPosName == AutonomousTeamAndStationPositions::Red1) {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new RotateAtAngle{m_drivebase, 0.5, 90_deg}));
  }

  commands.push_back(std::unique_ptr<frc2::Command>(
      new DriveAtPowerForMeters{m_drivebase, 0.5, 1.462_m}));

  if (teamAndPosName == AutonomousTeamAndStationPositions::Blue1 ||
      teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
      teamAndPosName == AutonomousTeamAndStationPositions::Blue3) {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new RotateAtAngle{m_drivebase, 0.5, 25.8_deg}));
  }

  else {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new RotateAtAngle{m_drivebase, 0.5, -25.8_deg}));
  }

  commands.push_back(std::unique_ptr<frc2::Command>(
      new DriveAtPowerForMeters{m_drivebase, 0.5, 3.845_m}));

  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::Command *RobotContainer::moveToDefenseAgainstOuterWall(
    std::string teamAndPosName, Drivebase *m_drivebase) {
  std::vector<std::unique_ptr<frc2::Command>> commands;

  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::Command *RobotContainer::ScoreAndLeave(std::string teamAndPosName,
                                             Drivebase *drivebase,
                                             IntakeDeployment *IntakeDeployment,
                                             IntakeClamp *intakeClamp) {}

frc2::Command *RobotContainer::JustCharge(std::string teamAndPosName,
                                          Drivebase *drivebase) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
      teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveUntilPitchAngleChange(drivebase, -0.5)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(new SelfBalancing(drivebase)));
  } else {
    const bool firstTurnIsClockwise =
        (teamAndPosName == AutonomousTeamAndStationPositions::Blue3 ||
         teamAndPosName == AutonomousTeamAndStationPositions::Red3);
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            RotateAtAngle{drivebase, 0.5, 90_deg},
            RotateAtAngle{drivebase, 0.5, -90_deg},
            [firstTurnIsClockwise]() { return firstTurnIsClockwise; })));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters(drivebase, 0.5, 1.719_m)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(new frc2::ConditionalCommand(
            RotateAtAngle{drivebase, 0.5, 90_deg},
            RotateAtAngle{drivebase, 0.5, -90_deg},
            [firstTurnIsClockwise]() { return firstTurnIsClockwise; })));
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveUntilPitchAngleChange(drivebase, 0.5)));
    commands.push_back(
        std::unique_ptr<frc2::Command>(new SelfBalancing(drivebase)));
  }
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::Command *RobotContainer::ScoreThenCharge(
    std::string teamAndPosName, Drivebase *drivebase,
    IntakeDeployment *intakeDeployment, IntakeClamp *intakeClamp) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::unique_ptr<frc2::Command>(
      ScoreGamePieceHelperCommand(drivebase, intakeDeployment, intakeClamp)));
  commands.push_back(
      std::unique_ptr<frc2::Command>(JustCharge(teamAndPosName, drivebase)));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::Command *RobotContainer::ScoreThenEndNearGamePieceCommand(
    std::string teamAndPosName, Drivebase *drivebase,
    IntakeDeployment *intakeDeployment, IntakeClamp *intakeClamp) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::unique_ptr<frc2::Command>(
      ScoreGamePieceHelperCommand(drivebase, intakeDeployment, intakeClamp)));
  commands.push_back(std::unique_ptr<frc2::Command>(
      new RotateAtAngle(drivebase, 0.5, 180_deg)));
  if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
      teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters(drivebase, 0.5, 5.0_m)));
  } else {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters(drivebase, 0.5, 4.5_m)));
  }
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::Command *RobotContainer::DropGamePieceThenGTFOCommand(
    std::string teamAndPosName, Drivebase *drivebase,
    IntakeDeployment *intakeDeployment, IntakeClamp *intakeClamp) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::unique_ptr<frc2::Command>(
      DropGamePieceHelperCommand(intakeDeployment, intakeClamp)));
  if (teamAndPosName == AutonomousTeamAndStationPositions::Blue2 ||
      teamAndPosName == AutonomousTeamAndStationPositions::Red2) {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters(drivebase, 0.5, 4.5_m)));
  } else {
    commands.push_back(std::unique_ptr<frc2::Command>(
        new DriveAtPowerForMeters(drivebase, 0.5, 4.0_m)));
  }
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::Command *RobotContainer::DropGamePieceThenChargeCommand(
    std::string teamAndPosName, Drivebase *drivebase,
    IntakeDeployment *intakeDeployment, IntakeClamp *intakeClamp) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::unique_ptr<frc2::Command>(
      DropGamePieceHelperCommand(intakeDeployment, intakeClamp)));
  commands.push_back(
      std::unique_ptr<frc2::Command>(JustCharge(teamAndPosName, drivebase)));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup *RobotContainer::DropGamePieceHelperCommand(
    IntakeDeployment *intakeDeployment, IntakeClamp *intakeClamp) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::unique_ptr<frc2::Command>(
      new ExtendIntakeAtSpeedForTime(intakeDeployment, 0.5, 0.5_s)));
  commands.push_back(std::unique_ptr<frc2::Command>(
      new ReleaseWithIntakeAtSpeedForTime(intakeClamp, 0.5, 0.3_s)));
  commands.push_back(std::unique_ptr<frc2::Command>(
      new RetractIntakeAtSpeedForTime(intakeDeployment, 0.5, 0.5_s)));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup *RobotContainer::ScoreGamePieceHelperCommand(
    Drivebase *drivebase, IntakeDeployment *intakeDeployment,
    IntakeClamp *intakeClamp) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::unique_ptr<frc2::Command>(
      new ExtendIntakeAtSpeedForTime(intakeDeployment, 0.5, 0.5_s)));
  commands.push_back(std::unique_ptr<frc2::Command>(
      new DriveAtPowerForMeters(drivebase, 0.5, 0.3_m)));
  commands.push_back(std::unique_ptr<frc2::Command>(
      new ReleaseWithIntakeAtSpeedForTime(intakeClamp, 0.5, 0.3_s)));
  commands.push_back(std::unique_ptr<frc2::Command>(
      new DriveAtPowerForMeters(drivebase, -0.5, 0.3_m)));
  commands.push_back(std::unique_ptr<frc2::Command>(
      new RetractIntakeAtSpeedForTime(intakeDeployment, 0.5, 0.5_s)));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

void RobotContainer::AddTestButtonsToSmartDashboard() {
  frc::SmartDashboard::PutData(
      "Drive 1m at 45%", new DriveAtPowerForMeters(&m_drivebase, 0.45, 1_m));
  frc::SmartDashboard::PutData(
      "Drive 1m at 70%", new DriveAtPowerForMeters(&m_drivebase, 0.70, 1_m));
  frc::SmartDashboard::PutData(
      "Drive 1m at 100%", new DriveAtPowerForMeters(&m_drivebase, 1.00, 1_m));

  frc::SmartDashboard::PutData(
      "Drive -1m at 25%", new DriveAtPowerForMeters(&m_drivebase, 0.25, -1_m));
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

  frc::SmartDashboard::PutData(
      "Set inverted",
      new frc2::InstantCommand([this]() { setInverted(true); }));

  frc::SmartDashboard::PutData(
      "Set not inverted",
      new frc2::InstantCommand([this]() { setInverted(false); }));
}

frc2::Command *BuildNamedPrintCommand(std::string name, std::string text = "") {
  if (text.empty()) {
    text = name;
  }
  frc2::Command *cmd = new frc2::PrintCommand(text);
  cmd->SetName(name);
  return cmd;
}

void AddNamedCommandToSelector(frc::SendableChooser<frc2::Command *> &selector,
                               std::string name, std::string text = "") {
  selector.AddOption(name, BuildNamedPrintCommand(name, text));
}

void AddingNamedPositionsToSelectorWithLoop(
    frc::SendableChooser<frc2::Command *> &selector) {
  const std::list<std::tuple<std::string, std::string>>
      nonDefaultTeamsAndPositionsList{
          {AutonomousTeamAndStationPositions::Blue2, "Blue 2"},
          {AutonomousTeamAndStationPositions::Blue3, "Blue 3"},
          {AutonomousTeamAndStationPositions::Red1, "Red 1"},
          {AutonomousTeamAndStationPositions::Red2, "Red 2"},
          {AutonomousTeamAndStationPositions::Red3, "Red 3"},
      };

  for (auto &[name, text] : nonDefaultTeamsAndPositionsList) {
    AddNamedCommandToSelector(selector, name, text);
  }
}

void AddingNamedAutonomousSequencesToSelectorWithLoop(
    frc::SendableChooser<frc2::Command *> &selector) {
  const std::list<std::tuple<std::string, std::string>>
      nonDefaultAutonomousSequenceList{
          {AutonomousSelectedOperation::DropAndCharge,
           "Drop the Game Piece then Charge"},
          {AutonomousSelectedOperation::DropAndGTFO,
           "Drop the Game Piece then Get Out of Community"},
          {AutonomousSelectedOperation::DropGamePiece,
           "Just drop the Game Piece"},
          {AutonomousSelectedOperation::GTFO, "Just Get out of the Community"},
          {AutonomousSelectedOperation::GTFODock,
           "Get Out of the Community then Get on the Charging Station"},
          {AutonomousSelectedOperation::JustCharge,
           "Just Get on the Starting Station"},
          {AutonomousSelectedOperation::moveToDefense,
           "Get out of the Community and Move to Defensive Position"},
          {AutonomousSelectedOperation::ScorePiece, "Score The game Piece"},
          {AutonomousSelectedOperation::ScoreThenCharge,
           "Score the Game Piece then Get on the Charging Station"},
          {AutonomousSelectedOperation::ScoreThenEndNearGamePiece,
           "Score then End next to a Game Piece"}};

  for (auto &[name, text] : nonDefaultAutonomousSequenceList) {
    AddNamedCommandToSelector(selector, name, text);
  }
}

void RobotContainer::AddTeamAndStationSelectorToSmartDashboard() {
  m_TeamAndStationAutonomousOptions.SetDefaultOption(
      AutonomousTeamAndStationPositions::Blue1,
      BuildNamedPrintCommand(AutonomousTeamAndStationPositions::Blue1,
                             "Blue 1"));

  AddingNamedPositionsToSelectorWithLoop(m_TeamAndStationAutonomousOptions);

  frc::SmartDashboard::PutData("Team and Station Auto Selector",
                               &m_TeamAndStationAutonomousOptions);
}

void RobotContainer::AddRobotSequenceSelectorToSmartDashboard() {
  m_RobotSequenceAutonomousOptions.SetDefaultOption(
      AutonomousSelectedOperation::DoNothing,
      BuildNamedPrintCommand(AutonomousSelectedOperation::DoNothing,
                             "Do Nothing"));

  AddingNamedAutonomousSequencesToSelectorWithLoop(
      m_RobotSequenceAutonomousOptions);

  frc::SmartDashboard::PutData("Robot Sequence Auto Selector",
                               &m_RobotSequenceAutonomousOptions);
}
