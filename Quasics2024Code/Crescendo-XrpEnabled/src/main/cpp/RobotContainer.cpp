// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include <iostream>

#include "TrajectoryGenerator.h"
#include "commands/ArcadeDrive.h"
#include "commands/Autos.h"
#include "commands/MoveClimbers.h"
#include "commands/MoveLinearActuators.h"
#include "commands/PIDRotate.h"
#include "commands/PivotIntake.h"
#include "commands/RunIntake.h"
#include "commands/RunIntakeTimed.h"
#include "commands/RunShooter.h"
#include "commands/RunShooterTimed.h"
#include "commands/SetRobotOdometry.h"
#include "commands/TankDrive.h"
#include "commands/TimedMovementTest.h"
#include "commands/Wait.h"
#include "subsystems/RealDrivebase.h"
#include "subsystems/SimulatedDrivebase.h"
#include "subsystems/XRPDrivebase.h"

constexpr bool USE_XRP_UNDER_SIMULATION = false;
constexpr bool USE_ARCADE_DRIVE = true;

namespace {
  /**
   * Builds a simple command group to run shooter at a given speed (and pushing
   * a note out of the intake into the shooter after a short delay for it to
   * come up to speed).  This is intended to help "dial in" the approximate
   * speed at which we want the shooter to be running for a given target point.
   *
   * @param shooter  the shooter
   * @param intakeRoller the intake
   * @param speed the target speed (as a % value, ranging from 0.0 to 1.0)
   */
  frc2::ParallelRaceGroup *BuildSimpleShooterSpeedTestCommand(
      Shooter &shooter, IntakeRoller &intakeRoller, double speed) {
    frc2::SequentialCommandGroup *shootSequence = nullptr;
    // "Shooty boy"
    {
      std::vector<std::unique_ptr<frc2::Command>> intakeCommands;
      intakeCommands.push_back(std::make_unique<Wait>(0.75_s));
      intakeCommands.push_back(
          std::make_unique<RunIntakeTimed>(intakeRoller, .5, 1.25_s, false));
      shootSequence =
          new frc2::SequentialCommandGroup(std::move(intakeCommands));
    }

    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(
        std::make_unique<RunShooterTimed>(shooter, speed, 2_s, true));
    commands.push_back(
        std::move(std::unique_ptr<frc2::Command>(shootSequence)));
    return new frc2::ParallelRaceGroup(std::move(commands));
  }
}  // namespace

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  allocateDriveBase();
  if (USE_ARCADE_DRIVE) {
    setUpArcadeDrive();
  } else {
    setUpTankDrive();
  }

  ConfigureDriverControllerButtonBindings();
  ConfigureOperatorControllerButtonBindings();

  AddTestButtonsOnSmartDashboard();
  AddAutoSelectionsToSmartDashboard();
  AddShooterTestButtonsToDashboard();

  std::cerr << "Log files being written to: "
            << frc::DataLogManager::GetLogDir() << std::endl;
}

void RobotContainer::setUpTankDrive() {
  // Figure out the joystick axes to be used to control driving.  (This assumes
  // that we're using the keyboard controls to simulate a joystick under the
  // simulator, or else a Logitech controller for a real robot.)
  int leftDriveJoystickAxis, rightDriveJoystickAxis;
  if (frc::RobotBase::IsSimulation()) {
    // On the keyboard:
    // * axis 0 is the "A" (negative)/"D" (positive) keys
    // * axis 1 is the "W" (negative)/"S" (positive) keys
    leftDriveJoystickAxis = 0;
    rightDriveJoystickAxis = 1;
  } else {
    leftDriveJoystickAxis = OperatorConstants::LogitechGamePad::LeftYAxis;
    rightDriveJoystickAxis = OperatorConstants::LogitechGamePad::RightYAxis;
  }

  // Create the TankDrive command (reading from the controller's joysticks), and
  // set it as the default command for the drive base.
  TankDrive::PercentSupplier leftSupplier = [=, this]() {
    const double scalingFactor = GetDriveSpeedScalingFactor();

    if (m_configSettings.normalDriveEngaged) {
      double joystickPercentage =
          m_driverController.GetRawAxis(leftDriveJoystickAxis) * -1;
      double joystickAfterScaling =
          m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
      return m_leftSlewRateLimiter.Calculate(joystickAfterScaling);
    } else {
      double joystickPercentage =
          m_driverController.GetRawAxis(rightDriveJoystickAxis);
      double joystickAfterScaling =
          m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
      return m_leftSlewRateLimiter.Calculate(joystickAfterScaling);
    }
  };
  TankDrive::PercentSupplier rightSupplier = [=, this]() {
    const double scalingFactor = GetDriveSpeedScalingFactor();

    if (m_configSettings.normalDriveEngaged) {
      double joystickPercentage =
          m_driverController.GetRawAxis(rightDriveJoystickAxis) * -1;
      double joystickAfterScaling =
          m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
      return m_rightSlewRateLimiter.Calculate(joystickAfterScaling);
    } else {
      double joystickPercentage =
          m_driverController.GetRawAxis(leftDriveJoystickAxis);
      double joystickAfterScaling =
          m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
      return m_rightSlewRateLimiter.Calculate(joystickAfterScaling);
    }
  };
  TankDrive tankDrive(*m_drivebase, leftSupplier, rightSupplier);
  m_drivebase->SetDefaultCommand(std::move(tankDrive));
}

void RobotContainer::setUpArcadeDrive() {
  int leftDriveJoystickAxis, rightDriveJoystickAxis;
  if (frc::RobotBase::IsSimulation()) {
    leftDriveJoystickAxis = 0;
    rightDriveJoystickAxis = 1;
  } else {
    leftDriveJoystickAxis = OperatorConstants::LogitechGamePad::LeftYAxis;
    rightDriveJoystickAxis = OperatorConstants::LogitechGamePad::RightXAxis;
  }

  ArcadeDrive::PercentSupplier forwardSupplier = [=, this]() {
    const double scalingFactor = GetDriveSpeedScalingFactor();

    if (m_configSettings.normalDriveEngaged) {
      double joystickPercentage =
          m_driverController.GetRawAxis(leftDriveJoystickAxis) * -1;
      double joystickAfterScaling =
          m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
      return m_leftSlewRateLimiter.Calculate(joystickAfterScaling);
    } else {
      double joystickPercentage =
          m_driverController.GetRawAxis(leftDriveJoystickAxis);
      double joystickAfterScaling =
          m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
      return m_leftSlewRateLimiter.Calculate(joystickAfterScaling);
    }
  };
  ArcadeDrive::PercentSupplier rotationSupplier = [=, this]() {
    const double scalingFactor = GetDriveSpeedScalingFactor();

    if (m_configSettings.normalDriveEngaged) {
      double joystickPercentage =
          m_driverController.GetRawAxis(rightDriveJoystickAxis) * -1;
      return m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
    } else {
      double joystickPercentage =
          m_driverController.GetRawAxis(rightDriveJoystickAxis);
      return m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
    }
  };
  ArcadeDrive arcadeDrive(*m_drivebase, forwardSupplier, rotationSupplier);
  m_drivebase->SetDefaultCommand(std::move(arcadeDrive));
}

void RobotContainer::setDriveMode(DriveMode mode) {
  m_configSettings.normalDriveEngaged = (mode == DriveMode::eNormal);
}

double RobotContainer::GetDriveSpeedScalingFactor() {
  const bool isTurbo = m_driverController.GetRawButton(
      OperatorConstants::LogitechGamePad::RightShoulder);
  const bool isTurtle = m_driverController.GetRawButton(
      OperatorConstants::LogitechGamePad::LeftShoulder);

  if (isTurbo) {
    return RobotSpeedScaling::TURBO_MODE_SPEED_SCALING;
  } else if (isTurtle) {
    return RobotSpeedScaling::TURTLE_MODE_SPEED_SCALING;
  } else {
    return RobotSpeedScaling::NORMAL_MODE_SPEED_SCALING;
  }
}

void RobotContainer::allocateDriveBase() {
  if (frc::RobotBase::IsReal()) {
    // OK, we're running on a "big bot".
    m_drivebase.reset(new RealDrivebase);
  } else {
#ifdef ENABLE_XRP
    // OK, we're running under simulation.  However, this could either mean
    // that we're talking to an XRP "little bot", or doing *pure* (GUI-based)
    // simulation on a PC of some sort.
    if (USE_XRP_UNDER_SIMULATION) {
      m_drivebase.reset(new XRPDrivebase);
    } else {
      m_drivebase.reset(new SimulatedDrivebase);
    }
#else
    // XRP isn't enabled, so we'll just go with the pure simulator.
    m_drivebase.reset(new SimulatedDrivebase);
#endif
  }
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  frc2::Command *selectedOperation = m_OverallAutonomousOptions.GetSelected();
  frc2::Command *teamAndPosCmd =
      m_TeamAndStationAutonomousOptions.GetSelected();
  frc2::Command *score2Dest = m_Score2DestAutonomousOptions.GetSelected();
  frc2::Command *score3Dest = m_Score3DestAutonomousOptions.GetSelected();

  if (selectedOperation == nullptr || teamAndPosCmd == nullptr) {
    // This shouldn't happen if things were set up right.  But it did.  So they
    // weren't. We'll bail out, but at least return a valid pointer that will
    // tell us something went wrong when it's run.
    frc2::PrintCommand somethingIsScrewyCommand(
        "Selection error: can't decide what to do");
    return std::move(somethingIsScrewyCommand).ToPtr();
  }

  std::string operationName = selectedOperation->GetName();
  std::string position = teamAndPosCmd->GetName();
  std::string score2DestName = score2Dest->GetName();
  std::string score3DestName = score3Dest->GetName();

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  return AutonomousCommands::GetAutonomousCommand(
      *m_drivebase, m_shooter, m_intakeDeployment, m_intakeRoller,
      operationName, position, score2DestName, score3DestName);
#else
  return AutonomousCommands::GetAutonomousCommand(
      *m_drivebase, operationName, position, score2DestName, score3DestName);
#endif
}

void RobotContainer::AddShooterTestButtonsToDashboard() {
  for (double d = 5.5; d <= 10; d += 0.5) {
    std::ostringstream sout;
    sout << "Shoot @ " << d << "%";
    auto *cmd = BuildSimpleShooterSpeedTestCommand(m_shooter, m_intakeRoller,
                                                   d / 100.0);

    frc::SmartDashboard::PutData(sout.str(), cmd);
  }
}

void RobotContainer::AddTestButtonsOnSmartDashboard() {
  // This is needed because we cannot just input a command ptr onto the FRC
  // Smart Dashboard bc it will be deleted and some values that it had would
  // be still needed. So one thing sais that it needs it, but there is no real
  // data behind it.  This allows us to make this data storage more permanent.
  //
  // TODO: (Matthew) Does this comment still apply?  If not, please remove it.

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  frc::SmartDashboard::PutData("Extend Climbers",
                               new MoveClimbers(m_climber, true));
  frc::SmartDashboard::PutData("Retract Climbers",
                               new MoveClimbers(m_climber, false));

  frc::SmartDashboard::PutData("Shoot Note",
                               new RunShooter(m_shooter, 0.25, true));
  frc::SmartDashboard::PutData("Retract Note",
                               new RunShooter(m_shooter, 0.25, false));
  frc::SmartDashboard::PutData(
      "reset Climber Revolutions:",
      new frc2::InstantCommand([this]() { m_climber.resetRevolutions(); }));
#endif

  frc::SmartDashboard::PutData(
      "reset encoders",
      new frc2::InstantCommand([this]() { m_drivebase->resetEncoders(); }));
#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  frc::SmartDashboard::PutData("coast intake",
                               new frc2::InstantCommand([this]() {
                                 m_intakeDeployment.EnableBraking(false);
                               }));

  frc::SmartDashboard::PutData("Reset Deployment Encoder",
                               new frc2::InstantCommand([this]() {
                                 m_intakeDeployment.ResetEncoders();
                               }));
#endif
  frc::SmartDashboard::PutData("reset odometry directly",
                               new frc2::InstantCommand([this]() {
                                 m_drivebase->resetOdometry(frc::Pose2d());
                               }));

  frc::SmartDashboard::PutData(
      "reset Odometry(via command) to (3,6)",
      new SetRobotOdometry(*m_drivebase, frc::Pose2d(3_m, 6_m, 0_rad)));

#ifdef ENABLE_INTAKE_TESTING
  frc::SmartDashboard::PutData("Run Intake 50%",
                               new RunIntake(&m_intakeRoller, 0.5, true));
  frc::SmartDashboard::PutData("Run Intake 60%",
                               new RunIntake(&m_intakeRoller, 0.6, true));
  frc::SmartDashboard::PutData("Run Intake 70%",
                               new RunIntake(&m_intakeRoller, 0.7, true));
  frc::SmartDashboard::PutData("Run Intake 80%",
                               new RunIntake(&m_intakeRoller, 0.8, true));
  frc::SmartDashboard::PutData("Retract Intake 50%",
                               new RunIntake(&m_intakeRoller, 0.5, false));
  frc::SmartDashboard::PutData("Deploy Intake 30%",
                               new PivotIntake(&m_intakeDeployment, 0.3, true));
  frc::SmartDashboard::PutData("Deploy Intake 50%",
                               new PivotIntake(&m_intakeDeployment, 0.5, true));
  frc::SmartDashboard::PutData("Deploy Intake 60%",
                               new PivotIntake(&m_intakeDeployment, 0.6, true));
  frc::SmartDashboard::PutData("Deploy Intake 70%",
                               new PivotIntake(&m_intakeDeployment, 0.7, true));
  frc::SmartDashboard::PutData(
      "Retract Intake 50%", new PivotIntake(&m_intakeDeployment, 0.5, false));
#endif

  frc::SmartDashboard::PutData(
      "Switch Drive", new frc2::InstantCommand(
                          [this]() { setDriveMode(DriveMode::eSwitched); }));

  frc::SmartDashboard::PutData(
      "Normal Drive",
      new frc2::InstantCommand([this]() { setDriveMode(DriveMode::eNormal); }));
#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  frc::SmartDashboard::PutData("Enable Coast",
                               new frc2::InstantCommand([this]() {
                                 m_intakeDeployment.EnableBraking(false);
                               }));
#endif

  frc::SmartDashboard::PutData("Rotate 90 degrees (UNTESTED)",
                               new PIDRotate(*m_drivebase, 90_deg));

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  frc::SmartDashboard::PutData("Extend Actuator",
                               new MoveLinearActuators(m_shooter, true));

  frc::SmartDashboard::PutData("Retract Actuator",
                               new MoveLinearActuators(m_shooter, false));
#endif

  // SysId Commands
  static frc2::CommandPtr quasistaticForward =
      m_drivebase->sysIdQuasistatic(frc2::sysid::kForward);
  static frc2::CommandPtr quasistaticReverse =
      m_drivebase->sysIdQuasistatic(frc2::sysid::kReverse);
  static frc2::CommandPtr dynamicForward =
      m_drivebase->sysIdDynamic(frc2::sysid::kForward);
  static frc2::CommandPtr dynamicReverse =
      m_drivebase->sysIdDynamic(frc2::sysid::kReverse);
  frc::SmartDashboard::PutData("Quasistatic Forward", quasistaticForward.get());
  frc::SmartDashboard::PutData("Quasistatic Reverse", quasistaticReverse.get());
  frc::SmartDashboard::PutData("Dynamic Forward", dynamicForward.get());
  frc::SmartDashboard::PutData("Dynamic Reverse", dynamicReverse.get());
}

void RobotContainer::RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                                      frc2::Command *command) {
  frc2::JoystickButton(&m_driverController, logitechButtonId)
      .WhileTrue(command);
}

void RobotContainer::RunCommandWhenOperatorButtonIsHeld(
    int buttonId, frc2::Command *command) {
  frc2::JoystickButton(&m_operatorController, buttonId).WhileTrue(command);
}

void RobotContainer::ConfigureDriverControllerButtonBindings() {
#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  static MoveClimbers extendClimbers(m_climber, true);
  static MoveClimbers retractClimbers(m_climber, false);
  static RunIntake intakeNote(m_intakeRoller, 1.00, true);
  static RunIntake dropNote(m_intakeRoller, 1.00, false);

  RunCommandWhenDriverButtonIsHeld(OperatorConstants::LogitechGamePad::YButton,
                                   &extendClimbers);
  RunCommandWhenDriverButtonIsHeld(OperatorConstants::LogitechGamePad::AButton,
                                   &retractClimbers);
  RunCommandWhenDriverButtonIsHeld(
      OperatorConstants::LogitechGamePad::LeftShoulder, &dropNote);
  RunCommandWhenDriverButtonIsHeld(
      OperatorConstants::LogitechGamePad::RightShoulder, &intakeNote);
#endif
}

void RobotContainer::ConfigureOperatorControllerButtonBindings() {
#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  static PivotIntake extendIntake(m_intakeDeployment, 0.5, true);
  static PivotIntake retractIntake(m_intakeDeployment, 0.5, false);
  // static RunShooter shootNote(m_shooter, 0.5, true);
  static RunShooter shootNote(m_shooter, 1.00, true);
  static frc2::ParallelRaceGroup *shootSequence = ShootingSequence(false);

  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kA,
                                     &extendIntake);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kY,
                                     &retractIntake);
  /*RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kB,
                                     &shootNote);*/
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kB,
                                     shootSequence);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kX,
                                     shootSequence);
#endif
}

frc2::Command *RobotContainer::BuildNamedPrintCommand(std::string name,
                                                      std::string text) {
  if (text.empty()) {
    text = name;
  }
  frc2::Command *cmd = new frc2::PrintCommand(text);
  cmd->SetName(name);
  return cmd;
}

void RobotContainer::AddNamedCommandToSelector(
    frc::SendableChooser<frc2::Command *> &selector, std::string name,
    std::string text) {
  selector.AddOption(name, BuildNamedPrintCommand(name, text));
}

void RobotContainer::AddingNamedStartingPositionsToSelector(
    frc::SendableChooser<frc2::Command *> &selector) {
  const std::list<std::tuple<std::string, std::string>>
      nonDefaultTeamsAndPositionsList{
          {AutonomousStartingPositions::leftOfSpeaker,
           AutonomousStartingPositions::leftOfSpeaker},
          {AutonomousStartingPositions::inFrontOfSpeaker,
           AutonomousStartingPositions::inFrontOfSpeaker},
          {AutonomousStartingPositions::rightOfSpeaker,
           AutonomousStartingPositions::rightOfSpeaker},
          {AutonomousStartingPositions::farField,
           AutonomousStartingPositions::farField},
      };

  for (auto &[name, text] : nonDefaultTeamsAndPositionsList) {
    AddNamedCommandToSelector(selector, name, text);
  }
}

void RobotContainer::AddingNamedOverallOperationsToSelector(
    frc::SendableChooser<frc2::Command *> &selector) {
  const std::list<std::tuple<std::string, std::string>>
      nonDefaultAutonomousSequenceList{
          {AutonomousSelectedOperation::GTFO,
           AutonomousSelectedOperation::GTFO},
          {AutonomousSelectedOperation::score1,
           AutonomousSelectedOperation::score1},
          {AutonomousSelectedOperation::score1GTFO,
           AutonomousSelectedOperation::score1GTFO},
          {AutonomousSelectedOperation::score2,
           AutonomousSelectedOperation::score2},
          {AutonomousSelectedOperation::score2GTFO,
           AutonomousSelectedOperation::score2GTFO},
          {AutonomousSelectedOperation::score3,
           AutonomousSelectedOperation::score3},
          {AutonomousSelectedOperation::score3GTFO,
           AutonomousSelectedOperation::score3GTFO},
          {AutonomousSelectedOperation::score4,
           AutonomousSelectedOperation::score4},
      };

  for (auto &[name, text] : nonDefaultAutonomousSequenceList) {
    AddNamedCommandToSelector(selector, name, text);
  }
}

void RobotContainer::AddingNamedScoreDestinationsToSelector(
    frc::SendableChooser<frc2::Command *> &selector1,
    frc::SendableChooser<frc2::Command *> &selector2) {
  const std::list<std::tuple<std::string, std::string>>
      nonDefaultTeamsAndPositionsList{
          {AutonomousScoreDestinations::amp, AutonomousScoreDestinations::amp},
          {AutonomousScoreDestinations::leftOfSpeaker,
           AutonomousScoreDestinations::leftOfSpeaker},
          {AutonomousScoreDestinations::inFrontOfSpeaker,
           AutonomousScoreDestinations::inFrontOfSpeaker},
          {AutonomousScoreDestinations::rightOfSpeaker,
           AutonomousScoreDestinations::rightOfSpeaker},
      };

  for (auto &[name, text] : nonDefaultTeamsAndPositionsList) {
    AddNamedCommandToSelector(selector1, name, text);
    AddNamedCommandToSelector(selector2, name, text);
  }
}

void RobotContainer::AddAutoSelectionsToSmartDashboard() {
  AddTeamAndStationSelectorToSmartDashboard();
  AddRobotOverallOperationToSmartDashboard();
  AddScoreDestinationsToSmartDashboard();
}

void RobotContainer::AddTeamAndStationSelectorToSmartDashboard() {
  m_TeamAndStationAutonomousOptions.SetDefaultOption(
      AutonomousStartingPositions::inFrontOfAmp,
      BuildNamedPrintCommand(AutonomousStartingPositions::inFrontOfAmp));

  AddingNamedStartingPositionsToSelector(m_TeamAndStationAutonomousOptions);

  frc::SmartDashboard::PutData("Team and Station Auto Selector",
                               &m_TeamAndStationAutonomousOptions);
}

void RobotContainer::AddRobotOverallOperationToSmartDashboard() {
  m_OverallAutonomousOptions.SetDefaultOption(
      AutonomousSelectedOperation::doNothing,
      BuildNamedPrintCommand(AutonomousSelectedOperation::doNothing));

  AddingNamedOverallOperationsToSelector(m_OverallAutonomousOptions);

  frc::SmartDashboard::PutData("Robot Overall Auto Selector",
                               &m_OverallAutonomousOptions);
}

void RobotContainer::AddScoreDestinationsToSmartDashboard() {
  m_Score2DestAutonomousOptions.SetDefaultOption(
      "Not Selected", BuildNamedPrintCommand("Not Selected"));
  m_Score3DestAutonomousOptions.SetDefaultOption(
      "Not Selected", BuildNamedPrintCommand("Not Selected"));

  AddingNamedScoreDestinationsToSelector(m_Score2DestAutonomousOptions,
                                         m_Score3DestAutonomousOptions);

  frc::SmartDashboard::PutData("Robot Score 2 Destination",
                               &m_Score2DestAutonomousOptions);
  frc::SmartDashboard::PutData("Robot Score 3 Destination",
                               &m_Score3DestAutonomousOptions);
}

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
frc2::ParallelRaceGroup *RobotContainer::ShootingSequence(bool amp) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<RunShooterTimed>(
      m_shooter, (amp ? ShooterSpeeds::amp : ShooterSpeeds::speaker), 2_s,
      true));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(IntakeDelay())));

  return new frc2::ParallelRaceGroup(std::move(commands));
}

frc2::SequentialCommandGroup *RobotContainer::IntakeDelay() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(
      std::make_unique<TimedMovementTest>(*m_drivebase, .25, .1_s, true));
  commands.push_back(std::make_unique<Wait>(0.65_s));
  commands.push_back(
      std::make_unique<RunIntakeTimed>(m_intakeRoller, .5, 1.25_s, false));

  return new frc2::SequentialCommandGroup(std::move(commands));
}
#endif