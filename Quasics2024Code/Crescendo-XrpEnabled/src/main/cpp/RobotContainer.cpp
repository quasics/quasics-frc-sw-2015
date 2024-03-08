// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include <iostream>

#include "TrajectoryGenerator.h"
#include "commands/ArcadeDrive.h"
#include "commands/Autos.h"
#include "commands/MoveClimbers.h"
#include "commands/MoveClimbersAuto.h"
#include "commands/MoveLinearActuators.h"
#include "commands/PIDRotate.h"
#include "commands/PivotIntake.h"
#include "commands/PivotIntakeAuto.h"
#include "commands/RotateToAprilTarget.h"
#include "commands/RunIntake.h"
#include "commands/RunIntakeTimed.h"
#include "commands/RunScorerTimed.h"
#include "commands/RunShooter.h"
#include "commands/RunShooterTimed.h"
#include "commands/SetRobotOdometry.h"
#include "commands/TankDrive.h"
#include "commands/TimedMovementTest.h"
#include "commands/TriggerBasedIntaking.h"
#include "commands/TriggerBasedShooting.h"
#include "commands/Wait.h"
#include "subsystems/RealDrivebase.h"
#include "subsystems/SimulatedDrivebase.h"
#include "subsystems/XRPDrivebase.h"

constexpr bool USE_XRP_UNDER_SIMULATION = false;
constexpr bool USE_ARCADE_DRIVE = true;

RobotContainer::RobotContainer() {
  allocateDriveBase();
  if (USE_ARCADE_DRIVE) {
    setUpArcadeDrive();
  } else {
    setUpTankDrive();
  }

  ConfigureDriverControllerButtonBindings();
  ConfigureOperatorControllerButtonBindings();

  SetDefaultShooterCommand();
#ifndef ENABLE_COMPETITION_ROBOT
  AddTestButtonsOnSmartDashboard();

#ifdef ENABLE_INTAKE_TESTING
  AddIntakeTestButtonsToDashboard();
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  AddClimberTestButtonsToDashboard();
  AddActuatorTestButtonsToDashboard();
  AddShooterTestButtonsToDashboard();
#ifdef ENABLE_SHOOTER_SPEED_TESTS
  AddShooterSpeedTestButtonsToDashboard();
#endif

#endif
  AddDriveTestButtonsToDashboard();
  AddSysIdButtonsToDashboard();

#ifdef ENABLE_VISION_TESTING
  AddVisionTestButtonsToDashboard();
#endif

#endif
#ifdef ENABLE_COMPETITION_ROBOT
  AddCompetitionButtonsToSmartDashboard();
  // AddClimberTestButtonsToDashboard();
  //  AddScorerTestButtonsToDashboard();
#endif

  AddAutoSelectionsToSmartDashboard();

  std::cerr << "Log files being written to: "
            << frc::DataLogManager::GetLogDir() << std::endl;
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

void RobotContainer::setUpTankDrive() {
  // Figure out the joystick axes to be used to control driving.  (This assumes
  // that we're using the keyboard controls to simulate a joystick under the
  // simulator, or else a Logitech controller for a real robot.)
  int leftDriveJoystickAxis, rightDriveJoystickAxis;
  if (frc::RobotBase::IsSimulation()) {
    // On the keyboard:
    // * axis 0 is the "A" (negative)/"D" (positive) keys
    // * axis 1 is the "W" (negative)/"S" (positive) keys
    rightDriveJoystickAxis = 0;
    leftDriveJoystickAxis = 1;
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
          m_driverController.GetRawAxis(leftDriveJoystickAxis);
      double joystickAfterScaling =
          m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
      return m_leftSlewRateLimiter.Calculate(joystickAfterScaling);
    } else {
      double joystickPercentage =
          m_driverController.GetRawAxis(leftDriveJoystickAxis) * -1;
      double joystickAfterScaling =
          m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor;
      return m_leftSlewRateLimiter.Calculate(joystickAfterScaling);
    }
  };
  ArcadeDrive::PercentSupplier rotationSupplier = [=, this]() {
    double scalingFactor;
    if (m_rotateFast) {
      scalingFactor = 10;  // 2 to correct for 0.5 in return statement
    } else {
      scalingFactor = GetDriveSpeedScalingFactor();
    }
    double joystickPercentage =
        m_driverController.GetRawAxis(rightDriveJoystickAxis);
    return m_joystickDeadbandEnforcer(joystickPercentage) * scalingFactor * -1 *
           0.5;
  };
  ArcadeDrive arcadeDrive(*m_drivebase, forwardSupplier, rotationSupplier);
  m_drivebase->SetDefaultCommand(std::move(arcadeDrive));
}

void RobotContainer::SetDefaultShooterCommand() {
#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  TriggerBasedShooting triggerBasedShooterCommand(m_shooter,
                                                  &m_operatorController);

  m_shooter.SetDefaultCommand(std::move(triggerBasedShooterCommand));
#endif
  /*TriggerBasedIntaking triggerBasedIntakeCommand(m_intakeRoller,
                                                 &m_driverController);
  m_intakeRoller.SetDefaultCommand(std::move(triggerBasedIntakeCommand));*/
}

void RobotContainer::RunCommandWhenDriverButtonIsPressed(
    int logitechButtonId, frc2::Command *command) {
  frc2::JoystickButton(&m_driverController, logitechButtonId)
      .Debounce(50_ms, frc::Debouncer::DebounceType::kBoth)
      .OnTrue(command);
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
  RunCommandWhenDriverButtonIsPressed(
      OperatorConstants::LogitechGamePad::BButton,
      new frc2::InstantCommand([this]() {
        m_configSettings.normalDriveEngaged =
            !m_configSettings.normalDriveEngaged;
      }));

  RunCommandWhenDriverButtonIsPressed(
      OperatorConstants::LogitechGamePad::XButton,
      new frc2::InstantCommand([this]() { m_rotateFast = !m_rotateFast; }));

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  // static MoveClimbers extendClimbers(m_climber, true);
  // static MoveClimbers retractClimbers(m_climber, false);
  static MoveClimbersAuto extendClimbers(m_climber, true);
  static MoveClimbersAuto retractClimbers(m_climber, false);
  static RunIntake intakeNote(m_intakeRoller, 0.75, true);
  static RunIntake dropNote(m_intakeRoller, 0.75, false);

  RunCommandWhenDriverButtonIsHeld(OperatorConstants::LogitechGamePad::YButton,
                                   &extendClimbers);
  RunCommandWhenDriverButtonIsHeld(OperatorConstants::LogitechGamePad::AButton,
                                   &retractClimbers);
  RunCommandWhenDriverButtonIsHeld(
      OperatorConstants::LogitechGamePad::LeftTrigger, &dropNote);
  RunCommandWhenDriverButtonIsHeld(
      OperatorConstants::LogitechGamePad::RightTrigger, &intakeNote);
#endif
}

void RobotContainer::ConfigureOperatorControllerButtonBindings() {
#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  static PivotIntake extendIntake(m_intakeDeployment, 0.5, true);
  // static PivotIntake retractIntake(m_intakeDeployment, 0.5, false);
  static frc2::ParallelRaceGroup *retractIntake = intakeWhileRetracting();
  // static RunShooter shootNote(m_shooter, 0.5, true);
  static RunShooter shootNote(m_shooter, 1.00, true);
  static frc2::ParallelRaceGroup *ampSequence =
      ShootInAmpThenRunActuatorAfterTime(1_s);
  static frc2::ParallelRaceGroup *shootSequence = ShootingSequence(false);

  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kA,
                                     &extendIntake);
  /*RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kY,
                                     &retractIntake);*/
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kY,
                                     retractIntake);
  /*RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kB,
                                     &shootNote);*/
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kB,
                                     shootSequence);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kX,
                                     ampSequence);
#endif
}

frc2::ParallelRaceGroup *RobotContainer::intakeWhileRetracting() {
  std::vector<std::unique_ptr<frc2::Command>> commands;

  commands.push_back(
      std::make_unique<PivotIntake>(m_intakeDeployment, 0.5, false));
  commands.push_back(std::make_unique<RunIntake>(m_intakeRoller, 0.4, true));

  return new frc2::ParallelRaceGroup(std::move(commands));
}

void RobotContainer::setDriveMode(DriveMode mode) {
  m_configSettings.normalDriveEngaged = (mode == DriveMode::eNormal);
}

double RobotContainer::GetDriveSpeedScalingFactor() {
  // LOOK ITO THIS
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

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  frc2::Command *selectedOperation = m_OverallAutonomousOptions.GetSelected();
  frc2::Command *position = m_PositionAutonomousOptions.GetSelected();
  frc2::Command *score2Dest = m_Score2DestAutonomousOptions.GetSelected();
  frc2::Command *score3Dest = m_Score3DestAutonomousOptions.GetSelected();

  if (selectedOperation == nullptr || position == nullptr) {
    // This shouldn't happen if things were set up right.  But it did.  So they
    // weren't. We'll bail out, but at least return a valid pointer that will
    // tell us something went wrong when it's run.
    frc2::PrintCommand somethingIsScrewyCommand(
        "Selection error: can't decide what to do");
    return std::move(somethingIsScrewyCommand).ToPtr();
  }

  std::string operationName = selectedOperation->GetName();
  std::string positionName = position->GetName();
  std::string score2DestName = score2Dest->GetName();
  std::string score3DestName = score3Dest->GetName();
  const frc::DriverStation::Alliance alliance =
      frc::DriverStation::GetAlliance().value_or(
          frc::DriverStation::Alliance::kBlue);
  const bool isBlue = alliance == frc::DriverStation::Alliance::kBlue;
  std::cerr << "Operation name: " << operationName << std::endl;

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  return AutonomousCommands::GetAutonomousCommand(
      *m_drivebase, m_intakeDeployment, m_intakeRoller, m_shooter,
      operationName, positionName, score2DestName, score3DestName, isBlue);
#else
  return AutonomousCommands::GetAutonomousCommand(*m_drivebase, operationName,
                                                  positionName, score2DestName,
                                                  score3DestName, isBlue);
#endif
}

void RobotContainer::AddCompetitionButtonsToSmartDashboard() {
  frc::SmartDashboard::PutData(
      "Switch Drive", new frc2::InstantCommand(
                          [this]() { setDriveMode(DriveMode::eSwitched); }));

  frc::SmartDashboard::PutData(
      "Normal Drive",
      new frc2::InstantCommand([this]() { setDriveMode(DriveMode::eNormal); }));
#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  frc::SmartDashboard::PutData("Run Shooter Reversed",
                               new frc2::InstantCommand([this]() {
                                 RunShooterTimed(m_shooter, 0.5, 1_s, false);
                               }));

#endif
}

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY

void RobotContainer::AddShooterTestButtonsToDashboard() {
  frc::SmartDashboard::PutData("Shoot Note",
                               new RunShooter(m_shooter, 0.25, true));
  frc::SmartDashboard::PutData("Retract Note",
                               new RunShooter(m_shooter, 0.25, false));
}

void RobotContainer::AddIntakeTestButtonsToDashboard() {
  frc::SmartDashboard::PutData("Run Intake 50%",
                               new RunIntake(m_intakeRoller, 0.5, true));
  frc::SmartDashboard::PutData("Run Intake 60%",
                               new RunIntake(m_intakeRoller, 0.6, true));
  frc::SmartDashboard::PutData("Run Intake 70%",
                               new RunIntake(m_intakeRoller, 0.7, true));
  frc::SmartDashboard::PutData("Run Intake 80%",
                               new RunIntake(m_intakeRoller, 0.8, true));
  frc::SmartDashboard::PutData("Retract Intake 50%",
                               new RunIntake(m_intakeRoller, 0.5, false));
  frc::SmartDashboard::PutData("Deploy Intake 30%",
                               new PivotIntake(m_intakeDeployment, 0.3, true));
  frc::SmartDashboard::PutData("Deploy Intake 50%",
                               new PivotIntake(m_intakeDeployment, 0.5, true));
  frc::SmartDashboard::PutData("Deploy Intake 60%",
                               new PivotIntake(m_intakeDeployment, 0.6, true));
  frc::SmartDashboard::PutData("Deploy Intake 70%",
                               new PivotIntake(m_intakeDeployment, 0.7, true));
  frc::SmartDashboard::PutData("Retract Intake 50%",
                               new PivotIntake(m_intakeDeployment, 0.5, false));
  frc::SmartDashboard::PutData("Coast intake",
                               new frc2::InstantCommand([this]() {
                                 m_intakeDeployment.EnableBraking(false);
                               }));
  frc::SmartDashboard::PutData("Reset Deployment Encoder",
                               new frc2::InstantCommand([this]() {
                                 m_intakeDeployment.ResetEncoders();
                               }));
}

void RobotContainer::AddActuatorTestButtonsToDashboard() {
  frc::SmartDashboard::PutData(
      "Extend Actuator", new MoveLinearActuators(m_linearActuators, true));

  frc::SmartDashboard::PutData(
      "Retract Actuator", new MoveLinearActuators(m_linearActuators, false));

  frc::SmartDashboard::PutData("Shoot in amp then run actuator after time",
                               new frc2::InstantCommand([this]() {
                                 ShootInAmpThenRunActuatorAfterTime(1_s);
                               }));

  frc::SmartDashboard::PutData("Extend then retract linear actuator after time",
                               ExtendThenRetractActuatorsAfterTime(.75_s));
}

void RobotContainer::AddScorerTestButtonsToDashboard() {
  frc::SmartDashboard::PutData(
      "Run scorers 50 up",
      new RunScorerTimed(m_pivotScorer, 0.50, 0.5_s, true));
  frc::SmartDashboard::PutData(
      "Run scorers 50 down",
      new RunScorerTimed(m_pivotScorer, 0.50, 0.5_s, false));
}

namespace {
  frc2::SequentialCommandGroup *BuildSequenceToHandNoteToShooterAfterDelay(
      IntakeRoller &intakeRoller, units::second_t delay = 0.75_s,
      double intakeSpeed = .5, units::second_t timeToRunIntake = 1.25_s) {
    std::vector<std::unique_ptr<frc2::Command>> intakeCommands;
    intakeCommands.push_back(std::make_unique<Wait>(delay));
    intakeCommands.push_back(std::make_unique<RunIntakeTimed>(
        intakeRoller, intakeSpeed, timeToRunIntake,
        /*takingIn*/ false));
    return new frc2::SequentialCommandGroup(std::move(intakeCommands));
  }

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
    std::vector<std::unique_ptr<frc2::Command>> commands;

    // Run the shooter for 2sec @ the target speed...
    commands.push_back(
        std::make_unique<RunShooterTimed>(shooter, speed, 2_s, true));

    // ...and hand off the note to it after a short delay (to come up to speed).
    commands.push_back(std::move(std::unique_ptr<frc2::Command>(
        BuildSequenceToHandNoteToShooterAfterDelay(intakeRoller))));

    return new frc2::ParallelRaceGroup(std::move(commands));
  }
}  // namespace

void RobotContainer::AddShooterSpeedTestButtonsToDashboard() {
  for (double d = 5.5; d <= 10; d += 0.5) {
    std::ostringstream sout;
    sout << "Shoot @ " << d << "%";
    auto *cmd = BuildSimpleShooterSpeedTestCommand(m_shooter, m_intakeRoller,
                                                   d / 100.0);

    frc::SmartDashboard::PutData(sout.str(), cmd);
  }
}
/*altenative method below. Doing it this way so I can bind stuff
frc2::CommandPtr RobotContainer::ShootInAmpThenRunActuatorAfterTime(
    units::second_t time) {
  std::vector<frc2::CommandPtr> commands;

  commands.push_back(std::move(*BuildSimpleShooterSpeedTestCommand(
                                   m_shooter, m_intakeRoller, 0.08))
                         .ToPtr());
  commands.push_back(ExtendThenRetractActuatorsAfterTime(time));

  return frc2::ParallelCommandGroup(
             frc2::CommandPtr::UnwrapVector(std::move(commands)))
      .ToPtr();
}*/

frc2::ParallelRaceGroup *RobotContainer::ShootInAmpThenRunActuatorAfterTime(
    units::second_t time) {
  std::vector<std::unique_ptr<frc2::Command>> commands;

  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      BuildSimpleShooterSpeedTestCommand(m_shooter, m_intakeRoller, 0.14))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      ExtendThenRetractActuatorsAfterTime(time))));

  return new frc2::ParallelRaceGroup(std::move(commands));
}

// Making an alternative method for button binding purposes

/*
  frc2::CommandPtr RobotContainer::ExtendThenRetractActuatorsAfterTime(
      units::second_t time) {
    std::vector<frc2::CommandPtr> commands;
    commands.push_back(frc2::CommandPtr(Wait(time)));
    commands.push_back(
        frc2::CommandPtr(MoveLinearActuators(m_linearActuators, true)));
    commands.push_back(
        frc2::CommandPtr(MoveLinearActuators(m_linearActuators, false)));

    return frc2::SequentialCommandGroup(
               frc2::CommandPtr::UnwrapVector(std::move(commands)))
        .ToPtr();
  }*/

frc2::SequentialCommandGroup *
RobotContainer::ExtendThenRetractActuatorsAfterTime(units::second_t time) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<Wait>(time));
  commands.push_back(
      std::make_unique<RunScorerTimed>(m_pivotScorer, 0.5, 0.25_s, true));
  commands.push_back(std::make_unique<Wait>(5_s));
  commands.push_back(
      std::make_unique<RunScorerTimed>(m_pivotScorer, 0.5, 0.25_s, false));

  return new frc2::SequentialCommandGroup(std::move(commands));
}

void RobotContainer::AddClimberTestButtonsToDashboard() {
  frc::SmartDashboard::PutData("Extend Climbers",
                               new MoveClimbers(m_climber, true));
  frc::SmartDashboard::PutData("Retract Climbers",
                               new MoveClimbers(m_climber, false));
  frc::SmartDashboard::PutData("Auto Extend Climbers",
                               new MoveClimbersAuto(m_climber, true));
  frc::SmartDashboard::PutData("Auto Retract Climbers",
                               new MoveClimbersAuto(m_climber, false));
  frc::SmartDashboard::PutData(
      "Reset Climber Revolutions:",
      new frc2::InstantCommand([this]() { m_climber.resetRevolutions(); }));
  frc::SmartDashboard::PutData(
      "Set Climber Revolutions -3:",
      new frc2::InstantCommand([this]() { m_climber.setRevolutions(); }));
}
#endif
void RobotContainer::AddVisionTestButtonsToDashboard() {
#ifdef ENABLE_VISION_TESTING
  frc::SmartDashboard::PutData(
      "Rotate to blue speaker",
      new RotateToAprilTarget(*m_drivebase, *m_vision, 6));
#endif
}

void RobotContainer::AddTestButtonsOnSmartDashboard() {
  frc::SmartDashboard::PutData("Rotate 90 degrees (UNTESTED)",
                               new PIDRotate(*m_drivebase, 90_deg));
}

void RobotContainer::AddDriveTestButtonsToDashboard() {
#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  frc::SmartDashboard::PutData(
      "Extend Intake Auto", new PivotIntakeAuto(m_intakeDeployment, .5, true));
  frc::SmartDashboard::PutData(
      "Retract Intake Auto",
      new PivotIntakeAuto(m_intakeDeployment, .5, false));

#endif

  frc::SmartDashboard::PutData(
      "Reset encoders",
      new frc2::InstantCommand([this]() { m_drivebase->resetEncoders(); }));

  frc::SmartDashboard::PutData("Reset odometry directly",
                               new frc2::InstantCommand([this]() {
                                 m_drivebase->resetOdometry(frc::Pose2d());
                               }));

  frc::SmartDashboard::PutData(
      "Reset Odometry(via command) to (3,6)",
      new SetRobotOdometry(*m_drivebase, frc::Pose2d(3_m, 6_m, 0_rad)));

  frc::SmartDashboard::PutData(
      "Switch Drive", new frc2::InstantCommand(
                          [this]() { setDriveMode(DriveMode::eSwitched); }));

  frc::SmartDashboard::PutData(
      "Normal Drive",
      new frc2::InstantCommand([this]() { setDriveMode(DriveMode::eNormal); }));
}

void RobotContainer::AddSysIdButtonsToDashboard() {
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

void RobotContainer::AddingNamedScoreOptionsToSelector(
    frc::SendableChooser<frc2::Command *> &score2Selector,
    frc::SendableChooser<frc2::Command *> &score3Selector) {
  const std::list<std::tuple<std::string, std::string>> score2Options{
      {AutonomousScore2Options::amp, AutonomousScore2Options::amp},
      {AutonomousScore2Options::leftOfSpeaker,
       AutonomousScore2Options::leftOfSpeaker},
      {AutonomousScore2Options::inFrontOfSpeaker,
       AutonomousScore2Options::inFrontOfSpeaker},
      {AutonomousScore2Options::rightOfSpeakerAllianceNote,
       AutonomousScore2Options::rightOfSpeakerAllianceNote},
      {AutonomousScore2Options::rightOfSpeakerCenterNote,
       AutonomousScore2Options::rightOfSpeakerCenterNote}};

  const std::list<std::tuple<std::string, std::string>> score3Options{
      {AutonomousScore3Options::amp, AutonomousScore3Options::amp},
      {AutonomousScore3Options::leftOfSpeaker,
       AutonomousScore3Options::leftOfSpeaker},
      {AutonomousScore3Options::inFrontOfSpeakerAmpNote,
       AutonomousScore3Options::inFrontOfSpeakerAmpNote},
      {AutonomousScore3Options::inFrontOfSpeakerCenterNote,
       AutonomousScore3Options::inFrontOfSpeakerCenterNote},
      {AutonomousScore3Options::inFrontOfSpeakerStageNote,
       AutonomousScore3Options::inFrontOfSpeakerStageNote},
      {AutonomousScore3Options::rightOfSpeaker,
       AutonomousScore3Options::rightOfSpeaker}};

  for (auto &[name, text] : score2Options) {
    AddNamedCommandToSelector(score2Selector, name, text);
  }

  for (auto &[name, text] : score3Options) {
    AddNamedCommandToSelector(score3Selector, name, text);
  }
}

void RobotContainer::AddAutoSelectionsToSmartDashboard() {
  AddTeamAndStationSelectorToSmartDashboard();
  AddRobotOverallOperationToSmartDashboard();
  AddScoreOptionsToSmartDashboard();
}

void RobotContainer::AddTeamAndStationSelectorToSmartDashboard() {
  m_PositionAutonomousOptions.SetDefaultOption(
      AutonomousStartingPositions::inFrontOfAmp,
      BuildNamedPrintCommand(AutonomousStartingPositions::inFrontOfAmp));

  AddingNamedStartingPositionsToSelector(m_PositionAutonomousOptions);

  frc::SmartDashboard::PutData("Team and Station Auto Selector",
                               &m_PositionAutonomousOptions);
}

void RobotContainer::AddRobotOverallOperationToSmartDashboard() {
  m_OverallAutonomousOptions.SetDefaultOption(
      AutonomousSelectedOperation::doNothing,
      BuildNamedPrintCommand(AutonomousSelectedOperation::doNothing));

  AddingNamedOverallOperationsToSelector(m_OverallAutonomousOptions);

  frc::SmartDashboard::PutData("Robot Overall Auto Selector",
                               &m_OverallAutonomousOptions);
}

void RobotContainer::AddScoreOptionsToSmartDashboard() {
  m_Score2DestAutonomousOptions.SetDefaultOption(
      "Not Selected", BuildNamedPrintCommand("Not Selected"));
  m_Score3DestAutonomousOptions.SetDefaultOption(
      "Not Selected", BuildNamedPrintCommand("Not Selected"));

  AddingNamedScoreOptionsToSelector(m_Score2DestAutonomousOptions,
                                    m_Score3DestAutonomousOptions);

  frc::SmartDashboard::PutData("Robot Score 2 Option",
                               &m_Score2DestAutonomousOptions);
  frc::SmartDashboard::PutData("Robot Score 3 Option",
                               &m_Score3DestAutonomousOptions);
}

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
frc2::ParallelRaceGroup *RobotContainer::ShootingSequence(bool amp) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<RunShooterTimed>(
      m_shooter, (amp ? ShooterSpeeds::amp : 1.00), 2_s, true));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(IntakeDelay())));

  return new frc2::ParallelRaceGroup(std::move(commands));
}

frc2::SequentialCommandGroup *RobotContainer::IntakeDelay() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<Wait>(0.75_s));
  commands.push_back(
      std::make_unique<RunIntakeTimed>(m_intakeRoller, .5, 1.25_s, false));

  return new frc2::SequentialCommandGroup(std::move(commands));
}
#endif