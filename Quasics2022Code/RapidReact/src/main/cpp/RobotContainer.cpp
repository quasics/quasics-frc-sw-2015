// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include "Constants.h"
#include "TrajectoryCommandGenerator.h"
#include "commands/BreathingAllianceLights.h"
#include "commands/BreathingLights.h"
#include "commands/DriveAtPowerForMeters.h"
#include "commands/ExtendClimber.h"
#include "commands/ExtendIntake.h"
#include "commands/ExtendIntakeAuto.h"
#include "commands/MoveRobotTestCommand.h"
#include "commands/RetractClimber.h"
#include "commands/RetractIntake.h"
#include "commands/RetractIntakeAtSpeedForTime.h"
#include "commands/RunConveyorAtSpeedForTime.h"
#include "commands/RunIntakeAtSpeed.h"
#include "commands/RunShooterAtSpeed.h"
#include "commands/SetLightsToColor.h"
#include "commands/ShootForTime.h"
#include "commands/TankDrive.h"

RobotContainer::RobotContainer()
    : m_trajectoryGenerator(
          // Drive base being controlled
          &m_drivebase,
          // Drive profile data
          {
              DriverConstants::ks,  // kS
              DriverConstants::kv,  // kV
              DriverConstants::ka   // kA
          },
          // PID configuration values
          {
              DriverConstants::kp,  // kP
              DriverConstants::ki,  // kI
              DriverConstants::kd   // kD
          }) {
  TankDrive tankDrive{
      &m_drivebase,
      [this] {
        // Figure out scaling for the current driving mode
        const double scalingFactor = GetDriveSpeedScalingFactor();

        // Get the "base" speed value from the correct joystick
        double joystickValue;
        if (!isSwitched) {
          joystickValue =
              -1 * m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        } else {
          joystickValue =
              -1 * m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
        }

        // Return scaled speed.
        return scalingFactor * joystickValue;
      },
      [this] {
        // Figure out scaling for the current driving mode
        const double scalingFactor = GetDriveSpeedScalingFactor();

        // Get the "base" speed value from the correct joystick
        double joystickValue;
        if (!isSwitched) {
          joystickValue =
              -1 * m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
        } else {
          joystickValue =
              +1 * m_driverStick.GetRawAxis(
                       OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
        }

        // Return scaled speed.
        return scalingFactor * joystickValue;
      }};

  // Initialize all of your commands and subsystems here
  m_drivebase.SetDefaultCommand(tankDrive);

  // Configure the button bindings
  ConfigureControllerButtonBindings();

  // Populate smart dashboard
  AddAutonomousCommandsToSmartDashboard();
  AddTestButtonsToSmartDashboard();
}

double RobotContainer::GetDriveSpeedScalingFactor() {
  const bool isTurbo = m_driverStick.GetRawButton(
      OperatorInterface::LogitechGamePad::LEFTSHOULDER);
  const bool isTurtle = m_driverStick.GetRawButton(
      OperatorInterface::LogitechGamePad::RIGHTSHOULDER);

  if (isTurbo) {
    return TURBO_MODE_SPEED_SCALING;
  } else if (isTurtle) {
    return TURTLE_MODE_SPEED_SCALING;
  } else {
    return NORMAL_MODE_SPEED_SCALING;
  }
}

// Configure your button bindings here
void RobotContainer::RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                                      frc2::Command* command) {
  frc2::JoystickButton(&driverJoystick, logitechButtonId).WhileHeld(command);
}

void RobotContainer::RunCommandWhenOperatorButtonIsHeld(
    int buttonId, frc2::Command* command) {
  frc2::JoystickButton(&operatorController, buttonId).WhileHeld(command);
}

// TODO: Configure other button bindings on driver and operator controllers.

void RobotContainer::ConfigureControllerButtonBindings() {
  static ExtendClimber extendClimber(&m_climber);
  static RetractClimber retractClimber(&m_climber);
  static RunIntakeAtSpeed runIntakeForward(&m_intake, 0.8);
  static RunIntakeAtSpeed runIntakeBackward(&m_intake, -0.6);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kY,
                                     &extendClimber);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kA,
                                     &retractClimber);
  RunCommandWhenDriverButtonIsHeld(
      OperatorInterface::LogitechGamePad::LEFTSHOULDER, &runIntakeForward);
  RunCommandWhenDriverButtonIsHeld(
      OperatorInterface::LogitechGamePad::RIGHTSHOULDER, &runIntakeBackward);
}
// Note: 0.65 seems to be reasonable power for the high goal.
void RobotContainer::AddTestButtonsToSmartDashboard() {
  // Basic drive base commands/tests
  frc::SmartDashboard::PutData(
      "Braking mode",
      new frc2::InstantCommand([this]() { m_drivebase.SetBrakingMode(true); },
                               {&m_drivebase}));
  frc::SmartDashboard::PutData(
      "Coasting mode",
      new frc2::InstantCommand([this]() { m_drivebase.SetBrakingMode(false); },
                               {&m_drivebase}));
  frc::SmartDashboard::PutData("Test Button Do Something",
                               new MoveRobotTestCommand(&m_drivebase, 0.2));
  frc::SmartDashboard::PutData(
      "Drivebase: 20m at 80%",
      new DriveAtPowerForMeters(&m_drivebase, 0.8, 20_m));

  // Shooter commands
  frc::SmartDashboard::PutData("Shoot @ 65%",
                               new RunShooterAtSpeed(&m_shooter, 0.65));
  frc::SmartDashboard::PutData("Shoot 50%",
                               new RunShooterAtSpeed(&m_shooter, 0.5));

  // Intake commands
  frc::SmartDashboard::PutData("Intake: 80% forward",
                               new RunIntakeAtSpeed(&m_intake, 0.80));
  frc::SmartDashboard::PutData("Intake: 30% backward",
                               new RunIntakeAtSpeed(&m_intake, -0.3));

  // Intake deployment commands
  frc::SmartDashboard::PutData(
      "Extend Intake at 20% speed",
      new ExtendIntake(&m_intakeDeployment,
                       0.2));  // seems like 20% is a safe speed for the intake
                               // not to over rotate
  frc::SmartDashboard::PutData("Retract Intake at 30%",
                               new RetractIntake(&m_intakeDeployment, -0.3));
  frc::SmartDashboard::PutData("Extend Intake auto",
                               new ExtendIntakeAuto(&m_intakeDeployment, -0.3));

  // Conveyor commands
  frc::SmartDashboard::PutData(
      "Conveyor: 40% forward for 2 seconds",
      new RunConveyorAtSpeedForTime(&m_conveyor, 0.4, 2_s));
  frc::SmartDashboard::PutData(
      "Conveyor: 30% backward for 2 seconds",
      new RunConveyorAtSpeedForTime(&m_conveyor, -0.3, 2_s));
  frc::SmartDashboard::PutData(
      "Conveyor: 20% forward for 2 seconds",
      new RunConveyorAtSpeedForTime(&m_conveyor, 0.2, 2_s));

  frc::SmartDashboard::PutData(
      "Conveyor at 60%. infinite time",
      new RunConveyorAtSpeedForTime(&m_conveyor, 0.6, 100_s));

  // Climber commands
  frc::SmartDashboard::PutData("Extend Climber", new ExtendClimber(&m_climber));
  frc::SmartDashboard::PutData("Retract Climber",
                               new RetractClimber(&m_climber));

  // Lighting commands
  frc::SmartDashboard::PutData(
      "Set All ligths to Red",
      new SetLightsToColor(&m_lighting, Lighting::StockColor::Red));
  frc::SmartDashboard::PutData(
      "Breathing Lights", new BreathingLights(&m_lighting, 0, 255, 0, 0.75));
  frc::SmartDashboard::PutData("Alliance Breathing Lights",
                               new BreathingAllianceLights(&m_lighting, 0.75));

  // Path following commands
  AddTestTrajectoryCommandsToSmartDashboard();
}

void RobotContainer::AddTestTrajectoryCommandsToSmartDashboard() {
  frc::SmartDashboard::PutData(
      "MaeTestVerySlow",
      m_trajectoryGenerator.GenerateCommandFromPathWeaverFile(
          "MaeTestVerySlow.wpilib.json",
          TrajectoryCommandGenerator::TelemetryHandling::
              ResetTelemetryAtStart));

  frc::SmartDashboard::PutData(
      "MaeStartAndGrabTopComeBack",
      m_trajectoryGenerator.GenerateCommandFromPathWeaverFile(
          "MaeStartAndGrabTopComeBack.wpilib.json",
          TrajectoryCommandGenerator::TelemetryHandling::
              ResetTelemetryAtStart));

  frc::SmartDashboard::PutData(
      "Trajectory: Start = 0,0 -> End = 1,1 -> intermediate = 0,1",
      m_trajectoryGenerator.GenerateCommand(
          TrajectoryCommandGenerator::SpeedProfile{
              0.2_mps, 0.4_mps_sq},  // changed values from 0.5 and 0.8 to
                                     // 0.2 and 0.4
          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
          std::vector<frc::Translation2d>{frc::Translation2d(0_m, 1_m)},
          frc::Pose2d(1_m, 1_m, frc::Rotation2d(0_deg)),
          TrajectoryCommandGenerator::ResetTelemetryAtStart));

  frc::SmartDashboard::PutData(
      "Trajectory: Start = 0,0 -> End = 1,0 -> intermediate = 1,0",
      m_trajectoryGenerator.GenerateCommand(
          TrajectoryCommandGenerator::SpeedProfile{
              0.2_mps,
              0.4_mps_sq},  // changed values from 0.5 and 0.8 to 0.2 and 0.4
          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
          std::vector<frc::Translation2d>{frc::Translation2d(1_m, 0_m)},
          frc::Pose2d(1_m, 0_m, frc::Rotation2d(0_deg)),
          TrajectoryCommandGenerator::ResetTelemetryAtStart));

  frc::SmartDashboard::PutData(
      "Trajectory: Start = 0,0 -> End = 0,0 -> intermediate = (1.5,0), "
      "(1.5,1.5), "
      "(0,1.5)",
      m_trajectoryGenerator.GenerateCommand(
          TrajectoryCommandGenerator::SpeedProfile{
              0.2_mps,
              0.4_mps_sq},  // changed values from 0.5 and 0.8 to 0.2 and 0.4
          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
          std::vector<frc::Translation2d>{frc::Translation2d(1.5_m, 0_m),
                                          frc::Translation2d(1.5_m, 1.5_m),
                                          frc::Translation2d(0_m, 1.5_m)},
          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
          TrajectoryCommandGenerator::ResetTelemetryAtStart));
}

void RobotContainer::AddAutonomousCommandsToSmartDashboard() {
  m_autonomousOptions.SetDefaultOption(
      "Do Nothing", new frc2::PrintCommand("I decline to do anything."));

  m_autonomousOptions.AddOption(
      "Move Forward 1m at 20% power",
      new DriveAtPowerForMeters(&m_drivebase, 0.2, 1_m));

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

// sequence for the autonomous part
frc2::SequentialCommandGroup*
RobotContainer::BuildAutonomousTrajectoryCommand() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(BallsToShoot(1))));
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(DrivingAndPickingUpBalls())));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      m_trajectoryGenerator.GenerateCommandFromPathWeaverFile(
          "PickUpTwoBallSallyPart2.wpilib.json",
          TrajectoryCommandGenerator::TelemetryHandling::
              ResetTelemetryAtStart))));
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(BallsToShoot(2))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::ParallelRaceGroup* RobotContainer::DrivingAndPickingUpBalls() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      m_trajectoryGenerator.GenerateCommandFromPathWeaverFile(
          "PickUpTwoBallSallyPart1.wpilib.json",
          TrajectoryCommandGenerator::TelemetryHandling::
              ResetTelemetryAtStart))));
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(PickingUpBalls())));
  commands.push_back(std::make_unique<RetractIntakeAtSpeedForTime>(
      &m_intakeDeployment, 0.7, 0.3_s));

  return new frc2::ParallelRaceGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::PickingUpBalls() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<ExtendIntake>(&m_intakeDeployment, 0.7));
  commands.push_back(std::make_unique<RunIntakeAtSpeed>(&m_intake, 0.7));
  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::SequentialCommandGroup* RobotContainer::BallsToShoot(int amountBalls) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  int count = amountBalls;
  while (count >= 0) {
    commands.push_back(
        std::move(std::unique_ptr<frc2::Command>(BuildShootBallSequence())));
    count--;
  }
  return new frc2::SequentialCommandGroup(std::move(commands));
}
frc2::ParallelCommandGroup* RobotContainer::BuildShootBallSequence() {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::make_unique<ShootForTime>(&m_shooter, 0.60, 1_s));
  commands.push_back(
      std::make_unique<RunConveyorAtSpeedForTime>(&m_conveyor, 0.8, 1_s));
  return new frc2::ParallelCommandGroup(std::move(commands));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autonomousOptions.GetSelected();
}
