// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/Filesystem.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

#include "Constants.h"
#include "commands/AutoIntakeCells.h"
#include "commands/DoASpin.h"
#include "commands/DriveAtPowerForMeters.h"
#include "commands/IntakePowerCells.h"
#include "commands/RunOnlyConveyorMotor.h"
#include "commands/RunOnlyConveyorMotorReverse.h"
#include "commands/RunOnlyIntakeMotor.h"
#include "commands/RunOnlyIntakeMotorReverse.h"
#include "commands/RunShootingMotor.h"
#include "commands/ShootWithLimitSwitch.h"
#include "commands/TankDrive.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Intake.h"

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

  // Set up sliders to allow for customizing vision processing on
  // the Raspberry Pi's "Vision" program.
  m_visionSettingsHelper.InstallSliders();

  // Configure the button bindings
  ConfigureButtonBindings();
  ConfigureSmartDashboard();
  drivebase.SetDefaultCommand(TankDrive(
      &drivebase,
      [this] {
        double stickValue = -driverJoystick.GetRawAxis(
            OIConstants::LogitechGamePad::RightYAxis);
        return deadband(stickValue);
      },
      [this] {
        double stickValue =
            -driverJoystick.GetRawAxis(OIConstants::LogitechGamePad::LeftYAxis);
        return deadband(stickValue);
      }));
  ConfigureAutoSelection();
}

void RobotContainer::RunCommandWhenOperatorButtonIsHeld(
    frc::XboxController::Button buttonId, frc2::Command* command) {
  frc2::JoystickButton(&operatorController, int(buttonId))
      .WhileHeld(command);  // see last year's code
}

void RobotContainer::ConfigureButtonBindings() {
  static RunShootingMotor runshootingmotor(&shooter);
  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kA,
      &runshootingmotor);  // see last year's code

  static IntakePowerCells intakepowercells(&intake);
  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kB,  // Run conveyor and intake
      &intakepowercells);

  static ShootWithLimitSwitch shootwithlimitswitch(&shooter, &intake);
  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kStart,  // Shoot
      &shootwithlimitswitch);

  static RunOnlyIntakeMotor runonlyintakemotor(&intake);
  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kBumperLeft,  // Run intake forwards
      &runonlyintakemotor);

  static RunOnlyIntakeMotorReverse runonlyintakemotorreverse(&intake);
  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kBumperRight,  // Run intake backwards
      &runonlyintakemotorreverse);

  static RunOnlyConveyorMotor runonlyconveyormotor(&intake);
  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kX,  // Run conveyor forwards
      &runonlyconveyormotor);

  static RunOnlyConveyorMotorReverse runonlyconveyormotorreverse(&intake);
  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kY,  // Run conveyor backwards
      &runonlyconveyormotorreverse);
}

void RobotContainer::ConfigureAutoSelection() {
  m_autoChooser.AddDefault("do nothing",
                           new frc2::PrintCommand("I refuse to move."));
  m_autoChooser.AddOption("Go in an S", GenerateRamseteCommandFromPathFile(
                                            "TestingS.wpilib.json", true));
  m_autoChooser.AddOption(
      "Barrel Racing",
      GenerateRamseteCommandFromPathFile("BarrelRacing.wpilib.json", true));
  m_autoChooser.AddOption(
      "Slalom", GenerateRamseteCommandFromPathFile("Slalom.wpilib.json", true));
  std::vector<frc::Translation2d> points{frc::Translation2d(1_m, 0_m),
                                         frc::Translation2d(2_m, 0_m)};
  m_autoChooser.AddOption(
      "Go in a line", GenerateRamseteCommand(
                          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)), points,
                          frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)), true));
  m_autoChooser.AddOption("Bounce Path", BuildBouncePathCommand());

  frc::SmartDashboard::PutData("Auto mode", &m_autoChooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoChooser.GetSelected();
}

void RobotContainer::ConfigureSmartDashboard() {
  frc::SmartDashboard::PutData("Do those spinnin", new DoASpin(&drivebase));
  frc::SmartDashboard::PutData("Run shooter at 75% power",
                               new RunShootingMotor(&shooter));
  frc::SmartDashboard::PutData(
      "Tank Drive", new TankDrive(
                        &drivebase,
                        [this] {
                          double stickValue = driverJoystick.GetRawAxis(
                              OIConstants::LogitechGamePad::RightYAxis);
                          return deadband(stickValue);
                        },
                        [this] {
                          double stickValue = driverJoystick.GetRawAxis(
                              OIConstants::LogitechGamePad::LeftYAxis);
                          return deadband(stickValue);
                        }));

  std::vector<frc::Translation2d> points{frc::Translation2d(1_m, 0_m),
                                         frc::Translation2d(2_m, 0_m)};
  frc::SmartDashboard::PutData(
      "Go in a line", GenerateRamseteCommand(
                          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)), points,
                          frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)), true));
  frc::SmartDashboard::PutData("Go in an S", GenerateRamseteCommandFromPathFile(
                                                 "TestingS.wpilib.json", true));
  frc::SmartDashboard::PutData(
      "Go 9.144 meters at 50%",
      new DriveAtPowerForMeters(&drivebase, .5, 9.144_m));

  frc::SmartDashboard::PutData(
      "Go -9.144 meters at -50%",
      new DriveAtPowerForMeters(&drivebase, -.5, -1_m));

  frc::SmartDashboard::PutData("Simple command group",
                               BuildBouncePathCommand());
}

frc2::SequentialCommandGroup* RobotContainer::BuildBouncePathCommand() {
  std::vector<std::unique_ptr<frc2::Command>> bouncePathPieces;
  bouncePathPieces.push_back(
      std::move(std::unique_ptr<frc2::Command>(new frc2::PrintCommand("foo"))));
  bouncePathPieces.push_back(
      std::move(std::unique_ptr<frc2::Command>(new frc2::PrintCommand("bar"))));
  bouncePathPieces.push_back(
      std::move(std::unique_ptr<frc2::Command>(new frc2::PrintCommand("baz"))));

  bouncePathPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile("Bounce Part1.wpilib.json", true))));
  // bouncePathPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
  //    new DriveAtPowerForMeters(&drivebase, .6, 1_m))));
  bouncePathPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile("Bounce Part2.wpilib.json", false))));
  // bouncePathPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
  //     new DriveAtPowerForMeters(&drivebase, .6, 1_m))));
  bouncePathPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile("Bounce Part3.wpilib.json", false))));
  /*bouncePathPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      new DriveAtPowerForMeters(&drivebase, .6, 1_m))));*/
  bouncePathPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile("Bounce Part4.wpilib.json", false))));

  return new frc2::SequentialCommandGroup(std::move(bouncePathPieces));
}

double RobotContainer::deadband(double num) {
  if (num > OIConstants::DeadBand_LowValue &&
      num < OIConstants::DeadBand_HighValue) {
    return 0;
  }
  return num;
}

frc2::SequentialCommandGroup* RobotContainer::GenerateRamseteCommand(
    const frc::Pose2d& start,
    const std::vector<frc::Translation2d>& interiorWaypoints,
    const frc::Pose2d& end, bool resetTelemetryAtStart) {
  using namespace DrivebaseConstants;
  frc::TrajectoryConfig config = buildConfig();

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      start, interiorWaypoints, end, config);

  frc::SimpleMotorFeedforward<units::meter> feedForward(
      ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

  return createRams(trajectory, resetTelemetryAtStart);
}

frc2::SequentialCommandGroup*
RobotContainer::GenerateRamseteCommandFromPathFile(std::string filename,
                                                   bool resetTelemetryAtStart) {
  frc::Trajectory trajectory = loadTraj(filename);

  return createRams(trajectory, resetTelemetryAtStart);
}

frc::TrajectoryConfig RobotContainer::buildConfig() {
  using namespace DrivebaseConstants;

  frc::SimpleMotorFeedforward<units::meter> feedForward(
      ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
  frc::DifferentialDriveVoltageConstraint voltageConstraints(
      feedForward, kDriveKinematics, 10_V);
  frc::TrajectoryConfig config(kMaxSpeed, kMaxAcceleration);

  config.SetKinematics(kDriveKinematics);

  config.AddConstraint(voltageConstraints);
  return config;
}

frc2::SequentialCommandGroup* RobotContainer::createRams(
    frc::Trajectory trajectory, bool resetTelemetryAtStart) {
  using namespace DrivebaseConstants;

  frc::SimpleMotorFeedforward<units::meter> feedForward(
      ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
  frc2::RamseteCommand ramseteCommand(
      trajectory, [this]() { return drivebase.GetPose(); },
      frc::RamseteController{kRamseteB, kRamseteZeta}, feedForward,
      kDriveKinematics, [this]() { return drivebase.GetWheelSpeeds(); },
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      [this](auto left, auto right) { drivebase.TankDriveVolts(left, right); },
      {&drivebase});

  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this, resetTelemetryAtStart, trajectory] {
        if (resetTelemetryAtStart) {
          drivebase.ResetOdometry(trajectory.InitialPose());
        }
      }),
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { drivebase.TankDriveVolts(0_V, 0_V); }, {}));
}

frc::Trajectory RobotContainer::loadTraj(std::string jsonFile) {
  wpi::SmallString<64> deployDirectory;
  frc::filesystem::GetDeployDirectory(deployDirectory);
  wpi::sys::path::append(deployDirectory, "paths");
  wpi::sys::path::append(deployDirectory, jsonFile);

  frc::Trajectory trajectory =
      frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
  return trajectory;
}
