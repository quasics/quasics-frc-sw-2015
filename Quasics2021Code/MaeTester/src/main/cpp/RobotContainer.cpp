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
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
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

#undef GALACTIC_SEARCH_JUST_PRINTS

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

  // obtaining info from the network table
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable(NetworkTableNames::kVisionTable);
  pathId = table->GetEntry(NetworkTableNames::kPathID);
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
  m_autoChooser.AddOption(
      "Galactic Search Red A",
      BuildGalacticSearchPath(
          "GSearchARed Part1.wpilib.json", "GSearchARed Part2.wpilib.json",
          "GSearchARed Part3.wpilib.json", "GSearchARed Part4.wpilib.json"));
  m_autoChooser.AddOption(
      "Galactic Search Blue A",
      BuildGalacticSearchPath(
          "GSearchABlue Part1.wpilib.json", "GSearchABlue Part2.wpilib.json",
          "GSearchABlue Part3.wpilib.json", "GSearchABlue Part4.wpilib.json"));
  m_autoChooser.AddOption(
      "Galactic Search Red B",
      BuildGalacticSearchPath(
          "GSearchBRed Part1.wpilib.json", "GSearchBRed Part2.wpilib.json",
          "GSearchBRed Part3.wpilib.json", "GSearchBRed Part4.wpilib.json"));
  m_autoChooser.AddOption(
      "Galactic Search Blue B",
      BuildGalacticSearchPath(
          "GSearchBBlue Part1.wpilib.json", "GSearchBBlue Part2.wpilib.json",
          "GSearchBBlue Part3.wpilib.json", "GSearchBBlue Part4.wpilib.json"));
  m_autoChooser.AddOption("Galactic Search Drive Only",
                          GalacticSearchAutoPath());
  m_autoChooser.AddOption("Galactic Search", GalacticSearchFullAuto());

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

// This builds a Galactic Search Path from 4 json files
frc2::SequentialCommandGroup* RobotContainer::BuildGalacticSearchPath(
    std::string jsonFile1, std::string jsonFile2, std::string jsonFile3,
    std::string jsonFile4) {
  std::vector<std::unique_ptr<frc2::Command>> GalacticPieces;
  GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile(jsonFile1, true))));
  GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      new DriveAtPowerForMeters(&drivebase, .6, 1_m))));
  GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile(jsonFile2, true))));
  GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      new DriveAtPowerForMeters(&drivebase, .6, 1_m))));
  GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile(jsonFile3, true))));
  GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      new DriveAtPowerForMeters(&drivebase, .6, 1_m))));
  GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile(jsonFile4, true))));

  return new frc2::SequentialCommandGroup(std::move(GalacticPieces));
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

/*These Functions evaluate, based on the path
ID, whether an error has taken place, what alliance
we're on, and what path we're following*/

bool RobotContainer::RecognizeError() {
  double path = pathId.GetDouble(-1);
  if (path < 1) {
    return true;
  }
  return false;
}

bool RobotContainer::RecognizePathA() {
  double path = pathId.GetDouble(-1);
  if (path < 3) {
    return true;
  }
  return false;
}

bool RobotContainer::RecognizeBlueAlliance() {
  double path = pathId.GetDouble(-1);
  int correctedPath = path;
  if (correctedPath % 2 == 0) {
    return true;
  }
  return false;
}

/*These Conditional Commands evaluate first whether there is
an error, then which alliance we're on, and then, based on what
path we're following, builds the correct Galactic Search Path
for the robot to follow.*/

frc2::ConditionalCommand* RobotContainer::BuildBlueAlliancePath() {
#ifndef GALACTIC_SEARCH_JUST_PRINTS
  frc2::Command* blueB = BuildGalacticSearchPath(
      "GSearchBBlue Part1.wpilib.json", "GSearchBBlue Part2.wpilib.json",
      "GSearchBBlue Part3.wpilib.json", "GSearchBBlue Part4.wpilib.json");
  frc2::Command* blueA = BuildGalacticSearchPath(
      "GSearchABlue Part1.wpilib.json", "GSearchABlue Part2.wpilib.json",
      "GSearchABlue Part3.wpilib.json", "GSearchABlue Part4.wpilib.json");
  return new frc2::ConditionalCommand(std::unique_ptr<frc2::Command>(blueA),
                                      std::unique_ptr<frc2::Command>(blueB),
                                      [this] { return RecognizePathA(); });
#else
  return new frc2::ConditionalCommand(
      std::unique_ptr<frc2::Command>(
          new frc2::PrintCommand("Alliance: B, Path: A")),
      std::unique_ptr<frc2::Command>(
          new frc2::PrintCommand("Alliance: B, Path: B")),
      [this] { return RecognizePathA(); });
#endif
}

frc2::ConditionalCommand* RobotContainer::BuildRedAlliancePath() {
#ifndef GALACTIC_SEARCH_JUST_PRINTS
  frc2::Command* redA = BuildGalacticSearchPath(
      "GSearchARed Part1.wpilib.json", "GSearchARed Part2.wpilib.json",
      "GSearchARed Part3.wpilib.json", "GSearchARed Part4.wpilib.json");
  frc2::Command* redB = BuildGalacticSearchPath(
      "GSearchBRed Part1.wpilib.json", "GSearchBRed Part2.wpilib.json",
      "GSearchBRed Part3.wpilib.json", "GSearchBRed Part4.wpilib.json");
  return new frc2::ConditionalCommand(std::unique_ptr<frc2::Command>(redA),
                                      std::unique_ptr<frc2::Command>(redB),
                                      [this] { return RecognizePathA(); });
#else
  return new frc2::ConditionalCommand(
      std::unique_ptr<frc2::Command>(
          new frc2::PrintCommand("Alliance: R, Path: A")),
      std::unique_ptr<frc2::Command>(
          new frc2::PrintCommand("Alliance: R, Path: B")),
      [this] { return RecognizePathA(); });
#endif
}

frc2::ConditionalCommand* RobotContainer::ChooseWhichAlliance() {
  frc2::Command* blueCombo = BuildBlueAlliancePath();
  frc2::Command* redCombo = BuildRedAlliancePath();
  return new frc2::ConditionalCommand(
      std::unique_ptr<frc2::Command>(blueCombo),
      std::unique_ptr<frc2::Command>(redCombo),
      [this] { return RecognizeBlueAlliance(); });
}

frc2::ConditionalCommand* RobotContainer::GalacticSearchAutoPath() {
  frc2::Command* AllianceCombo = ChooseWhichAlliance();
  return new frc2::ConditionalCommand(
      std::unique_ptr<frc2::Command>(
          new frc2::PrintCommand("An error occured in path recognition")),
      std::unique_ptr<frc2::Command>(AllianceCombo),
      [this] { return RecognizeError(); });
}

frc2::ParallelCommandGroup* RobotContainer::GalacticSearchFullAuto() {
  frc2::Command* DrivebaseAuto = GalacticSearchAutoPath();
  frc2::Command* IntakeAuto = new AutoIntakeCells(&intake);
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(DrivebaseAuto)));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(IntakeAuto)));
  return new frc2::ParallelCommandGroup(std::move(commands));
}