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

#include <iostream>

#include "Constants.h"
#include "commands/AutoIntakeCells.h"
#include "commands/DecrementLinearActuator.h"
#include "commands/DoASpin.h"
#include "commands/DriveAtPowerForMeters.h"
#include "commands/ExtendIntake.h"
#include "commands/IncrementLinearActuator.h"
#include "commands/IntakePowerCells.h"
#include "commands/RetractIntake.h"
#include "commands/RunOnlyConveyorMotor.h"
#include "commands/RunOnlyConveyorMotorReverse.h"
#include "commands/RunOnlyIntakeMotor.h"
#include "commands/RunOnlyIntakeMotorReverse.h"
#include "commands/RunShootingMotor.h"
#include "commands/ShootWithLimitSwitch.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Intake.h"

#undef GALACTIC_SEARCH_JUST_PRINTS

RobotContainer::RobotContainer() {
  //////////////////////////////////////////
  // Vision processing setup.
  //
  // Adds sliders to the dashboard, allowing for customizing vision processing
  // on the Raspberry Pi's "Vision" program.
  m_visionSettingsHelper.InstallSliders();

  // obtaining info from the network table
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable(NetworkTableNames::kVisionTable);
  pathId = table->GetEntry(NetworkTableNames::kPathID);

  //////////////////////////////////////////
  // Configure the button bindings.
  ConfigureTankDrive();
  ConfigureButtonBindings();
  ConfigureSmartDashboard();
  ConfigureAutoSelection();
}

double RobotContainer::deadband(double num) {
  if (num > OIConstants::DeadBand_LowValue &&
      num < OIConstants::DeadBand_HighValue) {
    return 0;
  }
  return num;
}

TankDrive* RobotContainer::BuildTankDriveCommand() {
  std::function<SpeedScaler::Mode()> speedModeSupplier = [this] {
    // Which mode is signaled by the driver?
    const int turtleTrigger = OIConstants::LogitechGamePad::LeftTriggerButton;
    const int turboTrigger = OIConstants::LogitechGamePad::RightTriggerButton;
    SpeedScaler::Mode result = SpeedScaler::Normal;
    if (driverJoystick.GetRawButton(turtleTrigger)) {
      result = SpeedScaler::Turtle;
    } else if (driverJoystick.GetRawButton(turboTrigger)) {
      result = SpeedScaler::Turbo;
    }

    // If we're changing modes, log it to the console.
    static SpeedScaler::Mode lastMode = SpeedScaler::Mode(-1);
    if (result != lastMode) {
      std::cout << "Driving in " << result << " mode" << std::endl;
    }
    lastMode = result;

    return result;
  };

  SpeedScaler speedScaler{
      speedModeSupplier,                        // Speed mode
      DrivebaseConstants::kNormalSpeedScaling,  // Scaling factor for normal
                                                // mode
      DrivebaseConstants::kTurtleSpeedScaling,  // Scaling factor for turtle
                                                // mode
      DrivebaseConstants::kTurboSpeedScaling    // Scaling factor for turbo
                                                // mode
  };

  return new TankDrive(
      &drivebase,
      [this, speedScaler] {
        double stickValue = -driverJoystick.GetRawAxis(
            OIConstants::LogitechGamePad::RightYAxis);
        return speedScaler(deadband(stickValue));
      },
      [this, speedScaler] {
        double stickValue =
            -driverJoystick.GetRawAxis(OIConstants::LogitechGamePad::LeftYAxis);
        return speedScaler(deadband(stickValue));
      });
}

void RobotContainer::ConfigureTankDrive() {
  std::unique_ptr<TankDrive> cmd(BuildTankDriveCommand());
  drivebase.SetDefaultCommand(*cmd);
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
      frc::XboxController::Button::kY,  // Shoot
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

  static IncrementLinearActuator incrementlinearactuator(&shooter);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kBack,
                                     &incrementlinearactuator);

  static DecrementLinearActuator decrementlinearactuator(&shooter);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kStart,
                                     &decrementlinearactuator);

  //If we end up using buttons for changing shooter angle with the actuator, use kBack to extend, kStart to retract
}

void RobotContainer::ConfigureAutoSelection() {
  m_autoChooser.SetDefaultOption("Do nothing",
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
  m_autoChooser.AddOption("Galactic Search", GalacticSearchAutoPath());
  m_autoChooser.AddOption("Cross the Auto line", new DriveAtPowerForMeters(&drivebase, .5, 3.1_m));

  frc::SmartDashboard::PutData("Auto mode", &m_autoChooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoChooser.GetSelected();
}

void RobotContainer::ConfigureSmartDashboard() {
  frc::SmartDashboard::PutData("Do those spinnin", new DoASpin(&drivebase));
  frc::SmartDashboard::PutData("Run shooter at 100% power",
                               new RunShootingMotor(&shooter, 1.0));
  frc::SmartDashboard::PutData("Run shooter at 95% power",
                               new RunShootingMotor(&shooter, 0.95));
  frc::SmartDashboard::PutData("Run shooter at 90% power",
                               new RunShootingMotor(&shooter, 0.9));
  frc::SmartDashboard::PutData("Run shooter at 80% power",
                               new RunShootingMotor(&shooter, 0.8));
  frc::SmartDashboard::PutData("Run shooter at 70% power",
                               new RunShootingMotor(&shooter, 0.7));

  frc::SmartDashboard::PutData("Shared tank drive", BuildTankDriveCommand());

#ifdef ENABLE_PNEUMATICS
  frc::SmartDashboard::PutData(
      "Extend solenoid",
      new frc2::InstantCommand([this]() { pneumatics.ExtendSolenoid(); },
                               {&pneumatics}));
  frc::SmartDashboard::PutData(
      "Retract solenoid",
      new frc2::InstantCommand([this]() { pneumatics.RetractSolenoid(); },
                               {&pneumatics}));
  frc::SmartDashboard::PutData(
      "Stop solenoid",
      new frc2::InstantCommand([this]() { pneumatics.StopSolenoid(); },
                               {&pneumatics}));
  frc::SmartDashboard::PutData(
      "Toggle solenoid",
      new frc2::InstantCommand([this]() { pneumatics.ToggleSolenoid(); },
                               {&pneumatics}));
  frc::SmartDashboard::PutData("Extend Intake",
                               new ExtendIntake(&pneumatics));
  frc::SmartDashboard::PutData("Retract Intake", 
                               new RetractIntake(&pneumatics));

#endif  // ENABLE_PNEUMATICS

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
  frc::SmartDashboard::PutData(
      "Min Shooter Pos",
      new frc2::InstantCommand([this]() { shooter.SetServoPosition(0.0); },
                               {&shooter}));
  frc::SmartDashboard::PutData(
      "Max Shooter Pos",
      new frc2::InstantCommand([this]() { shooter.SetServoPosition(1.0); },
                               {&shooter}));
  frc::SmartDashboard::PutData(
      "Inc Shooter Pos",
      new frc2::InstantCommand([this]() { shooter.IncrementPosition(); },
                               {&shooter}));
  frc::SmartDashboard::PutData(
      "Dec Shooter Pos",
      new frc2::InstantCommand([this]() { shooter.DecrementPosition(); },
                               {&shooter}));
}

frc2::SequentialCommandGroup* RobotContainer::BuildBouncePathCommand() {
  std::vector<std::unique_ptr<frc2::Command>> bouncePathPieces;

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
frc2::ParallelCommandGroup* RobotContainer::BuildGalacticSearchPath(
    std::string jsonFile1, std::string jsonFile2, std::string jsonFile3,
    std::string jsonFile4, bool includeIntakeOperation) {
  std::vector<std::unique_ptr<frc2::Command>> GalacticPieces;
  GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile(jsonFile1, true))));
  // GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
  //  new DriveAtPowerForMeters(&drivebase, .6, 1_m))));
  GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile(jsonFile2, false))));
  // GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
  //  new DriveAtPowerForMeters(&drivebase, .6, 1_m))));
  GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile(jsonFile3, false))));
  // GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
  //  new DriveAtPowerForMeters(&drivebase, .6, 1_m))));
  GalacticPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile(jsonFile4, false))));

  std::unique_ptr<frc2::Command> galacticPath(
      new frc2::SequentialCommandGroup(std::move(GalacticPieces)));

  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(std::move(galacticPath));

  if (includeIntakeOperation) {
    std::unique_ptr<frc2::Command> intakeAuto(new AutoIntakeCells(&intake));
    commands.push_back(std::move(intakeAuto));
  } else {
    std::unique_ptr<frc2::Command> noIntakeAuto(
        new frc2::PrintCommand("Not running the intake!"));
    commands.push_back(std::move(noIntakeAuto));
  }
  return new frc2::ParallelCommandGroup(std::move(commands));
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

frc2::ConditionalCommand* RobotContainer::BuildBlueAlliancePaths() {
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

frc2::ConditionalCommand* RobotContainer::BuildRedAlliancePaths() {
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
  frc2::Command* blueCombo = BuildBlueAlliancePaths();
  frc2::Command* redCombo = BuildRedAlliancePaths();
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
