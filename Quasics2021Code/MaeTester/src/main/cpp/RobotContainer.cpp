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
#include <frc2/command/ParallelRaceGroup.h>
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
#include "commands/ColorLights.h"
#include "commands/DecrementLinearActuator.h"
#include "commands/DelayForTime.h"
#include "commands/DoASpin.h"
#include "commands/DriveAtPowerForMeters.h"
#include "commands/IncrementLinearActuator.h"
#include "commands/IntakePowerCells.h"
#include "commands/RunConveyorUntilBallLoads.h"
#include "commands/RunOnlyConveyorMotor.h"
#include "commands/RunOnlyConveyorMotorReverse.h"
#include "commands/RunOnlyIntakeMotor.h"
#include "commands/RunOnlyIntakeMotorReverse.h"
#include "commands/RunShootingMotor.h"
#include "commands/ShootForTime.h"
#include "commands/ShootWithLimitSwitch.h"
#include "commands/TimedConveyor.h"
#include "commands/TimedIntake.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Intake.h"
#include "subsystems/Lights.h"

///////////////////////////////////////////////////////////////////////////////
// Conditional compilation flags start here.

// DEFINE this to disable "turbo" mode (e.g., with untrusted drivers :-).
#define DISABLE_TURBO_MODE

// Conditional compilation flags end here.
///////////////////////////////////////////////////////////////////////////////

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

  // Configure the key subsystems.
  ConfigureTankDrive();
  ConfigureLights();

  std::unique_ptr<TriggerDrivenShootingCommand> shooterCmd(
      BuildShootingCommand());
  shooter.SetDefaultCommand(*shooterCmd);

  //////////////////////////////////////////
  // Configure the button bindings.
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

TriggerDrivenShootingCommand* RobotContainer::BuildShootingCommand() {
  std::function<bool()> runHighSpeedSupplier = [this] {
    if (operatorController.GetTriggerAxis(frc::GenericHID::kLeftHand) >= 0.5) {
      return true;
    }
    return false;
  };
  std::function<bool()> runLowSpeedSupplier = [this] {
    if (operatorController.GetTriggerAxis(frc::GenericHID::kRightHand) >= 0.5) {
      return true;
    }
    return false;
  };

  return new TriggerDrivenShootingCommand(
      &shooter, 1.0, 0.8, runHighSpeedSupplier, runLowSpeedSupplier);
}

TankDrive* RobotContainer::BuildTankDriveCommand() {
#if defined(DISABLE_TURBO_MODE)
  constexpr int turtleTrigger =
      OIConstants::LogitechGamePad::RightTriggerButton;
  constexpr int turboTrigger = OIConstants::LogitechGamePad::InvalidButton;
#else
  constexpr int turtleTrigger = OIConstants::LogitechGamePad::LeftTriggerButton;
  constexpr int turboTrigger = OIConstants::LogitechGamePad::RightTriggerButton;
#endif  // DISABLE_TURBO_MODE

  std::function<SpeedScaler::Mode()> speedModeSupplier = [this, turtleTrigger,
                                                          turboTrigger] {
    SpeedScaler::Mode result = SpeedScaler::Normal;
    if (turtleTrigger != OIConstants::LogitechGamePad::InvalidButton &&
        driverJoystick.GetRawButton(turtleTrigger)) {
      result = SpeedScaler::Turtle;
    } else if (turboTrigger != OIConstants::LogitechGamePad::InvalidButton &&
               driverJoystick.GetRawButton(turboTrigger)) {
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

ColorLights* RobotContainer::BuildColorLightsCommand() {
  return new ColorLights(&lights, 0, 255, 0);
}

void RobotContainer::ConfigureTankDrive() {
  std::unique_ptr<TankDrive> cmd(BuildTankDriveCommand());
  drivebase.SetDefaultCommand(*cmd);
}

void RobotContainer::ConfigureLights() {
  std::unique_ptr<ColorLights> cmd(BuildColorLightsCommand());
  lights.SetDefaultCommand(*cmd);
}

void RobotContainer::RunCommandWhenOperatorButtonIsHeld(
    frc::XboxController::Button buttonId, frc2::Command* command) {
  frc2::JoystickButton(&operatorController, int(buttonId))
      .WhileHeld(command);  // see last year's code
}

void RobotContainer::RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                                      frc2::Command* command) {
  frc2::JoystickButton(&driverJoystick, logitechButtonId).WhileHeld(command);
}

void RobotContainer::ConfigureButtonBindings() {
  // Run the shooter (high/low speed).
  static RunShootingMotor runShooterFullSpeed(&shooter, 1.0);
  static RunShootingMotor runShooterLowSpeed(&shooter, 0.8);

  // Run the intake (forward/back)
  static RunOnlyIntakeMotor intakeForwardCommand(&intake);
  static RunOnlyIntakeMotorReverse intakeReverseCommand(&intake);

  // Run the conveyor (forward/back).
  static RunOnlyConveyorMotor conveyorForwardCommand(&intake);
  static RunOnlyConveyorMotorReverse conveyorBackwardCommand(&intake);

  // Gross adjustments to the shooting angle.
  static std::unique_ptr<frc2::Command> shooterToMinimumCommandPtr{
      new frc2::InstantCommand([this]() { shooter.SetServoPosition(0.0); },
                               {&shooter})};
  static std::unique_ptr<frc2::Command> shooterToMaximumCommandPtr{
      new frc2::InstantCommand([this]() { shooter.SetServoPosition(1.0); },
                               {&shooter})};

  // Fine adjustments to the shooting angle
  static IncrementLinearActuator incrementShootingAngle(&shooter);
  static DecrementLinearActuator decrementShootingAngle(&shooter);

#ifdef ENABLE_PNEUMATICS
  // Moving the intake in/out (via pneumatics).
  static std::unique_ptr<frc2::Command> openIntakeCommandPtr{
      new frc2::InstantCommand([this]() { pneumatics.ExtendSolenoid(); },
                               {&pneumatics})};
  static std::unique_ptr<frc2::Command> closeIntakeCommandPtr{
      new frc2::InstantCommand([this]() { pneumatics.RetractSolenoid(); },
                               {&pneumatics})};
#endif  // ENABLE_PNEUMATICS

  // Other commands.
  static IntakePowerCells intakepowercells(&intake);
  static ShootWithLimitSwitch shootwithlimitswitch(&shooter, &intake);

#ifdef BROKEN_BY_JOSH
  // Note: Shot speed value is needed.
  static SetShotSpeed SetShotSpeed(&shooter, &fastShotSpeed);
#endif  // BROKEN_BY_JOSH

#define ENABLE_BINDINGS_FOR_DEMO
#ifdef ENABLE_BINDINGS_FOR_DEMO
  // Bindings defined by Matt Healy and Meg Gilmore on 12Aug2021,
  // for use at the demo on 14Aug2021.
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kA,
                                     &conveyorBackwardCommand);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kB,
                                     &conveyorForwardCommand);

  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kBumperLeft,
                                     shooterToMaximumCommandPtr.get());
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kBumperRight,
                                     shooterToMinimumCommandPtr.get());

#ifdef ENABLE_PNEUMATICS
  RunCommandWhenDriverButtonIsHeld(OIConstants::LogitechGamePad::BackButton,
                                   closeIntakeCommandPtr.get());
  RunCommandWhenDriverButtonIsHeld(OIConstants::LogitechGamePad::StartButton,
                                   openIntakeCommandPtr.get());
#endif  // ENABLE_PNEUMATICS

  RunCommandWhenDriverButtonIsHeld(OIConstants::LogitechGamePad::LeftShoulder,
                                   &intakeForwardCommand);
  RunCommandWhenDriverButtonIsHeld(OIConstants::LogitechGamePad::RightShoulder,
                                   &intakeReverseCommand);

#else
  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kA,
      &runShooterFullSpeed);  // see last year's code

  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kY,  // Shoot
                                     &shootwithlimitswitch);

  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kB,  // Run conveyor forwards
      &conveyorForwardCommand);

  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kX,  // Run conveyor Backwards
      &conveyorBackwardCommand);

  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kStart,
                                     &incrementShootingAngle);

  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kBack,
                                     &decrementShootingAngle);

#ifdef BROKEN_BY_JOSH
  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kBumperRight,  // Fast Shot Speed
      &setshotspeed);
  );
#endif  // BROKEN_BY_JOSH

  /*
  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kB,  // Run conveyor and intake
      &intakepowercells);

  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kBumperLeft,  // Run intake forwards
      &intakeForwardCommand);

  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kBumperRight,  // Run intake backwards
      &intakeReverseCommand);
  */

  // If we end up using buttons for changing shooter angle with the actuator,
  // use kBack to extend, kStart to retract That makes no sense? Back and extend
  // are basically antonyms. so are start and retract.

#endif  // ENABLE_BINDINGS_FOR_DEMO
}

void RobotContainer::ConfigureAutoSelection() {
  const double kDistanceToDriveMeters = 0.25;
  const double kDrivingSpeedPercent = 0.5;
  const double kShootingTimeSeconds = 7;

  m_autoChooser.SetDefaultOption("Do nothing",
                                 new frc2::PrintCommand("I refuse to move."));
  m_autoChooser.AddOption("Move forward 3 ft",
                          new DriveAtPowerForMeters(&drivebase, .5, 1_m));
  m_autoChooser.AddOption("Move backwards 3 ft",
                          new DriveAtPowerForMeters(&drivebase, .5, -1_m));
  m_autoChooser.AddOption(
      "Shoot 3/Wait 2",
      BuildShootAndMoveSequence(3_s /*conveyor on (s)*/, 2_s /*wait (s)*/,
                                units::second_t(kShootingTimeSeconds),
                                kDrivingSpeedPercent, kDistanceToDriveMeters));

  if (false) {
    // Commands used to during the "At Home" game in 2021.
    m_autoChooser.AddOption("Go in an S", GenerateRamseteCommandFromPathFile(
                                              "TestingS.wpilib.json", true));
    m_autoChooser.AddOption(
        "Barrel Racing",
        GenerateRamseteCommandFromPathFile("BarrelRacing.wpilib.json", true));
    m_autoChooser.AddOption("Slalom", GenerateRamseteCommandFromPathFile(
                                          "Slalom.wpilib.json", true));
    m_autoChooser.AddOption("Bounce Path", BuildBouncePathCommand());

    m_autoChooser.AddOption(
        "Galactic Search Red A",
        BuildGalacticSearchPath(
            "GSearchARed Part1.wpilib.json", "GSearchARed Part2.wpilib.json",
            "GSearchARed Part3.wpilib.json", "GSearchARed Part4.wpilib.json"));

    m_autoChooser.AddOption(
        "Galactic Search Blue A",
        BuildGalacticSearchPath("GSearchABlue Part1.wpilib.json",
                                "GSearchABlue Part2.wpilib.json",
                                "GSearchABlue Part3.wpilib.json",
                                "GSearchABlue Part4.wpilib.json"));

    m_autoChooser.AddOption(
        "Galactic Search Red B",
        BuildGalacticSearchPath(
            "GSearchBRed Part1.wpilib.json", "GSearchBRed Part2.wpilib.json",
            "GSearchBRed Part3.wpilib.json", "GSearchBRed Part4.wpilib.json"));

    m_autoChooser.AddOption(
        "Galactic Search Blue B",
        BuildGalacticSearchPath("GSearchBBlue Part1.wpilib.json",
                                "GSearchBBlue Part2.wpilib.json",
                                "GSearchBBlue Part3.wpilib.json",
                                "GSearchBBlue Part4.wpilib.json"));

    m_autoChooser.AddOption("Galactic Search Drive Only",
                            GalacticSearchAutoPath());
    m_autoChooser.AddOption("Galactic Search", GalacticSearchAutoPath());
  }
  if (false) {
    // Commands used to test trajectory stuff, etc.
    std::vector<frc::Translation2d> points{frc::Translation2d(1_m, 0_m),
                                           frc::Translation2d(2_m, 0_m)};
    m_autoChooser.AddOption(
        "Go in a line",
        GenerateRamseteCommand(
            frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)), points,
            frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)), true));
    m_autoChooser.AddOption("Cross the Auto line",
                            new DriveAtPowerForMeters(&drivebase, .5, 3.1_m));
  }
  frc::SmartDashboard::PutData("Auto mode", &m_autoChooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoChooser.GetSelected();
}

void RobotContainer::ConfigureSmartDashboard() {
  if (false) {
    // Lighting controls (under construction)
    frc::SmartDashboard::PutData("Red Light",
                                 new ColorLights(&lights, 255, 0, 0));
    frc::SmartDashboard::PutData("Green Light",
                                 new ColorLights(&lights, 0, 255, 0));
    frc::SmartDashboard::PutData("Blue Light",
                                 new ColorLights(&lights, 0, 0, 255));
    frc::SmartDashboard::PutData("Lights Out",
                                 new ColorLights(&lights, 0, 0, 0));
  }
  // Adjustment for Robot specifications(BuildShootAndMoveSequence).
  //
  // To adjust power of shooting motor, change the value for
  // "kShooterSpeedPercent" in the "BuildConveyorAndShootingSequence()"
  // function.
  const double kDistanceToDriveMeters = 0.25;
  const double kDrivingSpeedPercent = 0.5;
  const double kShootingTimeSeconds = 13;

  frc::SmartDashboard::PutData(
      "Shoot 1/Wait 1",
      BuildShootAndMoveSequence(1_s /*conveyor on (s)*/, 1_s /*wait (s)*/,
                                units::second_t(kShootingTimeSeconds),
                                kDrivingSpeedPercent, kDistanceToDriveMeters));

  frc::SmartDashboard::PutData(
      "Shoot 4/Wait 1",
      BuildShootAndMoveSequence(4_s /*conveyor on (s)*/, 1_s /*wait (s)*/,
                                units::second_t(kShootingTimeSeconds),
                                kDrivingSpeedPercent, kDistanceToDriveMeters));

  frc::SmartDashboard::PutData(
      "Shoot 3/Wait 2",
      BuildShootAndMoveSequence(3_s /*conveyor on (s)*/, 2_s /*wait (s)*/,
                                units::second_t(kShootingTimeSeconds),
                                kDrivingSpeedPercent, kDistanceToDriveMeters));

  // Test command
  frc::SmartDashboard::PutData(
      "Load next ball", new RunConveyorUntilBallLoads(
                            &intake,                   // subsystem it uses
                            Intake::MOTOR_FULL_POWER,  // conveyor power
                            Intake::MOTOR_OFF_POWER,   // Ball pick-up power
                            units::second_t(3)         // Timeout
                            ));

  // Various shooter speed controls
  // TODO: Change this to be a single button for a command that reads a value
  // for speed from a chooser (or a text control) on the dashboard.
  frc::SmartDashboard::PutData("Run shooter at 100% power",
                               new RunShootingMotor(&shooter, 1.0));
  frc::SmartDashboard::PutData("Run shooter at 95% power",
                               new RunShootingMotor(&shooter, 0.95));
  frc::SmartDashboard::PutData("Run shooter at 90% power",
                               new RunShootingMotor(&shooter, 0.9));
  frc::SmartDashboard::PutData("Run shooter at 80% power",
                               new RunShootingMotor(&shooter, 0.8));
  frc::SmartDashboard::PutData("Run shooter at 75% power",
                               new RunShootingMotor(&shooter, 0.75));
  frc::SmartDashboard::PutData("Run shooter at 70% power",
                               new RunShootingMotor(&shooter, 0.7));

  if (false) {
    // Sample command from s/w team training
    frc::SmartDashboard::PutData("Do those spinnin", new DoASpin(&drivebase));
  }
  if (false) {
    frc::SmartDashboard::PutData("Shared tank drive", BuildTankDriveCommand());
  }

#ifdef ENABLE_PNEUMATICS
  frc::SmartDashboard::PutData(
      "Extend Intake",
      new frc2::InstantCommand([this]() { pneumatics.ExtendSolenoid(); },
                               {&pneumatics}));
  frc::SmartDashboard::PutData(
      "Retract Intake",
      new frc2::InstantCommand([this]() { pneumatics.RetractSolenoid(); },
                               {&pneumatics}));
  frc::SmartDashboard::PutData(
      "Toggle Intake",
      new frc2::InstantCommand([this]() { pneumatics.ToggleSolenoid(); },
                               {&pneumatics}));
#endif  // ENABLE_PNEUMATICS

  if (false) {
    // Sample trajectory commands
    std::vector<frc::Translation2d> points{frc::Translation2d(1_m, 0_m),
                                           frc::Translation2d(2_m, 0_m)};
    frc::SmartDashboard::PutData(
        "Go in a line",
        GenerateRamseteCommand(
            frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)), points,
            frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)), true));
    frc::SmartDashboard::PutData(
        "Go in an S",
        GenerateRamseteCommandFromPathFile("TestingS.wpilib.json", true));
  }

  if (false) {
    // Sample movement commands
    frc::SmartDashboard::PutData(
        "Go 9.144 meters at 50%",
        new DriveAtPowerForMeters(&drivebase, .5, 9.144_m));

    frc::SmartDashboard::PutData(
        "Go -9.144 meters at -50%",
        new DriveAtPowerForMeters(&drivebase, -.5, -1_m));

    frc::SmartDashboard::PutData("Simple command group",
                                 BuildBouncePathCommand());
  }

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

frc2::SequentialCommandGroup* RobotContainer::BuildConveyorSeqeunceForAuto(
    units::second_t secondsToRunConveyor, units::second_t secondsToWait) {
  std::vector<std::unique_ptr<frc2::Command>> commands;

  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(new DelayForTime(secondsToWait))));

  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      new TimedConveyor(&intake, secondsToRunConveyor, true))));

  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(new DelayForTime(secondsToWait))));

  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      new TimedConveyor(&intake, secondsToRunConveyor, true))));

  commands.push_back(std::move(
      std::unique_ptr<frc2::Command>(new DelayForTime(secondsToWait))));

  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      new TimedConveyor(&intake, secondsToRunConveyor, true))));

  return new frc2::SequentialCommandGroup(std::move(commands));
}

frc2::ParallelRaceGroup* RobotContainer::BuildConveyorAndShootingSequence(
    units::second_t secondsToRunConveyor, units::second_t secondsToWait,
    units::second_t timeForRunShooter) {
  std::vector<std::unique_ptr<frc2::Command>> commands;

  const double kShooterSpeedPercent = 0.8;
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      BuildConveyorSeqeunceForAuto(secondsToRunConveyor, secondsToWait))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      new ShootForTime(&shooter, timeForRunShooter, kShooterSpeedPercent))));

  return new frc2::ParallelRaceGroup(std::move(commands));
}

/*if (false) {
  frc2::SequentialCommandGroup* RobotContainer::BuildShootAndMoveSequence(
      units::second_t secondsToRunConveyor, units::second_t secondsToWait,
      units::second_t timeForRunShooter, double power, double amountToMove) {
    std::vector<std::unique_ptr<frc2::Command>> commands;
    commands.push_back(std::move(
        std::unique_ptr<frc2::Command>(BuildConveyorAndShootingSequence(
            secondsToRunConveyor, secondsToWait, timeForRunShooter))));
    commands.push_back(
        std::move(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
            &drivebase, power, units::length::meter_t(amountToMove)))));
    return new frc2::SequentialCommandGroup(std::move(commands));
  }
}*/

/*
frc2::SequentialCommandGroup* RobotContainer::BuildShootAndMoveSequence(
    units::second_t secondsToRunConveyor, units::second_t secondsToWait,
    units::second_t timeForRunShooter, double power, double amountToMove) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(new frc2::InstantCommand(
          [this]() { pneumatics.ExtendSolenoid(); }, {&pneumatics}))));
  commands.push_back(std::move(std::unique_ptr<frc2::Command>(
      new TimedIntake(&intake, units::second_t(1), true))));
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
          &drivebase, power, units::length::meter_t(amountToMove)))));
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(BuildConveyorAndShootingSequence(
          secondsToRunConveyor, secondsToWait, timeForRunShooter))));
  return new frc2::SequentialCommandGroup(std::move(commands));
}
*/

frc2::SequentialCommandGroup* RobotContainer::BuildShootAndMoveSequence(
    units::second_t secondsToRunConveyor, units::second_t secondsToWait,
    units::second_t timeForRunShooter, double power, double amountToMove) {
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(new frc2::InstantCommand(
          [this]() { pneumatics.ExtendSolenoid(); }, {&pneumatics}))));
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(new DriveAtPowerForMeters(
          &drivebase, power, units::length::meter_t(amountToMove)))));
  commands.push_back(
      std::move(std::unique_ptr<frc2::Command>(BuildConveyorAndShootingSequence(
          secondsToRunConveyor, secondsToWait, timeForRunShooter))));
  return new frc2::SequentialCommandGroup(std::move(commands));
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
  frc2::Command* blueB = BuildGalacticSearchPath(
      "GSearchBBlue Part1.wpilib.json", "GSearchBBlue Part2.wpilib.json",
      "GSearchBBlue Part3.wpilib.json", "GSearchBBlue Part4.wpilib.json");
  frc2::Command* blueA = BuildGalacticSearchPath(
      "GSearchABlue Part1.wpilib.json", "GSearchABlue Part2.wpilib.json",
      "GSearchABlue Part3.wpilib.json", "GSearchABlue Part4.wpilib.json");
  return new frc2::ConditionalCommand(std::unique_ptr<frc2::Command>(blueA),
                                      std::unique_ptr<frc2::Command>(blueB),
                                      [this] { return RecognizePathA(); });
}

frc2::ConditionalCommand* RobotContainer::BuildRedAlliancePaths() {
  frc2::Command* redA = BuildGalacticSearchPath(
      "GSearchARed Part1.wpilib.json", "GSearchARed Part2.wpilib.json",
      "GSearchARed Part3.wpilib.json", "GSearchARed Part4.wpilib.json");
  frc2::Command* redB = BuildGalacticSearchPath(
      "GSearchBRed Part1.wpilib.json", "GSearchBRed Part2.wpilib.json",
      "GSearchBRed Part3.wpilib.json", "GSearchBRed Part4.wpilib.json");
  return new frc2::ConditionalCommand(std::unique_ptr<frc2::Command>(redA),
                                      std::unique_ptr<frc2::Command>(redB),
                                      [this] { return RecognizePathA(); });
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
