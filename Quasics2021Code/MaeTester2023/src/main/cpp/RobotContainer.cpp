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

#include <iostream>

#include "Constants.h"
#include "commands/AutoIntakeCells.h"
#include "commands/DecrementLinearActuator.h"
#include "commands/DelayForTime.h"
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
#include "commands/TimedConveyor.h"
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
  // Configure the button bindings.
  ConfigureControllerButtonBindings();
  ConfigureSmartDashboard();
  //ConfigureAutoSelection();
}

double RobotContainer::deadband(double num) {
  if (num > OIConstants::DeadBand_LowValue &&
      num < OIConstants::DeadBand_HighValue) {
    return 0;
  }
  return num;
}

std::unique_ptr<TriggerDrivenShootingCommand>
RobotContainer::BuildShootingCommand() {
  // Dead band control for the left trigger, letting us use it as though it
  // was a button (on/off).
  std::function<bool()> runHighSpeedSupplier = [this] {
    if (operatorController.GetLeftTriggerAxis() >= 0.5) {
      return true;
    }
    return false;
  };

  // Dead band control for the left trigger, letting us use it as though it
  // was a button (on/off).
  std::function<bool()> runLowSpeedSupplier = [this] {
    if (operatorController.GetRightTriggerAxis() >= 0.5) {
      return true;
    }
    return false;
  };

  return std::make_unique<TriggerDrivenShootingCommand>(
      &shooter, 1.0, 0.8, runHighSpeedSupplier, runLowSpeedSupplier);
}

std::unique_ptr<TankDrive> RobotContainer::BuildTankDriveCommand() {
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

  return std::make_unique<TankDrive>(
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

/*void RobotContainer::InstallDefaultCommands() {
  auto tankDriveCmd = BuildTankDriveCommand();
  drivebase.SetDefaultCommand(*tankDriveCmd);

  lights.SetDefaultCommand(m_defaultLightingCommand);

  std::unique_ptr<TriggerDrivenShootingCommand> shooterCmd(
      BuildShootingCommand());
  shooter.SetDefaultCommand(*shooterCmd);
}
*/
void RobotContainer::RunCommandWhenOperatorButtonIsHeld(
    int buttonId, frc2::Command* command) {
  frc2::JoystickButton(&operatorController, buttonId)
      .Trigger::WhileTrue(command);  // see last year's code
}

void RobotContainer::RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                                      frc2::Command* command) {
  frc2::JoystickButton(&driverJoystick, logitechButtonId).Trigger::WhileTrue(command);
}

void RobotContainer::ConfigureControllerButtonBindings() {
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

  // Other commands.
  static IntakePowerCells intakepowercells(&intake);

#define ENABLE_BINDINGS_FOR_DEMO
#ifdef ENABLE_BINDINGS_FOR_DEMO
  // Bindings defined by Matt Healy and Meg Gilmore on 12Aug2021,
  // for use at the demo on 14Aug2021.
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kA,
                                     &conveyorBackwardCommand);
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kB,
                                     &conveyorForwardCommand);

  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kLeftBumper,
                                     shooterToMaximumCommandPtr.get());
  RunCommandWhenOperatorButtonIsHeld(frc::XboxController::Button::kRightBumper,
                                     shooterToMinimumCommandPtr.get());

  RunCommandWhenDriverButtonIsHeld(OIConstants::LogitechGamePad::LeftShoulder,
                                   &intakeForwardCommand);
  RunCommandWhenDriverButtonIsHeld(OIConstants::LogitechGamePad::RightShoulder,
                                   &intakeReverseCommand);

#else
  RunCommandWhenOperatorButtonIsHeld(
      frc::XboxController::Button::kA,
      &runShooterFullSpeed);  // see last year's code

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
#endif  // ENABLE_BINDINGS_FOR_DEMO
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoChooser.GetSelected();
}

void RobotContainer::AddLightingButtonsToSmartDashboard() {
  // Lighting controls (under construction)
  frc::SmartDashboard::PutData("Red Light",
                               new ColorLights(&lights, 255, 0, 0));
  frc::SmartDashboard::PutData("Green Light",
                               new ColorLights(&lights, 0, 255, 0));
  frc::SmartDashboard::PutData("Blue Light",
                               new ColorLights(&lights, 0, 0, 255));
  frc::SmartDashboard::PutData("Lights Out", new ColorLights(&lights, 0, 0, 0));
}

void RobotContainer::AddShooterAngleControlsToSmartDashboard() {
  frc::SmartDashboard::PutData(
      "Min Shooter Pos",
      new frc2::InstantCommand([this]() { shooter.SetServoPosition(0.0); },
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


void RobotContainer::AddShooterSpeedControlsToSmartDashboard() {
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
}

void RobotContainer::AddSampleMovementCommandsToSmartDashboard() {
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

void RobotContainer::AddExampleTrajectoryCommandsToSmartDashboard() {
  // Sample trajectory commands
  std::vector<frc::Translation2d> points{frc::Translation2d(1_m, 0_m),
                                         frc::Translation2d(2_m, 0_m)};
  frc::SmartDashboard::PutData(
      "Go in a line", GenerateRamseteCommand(
                          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)), points,
                          frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)), true));

  frc::SmartDashboard::PutData("Go in an S", GenerateRamseteCommandFromPathFile(
                                                 "TestingS.wpilib.json", true));
}

void RobotContainer::ConfigureSmartDashboard() {
  if (true) {
    AddLightingButtonsToSmartDashboard();
  }

  AddShooterAngleControlsToSmartDashboard();
  AddShooterSpeedControlsToSmartDashboard();
  // Test command
  frc::SmartDashboard::PutData(
      "Load next ball", new RunConveyorUntilBallLoads(
                            &intake,                   // subsystem it uses
                            Intake::MOTOR_FULL_POWER,  // conveyor power
                            Intake::MOTOR_OFF_POWER,   // Ball pick-up power
                            units::second_t(3)         // Timeout
                            ));

  if (false) {
    AddExampleTrajectoryCommandsToSmartDashboard();
  }

  if (false) {
    AddSampleMovementCommandsToSmartDashboard();
  }
}

frc2::SequentialCommandGroup* RobotContainer::BuildBouncePathCommand() {
  std::vector<std::unique_ptr<frc2::Command>> bouncePathPieces;

  bouncePathPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile("Bounce Part1.wpilib.json", true))));
  bouncePathPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile("Bounce Part2.wpilib.json", false))));
  bouncePathPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile("Bounce Part3.wpilib.json", false))));
  bouncePathPieces.push_back(std::move(std::unique_ptr<frc2::Command>(
      GenerateRamseteCommandFromPathFile("Bounce Part4.wpilib.json", false))));

  return new frc2::SequentialCommandGroup(std::move(bouncePathPieces));
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
  std::string jsonFilePath =
      frc::filesystem::GetDeployDirectory() + "/paths/" + jsonFile;

  frc::Trajectory trajectory =
      frc::TrajectoryUtil::FromPathweaverJson(jsonFilePath);
  return trajectory;
}
