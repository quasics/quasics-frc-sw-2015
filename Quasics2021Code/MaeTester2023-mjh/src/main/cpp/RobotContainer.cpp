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
#include "commands/DecrementLinearActuator.h"
#include "commands/IncrementLinearActuator.h"
#include "commands/RunOnlyConveyorMotor.h"
#include "commands/RunOnlyConveyorMotorReverse.h"
#include "commands/RunOnlyIntakeMotor.h"
#include "commands/RunOnlyIntakeMotorReverse.h"
#include "commands/RunShootingMotor.h"
#include "subsystems/Drivebase.h"
#include "subsystems/Intake.h"
#include "subsystems/Lights.h"

///////////////////////////////////////////////////////////////////////////////
// Conditional compilation flags start here.

// DEFINE this to disable "turbo" mode (e.g., with untrusted drivers :-).
#define DISABLE_TURBO_MODE

#define ENABLE_BINDINGS_FOR_DEMO

#undef ENABLE_ARCADE_DRIVE

// Conditional compilation flags end here.
///////////////////////////////////////////////////////////////////////////////

RobotContainer::RobotContainer() {
  //////////////////////////////////////////
  // Configure default commands for the key subsystems.
  InstallDefaultCommands();

  //////////////////////////////////////////
  // Configure the button bindings.
  ConfigureControllerButtonBindings();
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

std::unique_ptr<ArcadeDrive> RobotContainer::BuildSplitArcadeDriveCommand() {
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

  return std::make_unique<ArcadeDrive>(
      &drivebase,
      /* forwardSpeed */
      [this, speedScaler] {
        double stickValue =
            -driverJoystick.GetRawAxis(OIConstants::LogitechGamePad::LeftYAxis);
        return speedScaler(deadband(stickValue));
      },
      /* turnSpeed */
      [this, speedScaler] {
        double stickValue = -driverJoystick.GetRawAxis(
            OIConstants::LogitechGamePad::RightXAxis);
        return speedScaler(deadband(stickValue));
      });
}

void RobotContainer::InstallDefaultCommands() {
#ifdef ENABLE_ARCADE_DRIVE
  auto driveCmd = BuildSplitArcadeDriveCommand();
#else
  auto driveCmd = BuildTankDriveCommand();
#endif

  drivebase.SetDefaultCommand(*driveCmd);

  lights.SetDefaultCommand(m_defaultLightingCommand);

  std::unique_ptr<TriggerDrivenShootingCommand> shooterCmd(
      BuildShootingCommand());
  shooter.SetDefaultCommand(*shooterCmd);
}

void RobotContainer::RunCommandWhenOperatorButtonIsHeld(
    int buttonId, frc2::Command* command) {
  frc2::JoystickButton(&operatorController, buttonId)
      .WhileHeld(command);  // see last year's code
}

void RobotContainer::RunCommandWhenDriverButtonIsHeld(int logitechButtonId,
                                                      frc2::Command* command) {
  frc2::JoystickButton(&driverJoystick, logitechButtonId).WhileHeld(command);
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

#ifdef ENABLE_BINDINGS_FOR_DEMO
  // Bindings defined by Matt Healy on 18May2023, moving all controls onto
  // a single controller for convenience at demos.
  RunCommandWhenDriverButtonIsHeld(OIConstants::LogitechGamePad::AButton,
                                   &conveyorBackwardCommand);
  RunCommandWhenDriverButtonIsHeld(OIConstants::LogitechGamePad::BButton,
                                   &conveyorForwardCommand);

  RunCommandWhenDriverButtonIsHeld(OIConstants::LogitechGamePad::LeftShoulder,
                                   shooterToMaximumCommandPtr.get());
  RunCommandWhenDriverButtonIsHeld(OIConstants::LogitechGamePad::RightShoulder,
                                   shooterToMinimumCommandPtr.get());

  RunCommandWhenDriverButtonIsHeld(OIConstants::LogitechGamePad::XButton,
                                   &runShooterFullSpeed); // see last year's code
  RunCommandWhenDriverButtonIsHeld(OIConstants::LogitechGamePad::YButton,
                                   &runShooterLowSpeed); // see last year's code
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

void RobotContainer::ConfigureAutoSelection() {
  m_autoChooser.SetDefaultOption("Do nothing",
                                 new frc2::PrintCommand("I refuse to move."));
  frc::SmartDashboard::PutData("Auto mode", &m_autoChooser);
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

void RobotContainer::ConfigureSmartDashboard() {
  AddLightingButtonsToSmartDashboard();

  AddShooterAngleControlsToSmartDashboard();
  AddShooterSpeedControlsToSmartDashboard();
}
