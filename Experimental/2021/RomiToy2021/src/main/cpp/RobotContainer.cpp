// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>

#include <algorithm>

#include "../../../../Common2021/ButtonHelpers.h"
#include "../../../../Common2021/DriveStraightCommand.h"
#include "../../../../Common2021/SpeedScaler.h"
#include "../../../../Common2021/TeleopArcadeDrive.h"
#include "../../../../Common2021/TeleopCurvatureDrive.h"
#include "../../../../Common2021/TeleopTankDrive.h"
#include "../../../../Common2021/TrajectoryCommandGenerator.h"
#include "../../../../Common2021/TurnToTargetCommand.h"
#include "Constants.h"

#undef DRIVE_ARCADE_STYLE
#undef DRIVE_CURVATURE_STYLE
// #define USE_GAMESIR_CONTROLLER

constexpr double kTurtleModeMaxSpeed = 0.5;
constexpr double kNormalModeMaxSpeed = 0.75;
constexpr double kTurboModeMaxSpeed = 1.0;

// Unfortunately, SendableChoosers don't seem to work with the simulator
// environment, just with a real Driver Station.  And since that won't work with
// my Mac, I need to go with a fallback option.
constexpr bool kUseChooserForAutoCmd = false;

// The Logitech controllers can't be used with the Mac OS, which is what Matt
// has handy.  So here's a convenient hack to encapsulate the idea of what type
// of device we're using.
inline constexpr bool usingLogitechController() {
#ifdef USE_GAMESIR_CONTROLLER
  return false;
#else
  return true;
#endif
}

RobotContainer::RobotContainer() {
  //////////////////////////////////////////////////////////////////
  // Set up scaling support for the speed controls.
  DeadBandEnforcer scalingDeadBand(
      0.25);  // Require at least 25% on the triggers to activate speed scaling

  m_speedScaler.reset(new SpeedScaler(
      [this, scalingDeadBand] {
        // Speed mode signal
        const bool usingLogitech = usingLogitechController();
        const int turtleTrigger =
            usingLogitech
                ? JoystickDefinitions::LogitechGamePad::LeftTriggerAxis
                : JoystickDefinitions::GameSirPro::LeftTrigger;
        const int turboTrigger =
            usingLogitech
                ? JoystickDefinitions::LogitechGamePad::RightTriggerAxis
                : JoystickDefinitions::GameSirPro::RightTrigger;
        if (scalingDeadBand(m_controller.GetRawAxis(turtleTrigger)) > 0)
          return SpeedScaler::Turtle;
        else if (scalingDeadBand(m_controller.GetRawAxis(turboTrigger)) > 0)
          return SpeedScaler::Turbo;
        else
          return SpeedScaler::Normal;
      },
      kNormalModeMaxSpeed, kTurtleModeMaxSpeed, kTurboModeMaxSpeed));

  //////////////////////////////////////////////////////////////////
  // Set up sliders to allow for customizing vision processing on
  // the RasPi.
  m_helper.InstallSliders();

  //////////////////////////////////////////////////////////////////
  // Install different commands.
  ConfigureDrivingCommand();
  ConfigureButtonBindings();
  ConfigureAutonomousSelection();
}

void RobotContainer::ConfigureAutonomousSelection() {
  // Note: I'd just use a SendableChooser here, but that doesn't work with the
  // simulator, so I need something else on the Romi.  On the other hand,
  // setting up the chooser, anyway, gives me an example to use when working
  // with the real DS.

  // A convenient way to make sure that commands are tied into both mechanisms
  // for selection, and in the same order.  This helper widget will add them to
  // both for me, so that I don't need to worry about missing anything, or
  // moving stuff around in one but not the other, etc.
  auto adder = [this](const char* name, frc2::Command* cmd) {
    if (m_autoModeOptions.empty()) {
      // The first command is assumed to be the default one.
      m_autonomousChooser.SetDefaultOption(name, cmd);
    } else {
      m_autonomousChooser.AddOption(name, cmd);
    }
    m_autoModeOptions.push_back(std::shared_ptr<frc2::Command>(cmd));
  };
  adder("Linear", GenerateSampleRamseteCommand(StraightLineTrajectory, true));
  adder("S-curve", GenerateSampleRamseteCommand(S_CurveTrajectory, true));
  adder("Figure-8", GenerateSampleRamseteCommand(FigureEightTrajectory, true));
  adder("Barrel roll (traj)",
        GenerateRamseteCommand("BarrelRoll.wpilib.json", true));
  adder("Single loop (traj)",
        GenerateRamseteCommand("SingleLoop.wpilib.json", true));
  adder("Turn to Target", new TurnToTargetCommand(&m_drive, 0.350));

  // Put the SendableChooser on the Smart Dashboard for the driver's station.
  frc::SmartDashboard::PutData("Auto mode", &m_autonomousChooser);

  // Put the numeric input widget into the network tables for use with the
  // simulator.
  m_autoModeSelection = frc::Shuffleboard::GetTab("Settings")
                            .Add("Auto mode index", 0)
                            .WithWidget(frc::BuiltInWidgets::kTextView)
                            .GetEntry();
}

void RobotContainer::EnableTankDrive() {
  m_drive.SetDefaultCommand(TeleopTankDrive(
      &m_drive,
      [this] {
        // Left speed control
        const int leftJoystickAxis =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::LeftYAxis
                : int(JoystickDefinitions::GameSirPro::LeftVertical);
        return -(*m_speedScaler)(
            m_throttleDeadBand(m_controller.GetRawAxis(leftJoystickAxis)));
      },
      [this] {
        // Right speed control
        const int rightJoystickAxis =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::RightYAxis
                : int(JoystickDefinitions::GameSirPro::RightVertical);
        return -(*m_speedScaler)(
            m_throttleDeadBand(m_controller.GetRawAxis(rightJoystickAxis)));
      },
      [this] {
        // Provides "Switch drive enabled?" signal to tank drive.
        static ButtonToggleMonitor monitor(
            m_controller,
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::StartButton
                : int(JoystickDefinitions::GameSirPro::S));
        static bool enabled = false;

        if (monitor.ShouldToggle()) {
          enabled = !enabled;
        }
        return enabled;
      }));
}

void RobotContainer::EnableCurvatureDrive() {
  m_drive.SetDefaultCommand(TeleopCurvatureDrive(
      &m_drive,
      [this] {
        const int joystickVerticalAxis =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::LeftYAxis
                : int(JoystickDefinitions::GameSirPro::LeftVertical);
        return -m_throttleDeadBand(
            m_controller.GetRawAxis(joystickVerticalAxis));
      },
      [this] {
        const int joystickHorizontalalAxis =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::LeftXAxis
                : int(JoystickDefinitions::GameSirPro::LeftHorizontal);
        return m_throttleDeadBand(
            m_controller.GetRawAxis(joystickHorizontalalAxis));
      },
      [this] {
        return m_controller.GetRawButton(
          usingLogitechController() ?
        JoystickDefinitions::LogitechGamePad::StartButton : JoystickDefinitions::GameSirPro::LeftShoulder);
      }));
}

void RobotContainer::EnableArcadeDrive() {
  m_drive.SetDefaultCommand(TeleopArcadeDrive(
      &m_drive,
      [this] {
        const int joystickVerticalAxis =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::LeftYAxis
                : int(JoystickDefinitions::GameSirPro::LeftVertical);
        return -m_throttleDeadBand(
            m_controller.GetRawAxis(joystickVerticalAxis));
      },
      [this] {
        const int joystickHorizontalalAxis =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::LeftXAxis
                : int(JoystickDefinitions::GameSirPro::LeftHorizontal);
        return m_throttleDeadBand(
            m_controller.GetRawAxis(joystickHorizontalalAxis));
      }));
}

void RobotContainer::ConfigureDrivingCommand() {
#ifdef DRIVE_ARCADE_STYLE
  EnableArcadeDrive();
#elif defined(DRIVE_CURVATURE_STYLE)
  EnableCurvatureDrive();
#else
  EnableTankDrive();
#endif
}

// Configure your button bindings here
void RobotContainer::ConfigureButtonBindings() {
  // Sample binding for a button.
  frc2::JoystickButton(&m_controller, int(JoystickDefinitions::GameSirPro::G))
      .WhenPressed(frc2::PrintCommand("Button 'G' on controller was pressed"));

  // Simple command to drive straight ahead at a fixed speed.
  DriveStraightCommand forward(
      &m_drive, [] { return .5; },  // Power setting
      [] { return false; },         // Stop condition (move until interrupted)
      DriveStraightCommand::Gyro,   // Sensor mode
      false                         // Noisy?
  );
  frc2::JoystickButton(&m_controller, int(JoystickDefinitions::GameSirPro::A))
      .WhenHeld(forward);

  // Set up variable-speed "drive forward" command
  std::function<double()> variableSpeedController = [this] {
    const int rightJoystickAxis =
        usingLogitechController()
            ? JoystickDefinitions::LogitechGamePad::RightYAxis
            : int(JoystickDefinitions::GameSirPro::RightVertical);
    auto result = -(*m_speedScaler)(
        m_throttleDeadBand(m_controller.GetRawAxis(rightJoystickAxis)));
    return result;
  };
  DriveStraightCommand forwardVariableSpeed(
      &m_drive,                 // Drive base
      variableSpeedController,  // Power setting
      [] { return false; },     // Stop condition (move until interrupted)
      DriveStraightCommand::Encoders,  // Sensor mode
      true                             // Noisy?
  );
  frc2::JoystickButton(&m_controller,
                       int(JoystickDefinitions::GameSirPro::LeftShoulder))
      .WhenHeld(forwardVariableSpeed);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  if (kUseChooserForAutoCmd) {
    frc2::Command* selectedInChooser = m_autonomousChooser.GetSelected();
    return selectedInChooser;
  } else {
    if (m_autoModeOptions.empty()) {
      return nullptr;
    }

    // Gets the selection from the NetworkTables entry, clamped to the range
    // that's supported (i.e., [0..N-1]).
    const int selection = std::clamp<int>(int(m_autoModeSelection.GetDouble(0)),
                                          0, m_autoModeOptions.size() - 1);
    return m_autoModeOptions[selection].get();
  }
}

frc2::SequentialCommandGroup* RobotContainer::GenerateRamseteCommand(
    const std::string jsonFileName, bool resetTelemetryAtStart) {
  // Configure trajectory generation.  (Note that this could be
  // embedded in the RobotContainer -- or elsewhere -- and reused
  // as needed, including tweaks to max speed/acceleration.)
  TrajectoryCommandGenerator generator(
      // Drive base being controlled
      &m_drive,
      // Drive profile data
      {RobotData::DriveConstants::ksVolts,
       RobotData::DriveConstants::kvVoltSecondsPerMeter,
       RobotData::DriveConstants::kaVoltSecondsSquaredPerMeter},
      // PID configuration values
      {RobotData::DriveConstants::kPDriveVel,
       RobotData::DriveConstants::kIDriveVel,
       RobotData::DriveConstants::kDDriveVel});

  return generator.GenerateCommandFromPathWeaverFile(
      jsonFileName, resetTelemetryAtStart
                        ? TrajectoryCommandGenerator::ResetTelemetryAtStart
                        : TrajectoryCommandGenerator::UseExistingTelemetry);
}

frc2::SequentialCommandGroup* RobotContainer::GenerateRamseteCommand(
    const frc::Pose2d& start,
    const std::vector<frc::Translation2d>& interiorWaypoints,
    const frc::Pose2d& end, bool resetTelemetryAtStart) {
  // Configure trajectory generation.  (Note that this could be
  // embedded in the RobotContainer -- or elsewhere -- and reused
  // as needed, including tweaks to max speed/acceleration.)
  TrajectoryCommandGenerator generator(
      // Drive base being controlled
      &m_drive,
      // Drive profile data
      {RobotData::DriveConstants::ksVolts,
       RobotData::DriveConstants::kvVoltSecondsPerMeter,
       RobotData::DriveConstants::kaVoltSecondsSquaredPerMeter},
      // PID configuration values
      {RobotData::DriveConstants::kPDriveVel,
       RobotData::DriveConstants::kIDriveVel,
       RobotData::DriveConstants::kDDriveVel});

  TrajectoryCommandGenerator::SpeedProfile speedProfile{
      RobotData::PathFollowingLimits::kMaxSpeed,
      RobotData::PathFollowingLimits::kMaxAcceleration};

  return generator.GenerateCommand(
      speedProfile, start, interiorWaypoints, end,
      resetTelemetryAtStart ? TrajectoryCommandGenerator::ResetTelemetryAtStart
                            : TrajectoryCommandGenerator::UseExistingTelemetry);
}

frc2::SequentialCommandGroup* RobotContainer::GenerateSampleRamseteCommand(
    TrajectoryExample example, bool resetTelemetryAtStart) {
  frc::Pose2d start = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg));
  frc::Pose2d end;
  std::vector<frc::Translation2d> interiorWaypoints;

  switch (example) {
    case S_CurveTrajectory:
      interiorWaypoints = std::vector<frc::Translation2d>{
          // Pass through these two interior waypoints, making an 's' curve path
          frc::Translation2d(0.5_m, 0.5_m),
          frc::Translation2d(1_m, -0.5_m),
      };
      // End 3 meters straight ahead of where we started, facing forward
      end = frc::Pose2d(1.5_m, 0_m, frc::Rotation2d(0_deg));
      break;

    case StraightLineTrajectory:
      interiorWaypoints = std::vector<frc::Translation2d>{
          frc::Translation2d(1_m, 0_m),
          frc::Translation2d(2_m, 0_m),
      };
      // End @ (3m,0m) distance from start, facing foreward.
      end = frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg));
      break;

    case FigureEightTrajectory:
      interiorWaypoints = std::vector<frc::Translation2d>{
          // Pass through these interior waypoints, following a figure-8 path
          frc::Translation2d(0.5_m, 0.5_m),  frc::Translation2d(1_m, -0.5_m),
          frc::Translation2d(1.5_m, 0_m),    frc::Translation2d(1_m, 0.5_m),
          frc::Translation2d(0.5_m, -0.5_m),
      };
      // End back where we started, facing forward
      end = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg));
      break;

    default:
      std::cerr << "Unsupported example trajectory (" << int(example)
                << ") specified!" << std::endl;
      return nullptr;
  }

  return GenerateRamseteCommand(
      start, interiorWaypoints, end,
      resetTelemetryAtStart ? TrajectoryCommandGenerator::ResetTelemetryAtStart
                            : TrajectoryCommandGenerator::UseExistingTelemetry);
}
