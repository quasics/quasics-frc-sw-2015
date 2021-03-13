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
#include "../../../../Common2021/DeadBandEnforcer.h"
#include "../../../../Common2021/SpeedScaler.h"
#include "../../../../Common2021/TeleopArcadeDrive.h"
#include "../../../../Common2021/TeleopTankDrive.h"
#include "../../../../Common2021/TrajectoryCommandGenerator.h"
#include "../../../../Common2021/TurnToTargetCommand.h"
#include "Constants.h"
#include "commands/DriveForward.h"

#undef DRIVE_ARCADE_STYLE
#define USE_GAMESIR_CONTROLLER

// Unfortunately, SendableChoosers don't seem to work with the simulator
// environment, just with a real Driver Station.  And since that won't work with
// my Mac, I need to go with a fallback option.
constexpr bool kUseChooserForAutoCmd = false;

// The Logitech controllers can't be used with the Mac OS, which is what Matt
// has handy.  So here's a convenient hack to encapsulate the idea of what type
// of device we're using.
inline bool usingLogitechController() {
#ifdef USE_GAMESIR_CONTROLLER
  return false;
#else
  return true;
#endif
}

RobotContainer::RobotContainer() {
  // Note: when running under simulator, current path is that of the
  // project (.../Experimental/RomiToy2021).
  m_helper.InstallSliders();

  ConfigureDrivingCommand();

  // Configure the button bindings
  ConfigureButtonBindings();

  // Configure how we'll pick what to do in auto mode.
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
  //////////////////////////////////////////////////////////////////
  // Set up scaling support for the speed controls.
  constexpr double turtleMax = 0.5;
  constexpr double normalMax = 0.75;
  constexpr double turboMax = 1.0;
  DeadBandEnforcer scalingDeadBand(
      0.25);  // Require at least 25% on the triggers to activate speed scaling

  SpeedScaler scaler(
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
      normalMax, turtleMax, turboMax);

  //////////////////////////////////////////////////////////////////
  // Set up the actual tank drive command, using the scaler from above
  // to help provide limiting factors.
  //
  // Notice that I'm negating the speed values for both left and right.
  // That's because I find it more convenient to tuck a camera in under
  // what would normally be the "front" of the robot, facing in the
  // opposite direction, when doing work with vision processing.  (The
  // cameras I'm using don't anchor well on the other side.)  So I'm
  // negating the speed values, so that I can make it run in the
  // direction that "looks ahead" when trying to drive "forward".
  //
  // I could just mark the speed controllers as inverted in the
  // Drivetrain code, but that needs to be done before they're assigned
  // to a DifferentialDrive, which would complicate the code there: so
  // it's simpler just to make the small tweak here. :-)

  // "Dead band" evaluation for throttle control: if a stick hasn't moved at
  // least this much, it's ignored.
  DeadBandEnforcer throttleDeadBand(0.06);

  m_drive.SetDefaultCommand(TeleopTankDrive(
      &m_drive,
      [this, scaler, throttleDeadBand] {
        // Left speed control
        const int leftJoystickAxis =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::LeftYAxis
                : int(JoystickDefinitions::GameSirPro::LeftVertical);
        return -scaler(
            throttleDeadBand(m_controller.GetRawAxis(leftJoystickAxis)));
      },
      [this, scaler, throttleDeadBand] {
        // Right speed control
        const int rightJoystickAxis =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::RightYAxis
                : int(JoystickDefinitions::GameSirPro::RightVertical);
        return -scaler(
            throttleDeadBand(m_controller.GetRawAxis(rightJoystickAxis)));
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

void RobotContainer::EnableArcadeDrive() {
  // "Dead band" evaluation for speed controls.
  DeadBandEnforcer deadBand(0.06);

  m_drive.SetDefaultCommand(TeleopArcadeDrive(
      &m_drive,
      [this, deadBand] {
        const int joystickVerticalAxis =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::LeftYAxis
                : int(JoystickDefinitions::GameSirPro::LeftVertical);
        return -deadBand(m_controller.GetRawAxis(joystickVerticalAxis));
      },
      [this, deadBand] {
        const int joystickHorizontalalAxis =
            usingLogitechController()
                ? JoystickDefinitions::LogitechGamePad::LeftXAxis
                : int(JoystickDefinitions::GameSirPro::LeftHorizontal);
        return deadBand(m_controller.GetRawAxis(joystickHorizontalalAxis));
      }));
}

void RobotContainer::ConfigureDrivingCommand() {
#ifdef DRIVE_ARCADE_STYLE
  EnableArcadeDrive();
#else
  EnableTankDrive();
#endif
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  DriveForward forward(
      &m_drive,
      .5,                    // Power setting
      [] { return false; },  // Stop condition (move until interrupted)
      DriveForward::Gyro,    // Sensor mode
      false                  // Noisy?
  );

  frc2::JoystickButton(&m_controller, int(JoystickDefinitions::GameSirPro::G))
      .WhenPressed(frc2::PrintCommand("Button 'G' on controller was pressed"));

  frc2::JoystickButton(&m_controller, int(JoystickDefinitions::GameSirPro::A))
      .WhenHeld(forward);
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
    const frc::Pose2d& start,
    const std::vector<frc::Translation2d>& interiorWaypoints,
    const frc::Pose2d& end, bool resetTelemetryAtStart) {
#if 1
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
       RobotData::DriveConstants::kDDriveVel},
      // Speed profile
      {RobotData::PathFollowingLimits::kMaxSpeed,
       RobotData::PathFollowingLimits::kMaxAcceleration});

  return generator.GenerateCommand(start, interiorWaypoints, end,
                            resetTelemetryAtStart);
#else
  using namespace RobotData::DriveConstants;
  using namespace RobotData::PathFollowingLimits;

  const frc::DifferentialDriveKinematics kDriveKinematics{
      m_drive.GetTrackWidth()};

  // Set up config for trajectory
  frc::SimpleMotorFeedforward<units::meter> feedForward(
      ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
  frc::DifferentialDriveVoltageConstraint voltageConstraints(
      feedForward, kDriveKinematics, 10_V);
  frc::TrajectoryConfig config(kMaxSpeed, kMaxAcceleration);

  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(kDriveKinematics);

  // Apply the voltage constraint
  config.AddConstraint(voltageConstraints);

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      start, interiorWaypoints, end, config);

  frc2::RamseteCommand ramseteCommand(
      /* trajectory to follow */
      trajectory,
      /* function that supplies the robot pose */
      [this]() { return m_drive.GetPose(); },
      /* RAMSETE controller used to follow the trajectory */
      frc::RamseteController(kRamseteB, kRamseteZeta),
      /* calculates the feedforward for the drive */
      feedForward,
      /* kinematics for the robot drivetrain */
      kDriveKinematics,
      /* function that supplies the left/right side speeds */
      [this] { return m_drive.GetWheelSpeeds(); },
      /* left controller */
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      /* right controller */
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      /* output function */
      [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
      /* required subsystems */
      {&m_drive});

  return new frc2::SequentialCommandGroup(
      frc2::PrintCommand("Starting trajectory code"),
      frc2::InstantCommand(
          [this, resetTelemetryAtStart, trajectory] {
                if (resetTelemetryAtStart) {
                  m_drive.ResetOdometry(trajectory.InitialPose());
                }
              },
          {&m_drive}),
      std::move(ramseteCommand),
      // Shut the drive down.
      frc2::PrintCommand("Shutting down trajectory code"),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); },
                           {&m_drive}));
#endif
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

  return GenerateRamseteCommand(start, interiorWaypoints, end,
                                resetTelemetryAtStart);
}
