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
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/CommandHelper.h>

#include <filesystem>

#include "../../../../Common2021/ButtonHelpers.h"
#include "../../../../Common2021/DeadBandEnforcer.h"
#include "../../../../Common2021/SpeedScaler.h"
#include "../../../../Common2021/TeleopArcadeDrive.h"
#include "../../../../Common2021/TeleopTankDrive.h"
#include "Constants.h"
#include "commands/DriveForward.h"

#undef DRIVE_ARCADE_STYLE
#define TURN_TO_TARGET_AUTO
#define USE_GAMESIR_CONTROLLER

// Unfortunately, the Logitech controllers can't be used with the Mac OS,
// which is what Matt has handy.  So here's a convenient hack to encapsulate the
// idea of what type of device we're using.
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

  frc::SmartDashboard::PutData("Follow sample path", GenerateRamseteCommand());
}

void RobotContainer::EnableTankDrive() {
  // Set up scaling support for the speed controls.
  constexpr double turtleMax = 0.5;
  constexpr double normalMax = 0.75;
  constexpr double turboMax = 1.0;
  DeadBandEnforcer scalingDeadBand(0.25);  // Require at least 25% to activate
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
          return SpeedScaler::Mode::Turtle;
        else if (scalingDeadBand(m_controller.GetRawAxis(turboTrigger)) > 0)
          return SpeedScaler::Mode::Turbo;
        else
          return SpeedScaler::Mode::Normal;
      },
      normalMax, turtleMax, turboMax);

  // "Dead band" evaluation for speed controls.
  DeadBandEnforcer throttleDeadBand(0.06);

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
      DriveForward::SensorMode::Gyro,  // Sensor mode
      false                            // Noisy?
  );

  frc2::JoystickButton(&m_controller, int(JoystickDefinitions::GameSirPro::G))
      .WhenPressed(frc2::PrintCommand("Button 'G' on controller was pressed"));

  frc2::JoystickButton(&m_controller, int(JoystickDefinitions::GameSirPro::A))
      .WhenHeld(forward);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
#ifdef TURN_TO_TARGET_AUTO
  return &m_turnToTargetCommand;
#else
  // An example command will be run in autonomous
  return &m_autonomousCommand;
#endif
}

// TODO(mjh): Modify this method so that we can pass in the file name for
// a set of path data, rather than using a fixed trajectory.
frc2::SequentialCommandGroup* RobotContainer::GenerateRamseteCommand() {
  using namespace RobotData::DriveConstants;
  using namespace RobotData::PathFollowingLimits;

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

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  frc2::RamseteCommand ramseteCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },
      frc::RamseteController(kRamseteB, kRamseteZeta), feedForward,
      kDriveKinematics, [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController(kPDriveVel, 0, 0),
      frc2::PIDController(kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  return new frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}
