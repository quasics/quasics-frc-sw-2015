// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "Constants.h"
#include "commands/DoASpin.h"
#include "commands/RunShootingMotor.h"
#include "commands/TankDrive.h"
#include "subsystems/Drivebase.h"

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

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
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
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
  frc::SmartDashboard::PutNumber(
      "Right Side Encoders ", drivebase.GetRightEncoderDistance().to<double>());
  frc::SmartDashboard::PutNumber(
      "Left Side Encoders", drivebase.GetLeftEncoderDistance().to<double>());
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

  frc::SimpleMotorFeedforward<units::meter> feedForward(
      ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
  frc::DifferentialDriveVoltageConstraint voltageConstraints(
      feedForward, kDriveKinematics, 10_V);
  frc::TrajectoryConfig config(kMaxSpeed, kMaxAcceleration);

  config.SetKinematics(kDriveKinematics);

  config.AddConstraint(voltageConstraints);

  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      start, interiorWaypoints, end, config);

  frc2::RamseteCommand ramseteCommand(
      exampleTrajectory, [this]() { return drivebase.GetPose(); },
      frc::RamseteController{kRamseteB, kRamseteZeta}, feedForward,
      kDriveKinematics, [this]() { return drivebase.GetWheelSpeeds(); },
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      [this](auto left, auto right) { drivebase.TankDriveVolts(left, right); },
      {&drivebase});

  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this, resetTelemetryAtStart, exampleTrajectory] {
        if (resetTelemetryAtStart) {
          drivebase.ResetOdometry(exampleTrajectory.InitialPose());
        }
      }),
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { drivebase.TankDriveVolts(0_V, 0_V); }, {}));
}
