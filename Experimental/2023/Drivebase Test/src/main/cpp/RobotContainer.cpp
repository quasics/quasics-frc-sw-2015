// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <iostream>

#include "commands/DriveFromVoltageForTime.h"
#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand(frc2::InstantCommand([this]{m_drive.TankDriveVolts(0_V, 0_V);}, {&m_drive}) );

  // Configure the button bindings
  ConfigureBindings();
  AddTestButtonsOnSmartDashboard();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
// Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint{
      frc::SimpleMotorFeedforward<units::meters>{
          PathWeaverConstants::kS, PathWeaverConstants::kV, PathWeaverConstants::kA},
      kDriveKinematics, 5_V};

  // Set up config for trajectory
  frc::TrajectoryConfig config{kMaxSpeed, kMaxAcceleration};
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  //config.SetReversed(true);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {
        //frc::Translation2d{1_m, 0_m},
        //frc::Translation2d{-1_m, 1_m}
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{1_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc2::CommandPtr ramseteCommand{frc2::RamseteCommand(
      exampleTrajectory, [this] { return m_drive.GetPose(); }, // Displaying Get Pose in Dashboard
      frc::RamseteController{kRamseteB, kRamseteZeta},
      frc::SimpleMotorFeedforward<units::meters>{
          PathWeaverConstants::kS, PathWeaverConstants::kV, PathWeaverConstants::kA},
      kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController{PathWeaverConstants::kP, 0, 0},
      frc2::PIDController{PathWeaverConstants::kP, 0, 0},
      [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
      {&m_drive})}; 

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose()); // maybe here is the fundamental issue?????




  for (const auto& state : exampleTrajectory.States()) {
    std::cout << "  "
       << "t: " << state.t.value() << ", vel: " << state.velocity.value()
       << ", a: " << state.acceleration.value() << ", pose: ("
       << state.pose.X().value() << "," << state.pose.Y().value() << ")"
       << std::endl;
  }


  return std::move(ramseteCommand)
      .BeforeStarting(
          frc2::cmd::RunOnce([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}))
      // Because Mr. Healy is professionally paranoid....
      .AndThen(frc2::cmd::RunOnce([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}

void RobotContainer::AddTestButtonsOnSmartDashboard(){
  frc::SmartDashboard::PutData("3volts left, -3Volts right, 3 sec", new DriveFromVoltageForTime(&m_drive, 3_V, -3_V, 3_s));
  frc::SmartDashboard::PutData("3volts both, 3 sec", new DriveFromVoltageForTime(&m_drive, 3_V, 3_V, 3_s));
  frc::SmartDashboard::PutData("Reset Encoders", new frc2::InstantCommand([this]() { m_drive.ResetEncoders(); }, {&m_drive}));
}
