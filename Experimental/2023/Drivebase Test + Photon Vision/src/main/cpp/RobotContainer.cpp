// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <iostream>

#include "TrajectoryGenerator.h"
#include "commands/DriveFromVoltageForTime.h"
#include "commands/Autos.h"
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

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

/*frc2::CommandPtr DriveInALine(units::meter_t distance, bool reversed, Drivebase *drivebase) {
frc::Pose2d startingPose = drivebase.GetEstimatedPose();
  units::degree_t startingAngle = startingPose.Rotation().Degrees();
  units::meter_t endingXPosition =
      startingPose.X() + distance * cos(startingAngle.value());
  units::meter_t endingYPosition =
      startingPose.Y() + distance * sin(startingAngle.value());

  frc::TrajectoryConfig config{0.2_mps, 0.4_mps_sq};
  if (distance < 0_m) {
    std::cout << "Activating This" << std::endl;
    config.SetReversed(true);
  }
return BuildTrajectory(startingPose, frc::Pose2d(endingXPosition, endingYPosition, startingAngle), true, drivebase);

}*/




frc2::CommandPtr RobotContainer::DriveInALineUsingAprilTags(units::meter_t distance) {
frc::Pose2d startingPose = m_drivebase.GetEstimatedPose();
  units::degree_t startingAngle = startingPose.Rotation().Degrees();
  /*
  units::meter_t endingXPosition =
      startingPose.X() + distance * cos(startingAngle.value());
  units::meter_t endingYPosition =
      startingPose.Y() + distance * sin(startingAngle.value());*/
  
  units::meter_t endingXPosition = startingPose.X() + distance * startingPose.Rotation().Cos();
  units::meter_t endingYPosition = startingPose.Y() + distance * startingPose.Rotation().Sin();

  frc::TrajectoryConfig config{0.2_mps, 0.4_mps_sq};
  bool driveForward = true;
  if (distance < 0_m) {
    driveForward = false;
  }
  std::cout<<"**********NEW RUN***********"<<std::endl;
  std::cout<<"*"<<std::endl;
  std::cout<<"*"<<std::endl;
  std::cout<<"*"<<std::endl;
  std::cout<<"StartingPose (" << startingPose.X().value() << ", " << startingPose.Y().value() << ")" << std::endl;
  std::cout<<"StartingAngle: " << startingAngle.value() << std::endl;
  std::cout<<"EndingPose (" << endingXPosition.value() << ", " << endingYPosition.value() << ")" << std::endl;
return BuildTrajectoryUsingAprilTags(startingPose, frc::Pose2d(endingXPosition, endingYPosition, startingAngle), driveForward, &m_drivebase);

}







/*
frc2::SequentialCommandGroup *RobotContainer::DriveStraightLineAprilTag(
    units::meter_t distance) {
  frc::Pose2d startingPose = m_drivebase.GetEstimatedPose();
  units::degree_t startingAngle = startingPose.Rotation().Degrees();
  units::meter_t endingXPosition =
      startingPose.X() + distance * cos(startingAngle.value());
  units::meter_t endingYPosition =
      startingPose.Y() + distance * sin(startingAngle.value());

  frc::TrajectoryConfig config{0.2_mps, 0.4_mps_sq};
  if (distance < 0_m) {
    std::cout << "Activating This" << std::endl;
    config.SetReversed(true);
  }
  std::vector<std::unique_ptr<frc2::Command>> commands;


commands.push_back(
      std::unique_ptr<frc2::Command>(new BuildTrajectory(startingPose, frc::Pose2d(endingXPosition, endingYPosition, startingAngle), true, &m_drivebase)));


  return new frc2::SequentialCommandGroup(std::move(commands));
}*/


frc2::CommandPtr RobotContainer::TestManualBuild(){
  std::vector<frc::Translation2d> test;
  //test.push_back(frc::Translation2d{0_m, 0.25_m});
  //test.push_back(frc::Translation2d{0_m, 0.50_m});
  return ManuallyBuildTrajectoryUsingStandardOdometry(frc::Pose2d(0_m,0_m,0_deg), test, frc::Pose2d(1_m,0_m,0_deg), true, &m_drivebase);
}

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_drivebase.SetDefaultCommand(frc2::InstantCommand([this]{m_drivebase.TankDriveVolts(0_V, 0_V);}, {&m_drivebase}) );

  // Configure the button bindings
  ConfigureBindings();
  AddTestButtonsOnSmartDashboard();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  //return DriveInALine(1_m);
  //return DriveInALineUsingAprilTags(-1_m);
  return TestManualBuild();
}

void RobotContainer::AddTestButtonsOnSmartDashboard(){
  frc::SmartDashboard::PutData("3volts left, -3Volts right, 3 sec", new DriveFromVoltageForTime(&m_drivebase, 3_V, -3_V, 3_s));
  frc::SmartDashboard::PutData("3volts both, 3 sec", new DriveFromVoltageForTime(&m_drivebase, 3_V, 3_V, 3_s));
  frc::SmartDashboard::PutData("Reset Encoders", new frc2::InstantCommand([this]() { m_drivebase.ResetEncoders(); }, {&m_drivebase}));
}
