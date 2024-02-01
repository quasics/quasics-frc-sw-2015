// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "TrajectoryGenerator.h"
#include "commands/SetRobotOdometry.h"

namespace AutonomousCommands {

  /*frc2::Command *backwardTest(IDrivebase &drivebase) {
    std::vector<frc2::CommandPtr> commands;
    frc::Pose2d pose;
    pose = GetTrajectoryInitialPose("backward.wpilib.json");
    commands.push_back(std::move(
        frc2::CommandPtr(SetRobotOdometry(drivebase, pose).ToPtr())));
    commands.push_back(std::move(frc2::CommandPtr(
        GetCommandForTrajectory("backward.wpilib.json", drivebase.get()))));
    return frc2::SequentialCommandGroup(
               frc2::CommandPtr::UnwrapVector(std::move(commands)))
        .ToPtr();
  }*/
  namespace Helpers {

    frc2::CommandPtr backwardTest(IDrivebase &drivebase) {
      std::vector<frc2::CommandPtr> commands;
      frc::Pose2d pose;
      pose = GetTrajectoryInitialPose("backward.wpilib.json");
      commands.push_back(
          std::move(frc2::CommandPtr(SetRobotOdometry(drivebase, pose))));
      commands.push_back(std::move(frc2::CommandPtr(
          GetCommandForTrajectory("backward.wpilib.json", drivebase))));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

  }  // namespace Helpers

  frc2::CommandPtr GetAutonomousCommand(IDrivebase &drivebase,
                                        std::string operationName,
                                        std::string teamAndPosName) {
    using namespace Helpers;

    if (operationName == AutonomousSelectedOperation::DoNothing) {
      frc2::PrintCommand doNothing("Doing nothing, as instructed");
      return std::move(doNothing).ToPtr();
    } else if (operationName == AutonomousSelectedOperation::ScoreTwiceGTFO) {
      return backwardTest(drivebase);
    }
    static frc2::PrintCommand fallThroughCaseCommand(
        "*** Error: don't know what to do, based on "
        "selections!");
    return std::move(fallThroughCaseCommand).ToPtr();
  }
}  // namespace AutonomousCommands