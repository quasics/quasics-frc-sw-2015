// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "TrajectoryGenerator.h"
#include "commands/RunShooter.h"
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
    frc2::CommandPtr resetOdometryToStartingPosition(IDrivebase &drivebase,
                                                     std::string position) {
      frc::Pose2d startingPose;
      if (position == AutonomousTeamAndStationPositions::inFrontOfAmp)
        startingPose = GetTrajectoryInitialPose("blue1ago.wpilib.json");
      else if (position == AutonomousTeamAndStationPositions::leftOfSpeaker)
        startingPose = GetTrajectoryInitialPose("blue1bgo.wpilib.json");
      else if (position == AutonomousTeamAndStationPositions::inFrontOfSpeaker)
        startingPose = GetTrajectoryInitialPose("blue2go.wpilib.json");
      else if (position == AutonomousTeamAndStationPositions::rightOfSpeaker)
        startingPose = GetTrajectoryInitialPose("blue3ago.wpilib.json");
      else if (position == AutonomousTeamAndStationPositions::farField)
        startingPose = GetTrajectoryInitialPose("blue3bgo.wpilib.json");

      return SetRobotOdometry(drivebase, startingPose).ToPtr();
    }
    /*
        frc2::CommandPtr blue1aAmp1AmpGo(IDrivebase &drivebase) {
          std::vector<frc2::CommandPtr> commands;
          frc::Pose2d pose;
          pose = GetTrajectoryInitialPose("blue1atoamp.wpilib.json");
          commands.push_back(
              std::move(frc2::CommandPtr(SetRobotOdometry(drivebase, pose))));
          commands.push_back(std::move(frc2::CommandPtr(
              GetCommandForTrajectory("blue1atoamp.wpilib.json", drivebase))));

          commands.push_back(std::move(frc2::CommandPtr(
              GetCommandForTrajectory("blueamptonote1.wpilib.json",
       drivebase))));

          commands.push_back(std::move(frc2::CommandPtr(
              GetCommandForTrajectory("bluenote1toamp.wpilib.json",
       drivebase))));

          commands.push_back(std::move(frc2::CommandPtr(
              GetCommandForTrajectory("blueampleave.wpilib.json", drivebase))));
          return frc2::SequentialCommandGroup(
                     frc2::CommandPtr::UnwrapVector(std::move(commands)))
              .ToPtr();
        }
        */

    frc2::CommandPtr backwardTest(IDrivebase &drivebase) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(std::move(frc2::CommandPtr(
          GetCommandForTrajectory("backward.wpilib.json", drivebase))));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr GTFO(IDrivebase &drivebase, std::string position) {
      std::string path;
      if (position == AutonomousTeamAndStationPositions::inFrontOfAmp)
        path = "blue1ago.wpilib.json";
      else if (position == AutonomousTeamAndStationPositions::leftOfSpeaker)
        path = "blue1bgo.wpilib.json";
      else if (position == AutonomousTeamAndStationPositions::inFrontOfSpeaker)
        path = "blue2go.wpilib.json";
      else if (position == AutonomousTeamAndStationPositions::rightOfSpeaker)
        path = "blue3ago.wpilib.json";
      else if (position == AutonomousTeamAndStationPositions::farField)
        path = "blue3bgo.wpilib.json";

      std::vector<frc2::CommandPtr> commands;
      commands.push_back(std::move(
          frc2::CommandPtr(GetCommandForTrajectory(path, drivebase))));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr score1Command(Shooter &shooter, std::string position) {
      if (position == AutonomousTeamAndStationPositions::inFrontOfAmp) {
        return RunShooter(shooter, ShootingSpeeds::amp, true).ToPtr();
      } else if (position == AutonomousTeamAndStationPositions::leftOfSpeaker ||
                 position ==
                     AutonomousTeamAndStationPositions::inFrontOfSpeaker ||
                 position ==
                     AutonomousTeamAndStationPositions::rightOfSpeaker) {
        return RunShooter(shooter, ShootingSpeeds::speaker, true).ToPtr();
      } else if (position == AutonomousTeamAndStationPositions::farField) {
        frc2::PrintCommand doNothing("Can't shoot from far field!");
        return std::move(doNothing).ToPtr();
      }
    }

    frc2::CommandPtr score1GTFO(IDrivebase &drivebase, Shooter &shooter,
                                std::string position) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(score1Command(shooter, position));
      commands.push_back(GTFO(drivebase, position));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

  }  // namespace Helpers

  frc2::CommandPtr GetAutonomousCommand(IDrivebase &drivebase, Shooter &shooter,
                                        std::string operationName,
                                        std::string position,
                                        std::string score2Dest,
                                        std::string score3Dest) {
    using namespace Helpers;
    std::vector<frc2::CommandPtr> commands;
    commands.push_back(resetOdometryToStartingPosition(drivebase, position));

    if (operationName == AutonomousSelectedOperation::doNothing) {
      frc2::PrintCommand doNothing("Doing nothing, as instructed");
      commands.push_back(std::move(doNothing).ToPtr());
    } else if (operationName == AutonomousSelectedOperation::GTFO) {
      commands.push_back(GTFO(drivebase, position));
    } else if (operationName == AutonomousSelectedOperation::score1) {
      commands.push_back(score1Command(shooter, position));
    } else if (operationName == AutonomousSelectedOperation::score1GTFO) {
      commands.push_back(score1GTFO(drivebase, shooter, position));
    } else if (operationName == AutonomousSelectedOperation::score2) {
      // TODO
      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());
    } else if (operationName == AutonomousSelectedOperation::score2GTFO) {
      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());
    } else if (operationName == AutonomousSelectedOperation::score3) {
      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());
    } else if (operationName == AutonomousSelectedOperation::score3GTFO) {
      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());
    } else {
      static frc2::PrintCommand fallThroughCaseCommand(
          "*** Error: don't know what to do, based on "
          "selections!");
      return std::move(fallThroughCaseCommand).ToPtr();
    }

    return frc2::SequentialCommandGroup(
               frc2::CommandPtr::UnwrapVector(std::move(commands)))
        .ToPtr();
  }
}  // namespace AutonomousCommands