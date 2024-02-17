// Copyright (c) 2024 Quasics, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <iostream>

#include "Constants.h"
#include "TrajectoryGenerator.h"
#include "commands/PivotIntakeAuto.h"
#include "commands/RunIntake.h"
#include "commands/RunIntakeTimed.h"
#include "commands/RunShooter.h"
#include "commands/RunShooterTimed.h"
#include "commands/SetRobotOdometry.h"
#include "commands/Wait.h"

namespace AutonomousCommands {

  namespace Helpers {
    frc2::CommandPtr resetOdometryToStartingPosition(IDrivebase &drivebase,
                                                     std::string position,
                                                     bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      frc::Pose2d startingPose;
      if (position == AutonomousStartingPositions::inFrontOfAmp)
        startingPose = GetTrajectoryInitialPose(color + "1ago.wpilib.json");
      else if (position == AutonomousStartingPositions::leftOfSpeaker)
        startingPose = GetTrajectoryInitialPose(color + "1bgo.wpilib.json");
      else if (position == AutonomousStartingPositions::inFrontOfSpeaker)
        startingPose = GetTrajectoryInitialPose(color + "2go.wpilib.json");
      else if (position == AutonomousStartingPositions::rightOfSpeaker)
        startingPose = GetTrajectoryInitialPose(color + "3ago.wpilib.json");
      else if (position == AutonomousStartingPositions::farField)
        startingPose = GetTrajectoryInitialPose(color + "3bgo.wpilib.json");

      return SetRobotOdometry(drivebase, startingPose).ToPtr();
    }

    frc2::CommandPtr IntakeDelay(IntakeRoller &intakeRoller) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(frc2::CommandPtr(Wait(0.75_s)));
      commands.push_back(
          frc2::CommandPtr(RunIntakeTimed(intakeRoller, .5, 1.25_s, false)));

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr ShootingSequence(IntakeDeployment &intakeDeployment,
                                      IntakeRoller &intakeRoller,
                                      Shooter &shooter, bool amp) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(frc2::CommandPtr(RunShooterTimed(
          shooter, (amp ? ShooterSpeeds::amp : ShooterSpeeds::speaker), 2_s,
          true)));
      commands.push_back(
          std::move(frc2::CommandPtr(IntakeDelay(intakeRoller))));

      return frc2::ParallelRaceGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr extendThenRunIntake(IntakeDeployment &intakeDeployment,
                                         IntakeRoller &intakeRoller) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(std::move(frc2::CommandPtr(PivotIntakeAuto(
          intakeDeployment, IntakeSpeeds::intakeDeploymentSpeed, true))));
      commands.push_back(std::move(frc2::CommandPtr(
          RunIntake(intakeRoller, IntakeSpeeds::intakeRollerSpeed, true))));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr retractIntakeThenPause(
        IntakeDeployment &intakeDeployment) {
      // This command retracts the intake then waits forever. It should only be
      // used within a ParallelRaceGroup
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(std::move(frc2::CommandPtr(PivotIntakeAuto(
          intakeDeployment, IntakeSpeeds::intakeDeploymentSpeed, true))));
      commands.push_back(std::move(frc2::CommandPtr(Wait(100_s))));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr intakeWhileDriving(IDrivebase &drivebase,
                                        IntakeDeployment &intakeDeployment,
                                        IntakeRoller &intakeRoller,
                                        std::string pathName) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(GetCommandForTrajectory(pathName, drivebase));
      commands.push_back(extendThenRunIntake(intakeDeployment, intakeRoller));
      return frc2::ParallelRaceGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr retractIntakeWhileDriving(
        IDrivebase &drivebase, IntakeDeployment &intakeDeployment,
        std::string pathName) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(GetCommandForTrajectory(pathName, drivebase));
      commands.push_back(retractIntakeThenPause(intakeDeployment));
      return frc2::ParallelRaceGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr runShooterWhileDriving(IDrivebase &drivebase,
                                            IntakeDeployment &intakeDeployment,
                                            IntakeRoller &intakeRoller,
                                            Shooter &shooter,
                                            std::string pathName, bool amp) {
      // if amp == true run amp speed. else run shooter speed
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(GetCommandForTrajectory(pathName, drivebase));
      commands.push_back(std::move(frc2::CommandPtr(
          ShootingSequence(intakeDeployment, intakeRoller, shooter, false))));

      return frc2::ParallelRaceGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr retractIntakeAndRunShooterWhileDriving(
        IDrivebase &drivebase, Shooter &shooter,
        IntakeDeployment &intakeDeployment, IntakeRoller &intakeRoller,
        std::string pathName, bool amp) {
      std::vector<frc2::CommandPtr> commands;

      commands.push_back(GetCommandForTrajectory(pathName, drivebase));
      commands.push_back(retractIntakeThenPause(intakeDeployment));
      commands.push_back(
          std::move(frc2::CommandPtr(RunShooter(shooter, 1.0, true))));

      return frc2::ParallelRaceGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr runIntakeThenPause(IntakeRoller &intakeRoller) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(std::move(frc2::CommandPtr(
          RunIntake(intakeRoller, IntakeSpeeds::feedingSpeed, true))));
      commands.push_back(std::move(
          frc2::CommandPtr(Wait(AutonomousTimes::waitTimeAfterShooting))));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr GTFO(IDrivebase &drivebase, std::string position,
                          bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::string path;
      if (position == AutonomousStartingPositions::inFrontOfAmp)
        path = color + "1ago.wpilib.json";
      else if (position == AutonomousStartingPositions::leftOfSpeaker)
        path = color + "1bgo.wpilib.json";
      else if (position == AutonomousStartingPositions::inFrontOfSpeaker)
        path = color + "2go.wpilib.json";
      else if (position == AutonomousStartingPositions::rightOfSpeaker)
        path = color + "3ago.wpilib.json";
      else if (position == AutonomousStartingPositions::farField)
        path = color + "3bgo.wpilib.json";
      else if (position == AutonomousScoreDestinations::amp) {
        path = color + "ampgo.wpilib.json";
      }

      return GetCommandForTrajectory(path, drivebase);
    }

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score1Command(IDrivebase &drivebase,
                                   IntakeDeployment &intakeDeployment,
                                   IntakeRoller &intakeRoller, Shooter &shooter,
                                   std::string position, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      if (position == AutonomousStartingPositions::inFrontOfAmp) {
        std::vector<frc2::CommandPtr> commands;
        commands.push_back(
            GetCommandForTrajectory(color + "1atoamp.wpilib.json", drivebase));

        commands.push_back(std::move(frc2::CommandPtr(
            ShootingSequence(intakeDeployment, intakeRoller, shooter, true))));

        return frc2::SequentialCommandGroup(
                   frc2::CommandPtr::UnwrapVector(std::move(commands)))
            .ToPtr();
      } else if (position == AutonomousStartingPositions::leftOfSpeaker ||
                 position == AutonomousStartingPositions::inFrontOfSpeaker ||
                 position == AutonomousStartingPositions::rightOfSpeaker) {
        return ShootingSequence(intakeDeployment, intakeRoller, shooter, false);
      } else {
        frc2::PrintCommand doNothing("Can't shoot from far field!");
        return std::move(doNothing).ToPtr();
      }
    }
#else
    frc2::CommandPtr score1Command(IDrivebase &drivebase, std::string position,
                                   bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      if (position == AutonomousStartingPositions::inFrontOfAmp) {
        std::vector<frc2::CommandPtr> commands;

        commands.push_back(
            GetCommandForTrajectory(color + "1atoamp.wpilib.json", drivebase));

        return frc2::SequentialCommandGroup(
                   frc2::CommandPtr::UnwrapVector(std::move(commands)))
            .ToPtr();
      }
      frc2::PrintCommand doNothing("cant do anything");
      return std::move(doNothing).ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score1GTFO(IDrivebase &drivebase,
                                IntakeDeployment &intakeDeployment,
                                IntakeRoller &intakeRoller, Shooter &shooter,
                                std::string position, bool isBlue) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(score1Command(drivebase, intakeDeployment,
                                       intakeRoller, shooter, position,
                                       isBlue));
      commands.push_back(GTFO(drivebase, position, isBlue));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score1GTFO(IDrivebase &drivebase, std::string position,
                                bool isBlue) {
      frc2::PrintCommand doNothing("Can't shoot from far field!");
      return std::move(doNothing).ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY

    frc2::CommandPtr score2InFrontOfSpeaker(IDrivebase &drivebase,
                                            Shooter &shooter,
                                            IntakeDeployment &intakeDeployment,
                                            IntakeRoller &intakeRoller,
                                            bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;
      commands.push_back(intakeWhileDriving(drivebase, intakeDeployment,
                                            intakeRoller,
                                            color + "2tonote2.wpilib.json"));
      commands.push_back(retractIntakeAndRunShooterWhileDriving(
          drivebase, shooter, intakeDeployment, intakeRoller,
          color + "note2to2.wpilib.json", false));
      commands.push_back(
          ShootingSequence(intakeDeployment, intakeRoller, shooter, false));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else

    frc2::CommandPtr score2InFrontOfSpeaker(IDrivebase &drivebase,
                                            bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      commands.push_back(
          GetCommandForTrajectory(color + "2tonote2.wpilib.json", drivebase));

      commands.push_back(
          GetCommandForTrajectory(color + "note2to2.wpilib.json", drivebase));

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score2RightOfSpeaker(IDrivebase &drivebase,
                                          Shooter &shooter,
                                          IntakeDeployment &intakeDeployment,
                                          IntakeRoller &intakeRoller,
                                          bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      commands.push_back(intakeWhileDriving(drivebase, intakeDeployment,
                                            intakeRoller,
                                            color + "3atonote3.wpilib.json"));
      commands.push_back(retractIntakeAndRunShooterWhileDriving(
          drivebase, shooter, intakeDeployment, intakeRoller,
          color + "note3to3a.wpilib.json", false));
      commands.push_back(
          ShootingSequence(intakeDeployment, intakeRoller, shooter, false));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score2RightOfSpeaker(IDrivebase &drivebase, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      commands.push_back(
          GetCommandForTrajectory(color + "3atonote3.wpilib.json", drivebase));
      commands.push_back(
          GetCommandForTrajectory(color + "note3to3a.wpilib.json", drivebase));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score2LeftOfSpeaker(IDrivebase &drivebase,
                                         Shooter &shooter,
                                         IntakeDeployment &intakeDeployment,
                                         IntakeRoller &intakeRoller,
                                         std::string score2Dest, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;
      commands.push_back(intakeWhileDriving(drivebase, intakeDeployment,
                                            intakeRoller,
                                            color + "1btonote1.wpilib.json"));
      if (score2Dest == AutonomousScoreDestinations::leftOfSpeaker) {
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, shooter, intakeDeployment, intakeRoller,
            color + "note1to1b.wpilib.json", false));
        commands.push_back(
            ShootingSequence(intakeDeployment, intakeRoller, shooter, false));
      } else if (score2Dest == AutonomousScoreDestinations::amp) {
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, shooter, intakeDeployment, intakeRoller,
            color + "note1toamp.wpilib.json", false));
        commands.push_back(
            ShootingSequence(intakeDeployment, intakeRoller, shooter, false));
      } else {
        frc2::PrintCommand doNothing(
            "Can only score in amp or left of speaker!");
        commands.push_back(std::move(doNothing).ToPtr());
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score2LeftOfSpeaker(IDrivebase &drivebase,
                                         std::string score2Dest, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      commands.push_back(
          GetCommandForTrajectory(color + "1btonote1.wpilib.json", drivebase));

      if (score2Dest == AutonomousScoreDestinations::leftOfSpeaker) {
        commands.push_back(GetCommandForTrajectory(
            color + "note1to1b.wpilib.json", drivebase));
      } else if (score2Dest == AutonomousScoreDestinations::amp) {
        commands.push_back(GetCommandForTrajectory(
            color + "note1toamp.wpilib.json", drivebase));
      } else {
        frc2::PrintCommand doNothing(
            "Can only score in amp or left of speaker!");
        commands.push_back(std::move(doNothing).ToPtr());
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score2InFrontOfAmp(IDrivebase &drivebase, Shooter &shooter,
                                        IntakeDeployment &intakeDeployment,
                                        IntakeRoller &intakeRoller,
                                        std::string score2Dest, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;
      commands.push_back(intakeWhileDriving(drivebase, intakeDeployment,
                                            intakeRoller,
                                            color + "amptonote1.wpilib.json"));
      if (score2Dest == AutonomousScoreDestinations::leftOfSpeaker) {
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, shooter, intakeDeployment, intakeRoller,
            color + "note1to1b.wpilib.json", true));
        commands.push_back(
            ShootingSequence(intakeDeployment, intakeRoller, shooter, false));
      } else if (score2Dest == AutonomousScoreDestinations::amp) {
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, shooter, intakeDeployment, intakeRoller,
            color + "note1toamp.wpilib.json", false));
        commands.push_back(
            ShootingSequence(intakeDeployment, intakeRoller, shooter, false));
      } else {
        frc2::PrintCommand doNothing(
            "Can only score in amp or left of speaker!");
        commands.push_back(std::move(doNothing).ToPtr());
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score2InFrontOfAmp(IDrivebase &drivebase,
                                        std::string score2Dest, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      commands.push_back(
          GetCommandForTrajectory(color + "amptonote1.wpilib.json", drivebase));

      if (score2Dest == AutonomousScoreDestinations::leftOfSpeaker) {
        commands.push_back(GetCommandForTrajectory(
            color + "note1to1b.wpilib.json", drivebase));

      } else if (score2Dest == AutonomousScoreDestinations::amp) {
        commands.push_back(GetCommandForTrajectory(
            color + "note1toamp.wpilib.json", drivebase));
      } else {
        frc2::PrintCommand doNothing(
            "Can only score in amp or left of speaker!");
        commands.push_back(std::move(doNothing).ToPtr());
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score2Command(IDrivebase &drivebase, Shooter &shooter,
                                   IntakeDeployment &intakeDeployment,
                                   IntakeRoller &intakeRoller,
                                   std::string position, std::string score2Dest,
                                   bool isBlue) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(score1Command(drivebase, intakeDeployment,
                                       intakeRoller, shooter, position,
                                       isBlue));
      commands.push_back(
          frc2::CommandPtr(PivotIntakeAuto(intakeDeployment, 0.5, true)));
      if (position == AutonomousStartingPositions::inFrontOfAmp) {
        commands.push_back(score2InFrontOfAmp(drivebase, shooter,
                                              intakeDeployment, intakeRoller,
                                              score2Dest, isBlue));

      } else if (position == AutonomousStartingPositions::leftOfSpeaker) {
        commands.push_back(score2LeftOfSpeaker(drivebase, shooter,
                                               intakeDeployment, intakeRoller,
                                               score2Dest, isBlue));

      } else if (position == AutonomousStartingPositions::inFrontOfSpeaker) {
        commands.push_back(score2InFrontOfSpeaker(
            drivebase, shooter, intakeDeployment, intakeRoller, isBlue));
      } else if (position == AutonomousStartingPositions::rightOfSpeaker) {
        commands.push_back(score2RightOfSpeaker(
            drivebase, shooter, intakeDeployment, intakeRoller, isBlue));
      } else if (position == AutonomousStartingPositions::farField) {
        frc2::PrintCommand doNothing("Can't shoot from far field!");
        commands.push_back(std::move(doNothing).ToPtr());
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score2Command(IDrivebase &drivebase, std::string position,
                                   std::string score2Dest, bool isBlue) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(score1Command(drivebase, position, isBlue));
      if (position == AutonomousStartingPositions::inFrontOfAmp) {
        commands.push_back(score2InFrontOfAmp(drivebase, score2Dest, isBlue));

      } else if (position == AutonomousStartingPositions::leftOfSpeaker) {
        commands.push_back(score2LeftOfSpeaker(drivebase, score2Dest, isBlue));

      } else if (position == AutonomousStartingPositions::inFrontOfSpeaker) {
        commands.push_back(score2InFrontOfSpeaker(drivebase, isBlue));
      } else if (position == AutonomousStartingPositions::rightOfSpeaker) {
        commands.push_back(score2RightOfSpeaker(drivebase, isBlue));
      } else if (position == AutonomousStartingPositions::farField) {
        frc2::PrintCommand doNothing("Can't shoot from far field!");
        commands.push_back(std::move(doNothing).ToPtr());
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score2GTFO(IDrivebase &drivebase, Shooter &shooter,
                                IntakeDeployment &intakeDeployment,
                                IntakeRoller &intakeRoller,
                                std::string position, std::string score2Dest,
                                bool isBlue) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(score2Command(drivebase, shooter, intakeDeployment,
                                       intakeRoller, position, score2Dest,
                                       isBlue));
      if (position == AutonomousStartingPositions::rightOfSpeaker ||
          position == AutonomousStartingPositions::inFrontOfSpeaker) {
        commands.push_back(GTFO(drivebase, position, isBlue));
      } else {
        commands.push_back(GTFO(drivebase, score2Dest, isBlue));
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score2GTFO(IDrivebase &drivebase, std::string position,
                                std::string score2Dest, bool isBlue) {
      std::vector<frc2::CommandPtr> commands;

      commands.push_back(
          score2Command(drivebase, position, score2Dest, isBlue));
      if (position == AutonomousStartingPositions::rightOfSpeaker ||
          position == AutonomousStartingPositions::inFrontOfSpeaker) {
        commands.push_back(GTFO(drivebase, position, isBlue));
      } else {
        commands.push_back(GTFO(drivebase, score2Dest, isBlue));
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score3InFrontOfAmp(IDrivebase &drivebase, Shooter &shooter,
                                        IntakeDeployment &intakeDeployment,
                                        IntakeRoller &intakeRoller,
                                        std::string score3Dest, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score3InFrontOfAmp(IDrivebase &drivebase,
                                        std::string score3Dest, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score3LeftOfSpeaker(IDrivebase &drivebase,
                                         Shooter &shooter,
                                         IntakeDeployment &intakeDeployment,
                                         IntakeRoller &intakeRoller,
                                         std::string score3Dest, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score3LeftOfSpeaker(IDrivebase &drivebase,
                                         std::string score3Dest, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score3InFrontOfSpeaker(IDrivebase &drivebase,
                                            Shooter &shooter,
                                            IntakeDeployment &intakeDeployment,
                                            IntakeRoller &intakeRoller,
                                            std::string score3Dest,
                                            bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score3InFrontOfSpeaker(IDrivebase &drivebase,
                                            std::string score3Dest,
                                            bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score3RightOfSpeaker(IDrivebase &drivebase,
                                          Shooter &shooter,
                                          IntakeDeployment &intakeDeployment,
                                          IntakeRoller &intakeRoller,
                                          std::string score3Dest, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score3RightOfSpeaker(IDrivebase &drivebase,
                                          std::string score3Dest, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score3Command(IDrivebase &drivebase, Shooter &shooter,
                                   IntakeDeployment &intakeDeployment,
                                   IntakeRoller &intakeRoller,
                                   std::string score2Dest,
                                   std::string score3Dest, std::string position,
                                   bool isBlue) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(score2Command(drivebase, shooter, intakeDeployment,
                                       intakeRoller, position, score2Dest,
                                       isBlue));
      if (position == AutonomousStartingPositions::inFrontOfAmp) {
        commands.push_back(score3InFrontOfAmp(drivebase, shooter,
                                              intakeDeployment, intakeRoller,
                                              score3Dest, isBlue));

      } else if (position == AutonomousStartingPositions::leftOfSpeaker) {
        commands.push_back(score3LeftOfSpeaker(drivebase, shooter,
                                               intakeDeployment, intakeRoller,
                                               score3Dest, isBlue));

      } else if (position == AutonomousStartingPositions::inFrontOfSpeaker) {
        commands.push_back(
            score3InFrontOfSpeaker(drivebase, shooter, intakeDeployment,
                                   intakeRoller, score3Dest, isBlue));
      } else if (position == AutonomousStartingPositions::rightOfSpeaker) {
        commands.push_back(score3RightOfSpeaker(drivebase, shooter,
                                                intakeDeployment, intakeRoller,
                                                score3Dest, isBlue));
      } else if (position == AutonomousStartingPositions::farField) {
        frc2::PrintCommand doNothing("Can't shoot from far field!");
        commands.push_back(std::move(doNothing).ToPtr());
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score3Command(IDrivebase &drivebase,
                                   std::string score2Dest,
                                   std::string score3Dest, std::string position,
                                   bool isBlue) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(
          score2Command(drivebase, position, score2Dest, isBlue));
      if (position == AutonomousStartingPositions::inFrontOfAmp) {
        commands.push_back(score3InFrontOfAmp(drivebase, score3Dest, isBlue));

      } else if (position == AutonomousStartingPositions::leftOfSpeaker) {
        commands.push_back(score3LeftOfSpeaker(drivebase, score3Dest, isBlue));

      } else if (position == AutonomousStartingPositions::inFrontOfSpeaker) {
        commands.push_back(
            score3InFrontOfSpeaker(drivebase, score3Dest, isBlue));
      } else if (position == AutonomousStartingPositions::rightOfSpeaker) {
        commands.push_back(score3RightOfSpeaker(drivebase, score3Dest, isBlue));
      } else if (position == AutonomousStartingPositions::farField) {
        frc2::PrintCommand doNothing("Can't shoot from far field!");
        commands.push_back(std::move(doNothing).ToPtr());
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score4Command(IDrivebase &drivebase, Shooter &shooter,
                                   IntakeDeployment &intakeDeployment,
                                   IntakeRoller &intakeRoller,
                                   std::string position, std::string score2Dest,
                                   std::string score3Dest, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;
      if (position != AutonomousStartingPositions::inFrontOfSpeaker) {
        return frc2::SequentialCommandGroup(
                   frc2::CommandPtr::UnwrapVector(std::move(commands)))
            .ToPtr();
      }

      commands.push_back(intakeWhileDriving(drivebase, intakeDeployment,
                                            intakeRoller,
                                            color + "2tonote1.wpilib.json"));
      commands.push_back(retractIntakeAndRunShooterWhileDriving(
          drivebase, shooter, intakeDeployment, intakeRoller,
          color + "note1to2.wpilib.json", false));
      commands.push_back(
          ShootingSequence(intakeDeployment, intakeRoller, shooter, false));
      commands.push_back(intakeWhileDriving(drivebase, intakeDeployment,
                                            intakeRoller,
                                            color + "2tonote1.wpilib.json"));
      commands.push_back(retractIntakeAndRunShooterWhileDriving(
          drivebase, shooter, intakeDeployment, intakeRoller,
          color + "note1to2.wpilib.json", false));
      commands.push_back(
          ShootingSequence(intakeDeployment, intakeRoller, shooter, false));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score4Command(IDrivebase &drivebase, std::string position,
                                   std::string score2Dest,
                                   std::string score3Dest, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(
          score2Command(drivebase, position, score2Dest, isBlue));

      if (position != AutonomousStartingPositions::inFrontOfSpeaker) {
        return frc2::SequentialCommandGroup(
                   frc2::CommandPtr::UnwrapVector(std::move(commands)))
            .ToPtr();
      }

      commands.push_back(
          GetCommandForTrajectory(color + "2tonote1.wpilib.json", drivebase));
      commands.push_back(
          GetCommandForTrajectory(color + "note1to2.wpilib.json", drivebase));
      commands.push_back(
          GetCommandForTrajectory(color + "2tonote3.wpilib.json", drivebase));
      commands.push_back(
          GetCommandForTrajectory(color + "note3to2.wpilib.json", drivebase));

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

  }  // namespace Helpers

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
  frc2::CommandPtr GetAutonomousCommand(IDrivebase &drivebase, Shooter &shooter,
                                        IntakeDeployment &intakeDeployment,
                                        IntakeRoller &intakeRoller,
                                        std::string operationName,
                                        std::string position,
                                        std::string score2Dest,
                                        std::string score3Dest, bool isBlue) {
    using namespace Helpers;
    std::vector<frc2::CommandPtr> commands;
    commands.push_back(
        resetOdometryToStartingPosition(drivebase, position, isBlue));

    if (operationName == AutonomousSelectedOperation::doNothing) {
      frc2::PrintCommand doNothing("Doing nothing, as instructed");
      commands.push_back(std::move(doNothing).ToPtr());
    } else if (operationName == AutonomousSelectedOperation::GTFO) {
      commands.push_back(GTFO(drivebase, position, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score1) {
      commands.push_back(score1Command(drivebase, intakeDeployment,
                                       intakeRoller, shooter, position,
                                       isBlue));
    } else if (operationName == AutonomousSelectedOperation::score1GTFO) {
      commands.push_back(score1GTFO(drivebase, intakeDeployment, intakeRoller,
                                    shooter, position, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score2) {
      commands.push_back(score2Command(drivebase, shooter, intakeDeployment,
                                       intakeRoller, position, score2Dest,
                                       isBlue));
    } else if (operationName == AutonomousSelectedOperation::score2GTFO) {
      commands.push_back(score2GTFO(drivebase, shooter, intakeDeployment,
                                    intakeRoller, position, score2Dest,
                                    isBlue));
    } else if (operationName == AutonomousSelectedOperation::score3) {
      commands.push_back(score3Command(drivebase, shooter, intakeDeployment,
                                       intakeRoller, position, score2Dest,
                                       score3Dest, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score3GTFO) {
      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());
    } else if (operationName == AutonomousSelectedOperation::score4) {
      commands.push_back(score4Command(drivebase, shooter, intakeDeployment,
                                       intakeRoller, position, score2Dest,
                                       score3Dest, isBlue));
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
#else

  frc2::CommandPtr GetAutonomousCommand(IDrivebase &drivebase,
                                        std::string operationName,
                                        std::string position,
                                        std::string score2Dest,
                                        std::string score3Dest, bool isBlue) {
    using namespace Helpers;
    std::vector<frc2::CommandPtr> commands;
    commands.push_back(
        resetOdometryToStartingPosition(drivebase, position, isBlue));

    if (operationName == AutonomousSelectedOperation::doNothing) {
      frc2::PrintCommand doNothing("Doing nothing, as instructed");
      commands.push_back(std::move(doNothing).ToPtr());
    } else if (operationName == AutonomousSelectedOperation::GTFO) {
      commands.push_back(GTFO(drivebase, position, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score1) {
      commands.push_back(score1Command(drivebase, position, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score1GTFO) {
      commands.push_back(score1GTFO(drivebase, position, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score2) {
      commands.push_back(
          score2Command(drivebase, position, score2Dest, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score2GTFO) {
      commands.push_back(score2GTFO(drivebase, position, score2Dest, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score3) {
      commands.push_back(
          score3Command(drivebase, position, score2Dest, score3Dest, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score3GTFO) {
      frc2::PrintCommand notImplemented("Not implemented");
      commands.push_back(std::move(notImplemented).ToPtr());
    } else if (operationName == AutonomousSelectedOperation::score4) {
      commands.push_back(
          score4Command(drivebase, position, score2Dest, score3Dest, isBlue));
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
#endif

}  // namespace AutonomousCommands