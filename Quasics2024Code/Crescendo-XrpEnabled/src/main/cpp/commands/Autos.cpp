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
#include "commands/PIDRotate.h"
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

    frc2::CommandPtr rotateToEndOfPath(IDrivebase &drivebase,
                                       std::string path) {
      // this will NOT move to the end of the path, only rotate to the correct
      // angle. The x, y coordinates of robot may still be incorrect
      frc::Pose2d finalPose = GetTrajectoryFinalPose(path);
      return frc2::CommandPtr(
          PIDRotate(drivebase, finalPose.Rotation().Degrees()));
    }

    frc2::CommandPtr intakeDelay(IntakeRoller &intakeRoller) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(frc2::CommandPtr(Wait(0.75_s)));
      commands.push_back(
          frc2::CommandPtr(RunIntakeTimed(intakeRoller, .5, 1.25_s, false)));

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr shootingSequence(IntakeDeployment &intakeDeployment,
                                      IntakeRoller &intakeRoller,
                                      Shooter &shooter, bool amp) {
      std::vector<frc2::CommandPtr> commands;
      if (amp) {
        // TODO
      } else {
        commands.push_back(frc2::CommandPtr(RunShooterTimed(
            shooter, (amp ? ShooterSpeeds::amp : ShooterSpeeds::speaker), 2_s,
            true)));
        commands.push_back(
            std::move(frc2::CommandPtr(intakeDelay(intakeRoller))));
      }
      return frc2::ParallelRaceGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr shootingSequenceWithoutWait(IntakeRoller &intakeRoller,
                                                 Shooter &shooter, bool amp) {
      std::vector<frc2::CommandPtr> commands;
      // this command is the same as shootingSequence, but will not wait to run
      // the shooter. Assumes the shooter has already been running before
      // calling this command
      if (amp) {
        // TODO
      } else {
        commands.push_back(frc2::CommandPtr(RunShooterTimed(
            shooter, (amp ? ShooterSpeeds::amp : ShooterSpeeds::speaker),
            1.25_s, true)));
        commands.push_back(
            frc2::CommandPtr(RunIntakeTimed(intakeRoller, .5, 1.25_s, false)));
      }
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
          intakeDeployment, IntakeSpeeds::intakeDeploymentSpeed, false))));
      commands.push_back(std::move(frc2::CommandPtr(Wait(100_s))));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr intakeWhileDriving(IDrivebase &drivebase,
                                        IntakeRoller &intakeRoller,
                                        std::string pathName) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(GetCommandForTrajectory(pathName, drivebase));
      commands.push_back(std::move(frc2::CommandPtr(
          RunIntake(intakeRoller, IntakeSpeeds::intakeRollerSpeed, true))));
      return frc2::ParallelRaceGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr extendThenRunIntakeWhileDriving(
        IDrivebase &drivebase, IntakeDeployment &intakeDeployment,
        IntakeRoller &intakeRoller, std::string pathName) {
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
      commands.push_back(frc2::CommandPtr(RunShooter(
          shooter, (amp ? ShooterSpeeds::amp : ShooterSpeeds::speaker), true)));

      return frc2::ParallelRaceGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

    frc2::CommandPtr retractIntakeAndRunShooterWhileDriving(
        IDrivebase &drivebase, IntakeDeployment &intakeDeployment,
        Shooter &shooter, std::string pathName, bool amp) {
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
      else if (position == AutonomousScore2Options::amp) {
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
        commands.push_back(runShooterWhileDriving(drivebase, intakeDeployment,
                                                  intakeRoller, shooter,
                                                  "1atoamp.wpilib.json", true));

        commands.push_back(std::move(frc2::CommandPtr(
            shootingSequenceWithoutWait(intakeRoller, shooter, true))));

        return frc2::SequentialCommandGroup(
                   frc2::CommandPtr::UnwrapVector(std::move(commands)))
            .ToPtr();
      } else if (position == AutonomousStartingPositions::leftOfSpeaker ||
                 position == AutonomousStartingPositions::inFrontOfSpeaker ||
                 position == AutonomousStartingPositions::rightOfSpeaker) {
        return shootingSequence(intakeDeployment, intakeRoller, shooter, false);
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
      frc2::PrintCommand doNothing("");
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
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(score1Command(drivebase, position, isBlue));
      if (position == AutonomousStartingPositions::inFrontOfAmp) {
        // scoring in amp first, so GTFO directly from amp
        commands.push_back(
            GTFO(drivebase, AutonomousScore2Options::amp, isBlue));
      } else {
        commands.push_back(GTFO(drivebase, position, isBlue));
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY

    frc2::CommandPtr score2InFrontOfSpeaker(IDrivebase &drivebase,
                                            IntakeDeployment &intakeDeployment,
                                            IntakeRoller &intakeRoller,
                                            Shooter &shooter, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;
      commands.push_back(extendThenRunIntakeWhileDriving(
          drivebase, intakeDeployment, intakeRoller,
          color + "2tonote2.wpilib.json"));
      commands.push_back(retractIntakeAndRunShooterWhileDriving(
          drivebase, intakeDeployment, shooter, color + "note2to2.wpilib.json",
          false));
      commands.push_back(
          shootingSequenceWithoutWait(intakeRoller, shooter, false));
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
                                          IntakeDeployment &intakeDeployment,
                                          IntakeRoller &intakeRoller,
                                          Shooter &shooter,
                                          std::string score2Option,
                                          bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;
      if (score2Option == AutonomousScore2Options::rightOfSpeakerAllianceNote) {
        commands.push_back(extendThenRunIntakeWhileDriving(
            drivebase, intakeDeployment, intakeRoller,
            color + "3atonote3.wpilib.json"));
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, intakeDeployment, shooter,
            color + "note3to3a.wpilib.json", false));
        commands.push_back(
            shootingSequenceWithoutWait(intakeRoller, shooter, false));
      } else if (score2Option ==
                 AutonomousScore2Options::rightOfSpeakerCenterNote) {
        commands.push_back(extendThenRunIntakeWhileDriving(
            drivebase, intakeDeployment, intakeRoller,
            color + "3atonote8.wpilib.json"));
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, intakeDeployment, shooter,
            color + "note8to3a.wpilib.json", false));
        commands.push_back(
            shootingSequenceWithoutWait(intakeRoller, shooter, false));
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score2RightOfSpeaker(IDrivebase &drivebase,
                                          std::string score2Option,
                                          bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;
      if (score2Option == AutonomousScore2Options::rightOfSpeakerAllianceNote) {
        commands.push_back(GetCommandForTrajectory(
            color + "3atonote3.wpilib.json", drivebase));
        commands.push_back(GetCommandForTrajectory(
            color + "note3to3a.wpilib.json", drivebase));
      } else if (score2Option ==
                 AutonomousScore2Options::rightOfSpeakerCenterNote) {
        commands.push_back(GetCommandForTrajectory(
            color + "3atonote8.wpilib.json", drivebase));
        commands.push_back(GetCommandForTrajectory(
            color + "note8to3a.wpilib.json", drivebase));
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score2LeftOfSpeaker(IDrivebase &drivebase,
                                         IntakeDeployment &intakeDeployment,
                                         IntakeRoller &intakeRoller,
                                         Shooter &shooter,
                                         std::string score2Option,
                                         bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;
      commands.push_back(extendThenRunIntakeWhileDriving(
          drivebase, intakeDeployment, intakeRoller,
          color + "1btonote1.wpilib.json"));
      if (score2Option == AutonomousScore2Options::leftOfSpeaker) {
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, intakeDeployment, shooter,
            color + "note1to1b.wpilib.json", false));
        commands.push_back(
            shootingSequenceWithoutWait(intakeRoller, shooter, false));
      } else if (score2Option == AutonomousScore2Options::amp) {
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, intakeDeployment, shooter,
            color + "note1toamp.wpilib.json", false));
        commands.push_back(
            shootingSequenceWithoutWait(intakeRoller, shooter, false));
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
                                         std::string score2Option,
                                         bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      commands.push_back(
          GetCommandForTrajectory(color + "1btonote1.wpilib.json", drivebase));

      if (score2Option == AutonomousScore2Options::leftOfSpeaker) {
        commands.push_back(GetCommandForTrajectory(
            color + "note1to1b.wpilib.json", drivebase));
      } else if (score2Option == AutonomousScore2Options::amp) {
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
    frc2::CommandPtr score2InFrontOfAmp(IDrivebase &drivebase,
                                        IntakeDeployment &intakeDeployment,
                                        IntakeRoller &intakeRoller,
                                        Shooter &shooter,
                                        std::string score2Option, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;
      commands.push_back(extendThenRunIntakeWhileDriving(
          drivebase, intakeDeployment, intakeRoller,
          color + "amptonote1.wpilib.json"));
      if (score2Option == AutonomousScore2Options::leftOfSpeaker) {
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, intakeDeployment, shooter,
            color + "note1to1b.wpilib.json", true));
        commands.push_back(
            shootingSequenceWithoutWait(intakeRoller, shooter, false));
      } else if (score2Option == AutonomousScore2Options::amp) {
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, intakeDeployment, shooter,
            color + "note1toamp.wpilib.json", false));
        commands.push_back(
            shootingSequenceWithoutWait(intakeRoller, shooter, false));
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
                                        std::string score2Option, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      commands.push_back(
          GetCommandForTrajectory(color + "amptonote1.wpilib.json", drivebase));

      if (score2Option == AutonomousScore2Options::leftOfSpeaker) {
        commands.push_back(GetCommandForTrajectory(
            color + "note1to1b.wpilib.json", drivebase));

      } else if (score2Option == AutonomousScore2Options::amp) {
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
    frc2::CommandPtr score2Command(IDrivebase &drivebase,
                                   IntakeDeployment &intakeDeployment,
                                   IntakeRoller &intakeRoller, Shooter &shooter,
                                   std::string position,
                                   std::string score2Option, bool isBlue) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(score1Command(drivebase, intakeDeployment,
                                       intakeRoller, shooter, position,
                                       isBlue));

      if (position == AutonomousStartingPositions::inFrontOfAmp) {
        commands.push_back(score2InFrontOfAmp(drivebase, intakeDeployment,
                                              intakeRoller, shooter,
                                              score2Option, isBlue));

      } else if (position == AutonomousStartingPositions::leftOfSpeaker) {
        commands.push_back(score2LeftOfSpeaker(drivebase, intakeDeployment,
                                               intakeRoller, shooter,
                                               score2Option, isBlue));

      } else if (position == AutonomousStartingPositions::inFrontOfSpeaker) {
        commands.push_back(score2InFrontOfSpeaker(
            drivebase, intakeDeployment, intakeRoller, shooter, isBlue));
      } else if (position == AutonomousStartingPositions::rightOfSpeaker) {
        commands.push_back(score2RightOfSpeaker(drivebase, intakeDeployment,
                                                intakeRoller, shooter,
                                                score2Option, isBlue));
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
                                   std::string score2Option, bool isBlue) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(score1Command(drivebase, position, isBlue));
      if (position == AutonomousStartingPositions::inFrontOfAmp) {
        commands.push_back(score2InFrontOfAmp(drivebase, score2Option, isBlue));

      } else if (position == AutonomousStartingPositions::leftOfSpeaker) {
        commands.push_back(
            score2LeftOfSpeaker(drivebase, score2Option, isBlue));

      } else if (position == AutonomousStartingPositions::inFrontOfSpeaker) {
        commands.push_back(score2InFrontOfSpeaker(drivebase, isBlue));
      } else if (position == AutonomousStartingPositions::rightOfSpeaker) {
        commands.push_back(
            score2RightOfSpeaker(drivebase, score2Option, isBlue));
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
    frc2::CommandPtr score2GTFO(IDrivebase &drivebase,
                                IntakeDeployment &intakeDeployment,
                                IntakeRoller &intakeRoller, Shooter &shooter,
                                std::string position, std::string score2Option,
                                bool isBlue) {
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(score2Command(drivebase, intakeDeployment,
                                       intakeRoller, shooter, position,
                                       score2Option, isBlue));
      if (position == AutonomousStartingPositions::rightOfSpeaker ||
          position == AutonomousStartingPositions::inFrontOfSpeaker) {
        commands.push_back(GTFO(drivebase, position, isBlue));
      } else {
        commands.push_back(GTFO(drivebase, score2Option, isBlue));
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score2GTFO(IDrivebase &drivebase, std::string position,
                                std::string score2Option, bool isBlue) {
      std::vector<frc2::CommandPtr> commands;

      commands.push_back(
          score2Command(drivebase, position, score2Option, isBlue));
      if (position == AutonomousStartingPositions::rightOfSpeaker ||
          position == AutonomousStartingPositions::inFrontOfSpeaker) {
        commands.push_back(GTFO(drivebase, position, isBlue));
      } else {
        commands.push_back(GTFO(drivebase, score2Option, isBlue));
      }
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score3LeftOfSpeaker(IDrivebase &drivebase,
                                         IntakeDeployment &intakeDeployment,
                                         IntakeRoller &intakeRoller,
                                         Shooter &shooter,
                                         std::string score3Option,
                                         bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      commands.push_back(
          score2Command(drivebase, intakeDeployment, intakeRoller, shooter,
                        AutonomousStartingPositions::leftOfSpeaker,
                        AutonomousScore2Options::leftOfSpeaker, isBlue));
      commands.push_back(extendThenRunIntakeWhileDriving(
          drivebase, intakeDeployment, intakeRoller,
          color + "1btonote4.wpilib.json"));

      if (score3Option == AutonomousScore3Options::leftOfSpeaker) {
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, intakeDeployment, shooter,
            color + "note4to1b.wpilib.json", false));
        commands.push_back(
            shootingSequenceWithoutWait(intakeRoller, shooter, false));
      }

      else if (score3Option == AutonomousScore3Options::amp) {
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, intakeDeployment, shooter,
            color + "note4toamp.wpilib.json", false));
        commands.push_back(
            shootingSequenceWithoutWait(intakeRoller, shooter, false));
      }

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score3LeftOfSpeaker(IDrivebase &drivebase,
                                         std::string score3Option,
                                         bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      commands.push_back(
          score2Command(drivebase, AutonomousStartingPositions::leftOfSpeaker,
                        AutonomousScore2Options::leftOfSpeaker, isBlue));
      commands.push_back(
          GetCommandForTrajectory(color + "1btonote4.wpilib.json", drivebase));

      if (score3Option == AutonomousScore3Options::leftOfSpeaker) {
        commands.push_back(GetCommandForTrajectory(
            color + "note4to1b.wpilib.json", drivebase));
      }

      else if (score3Option == AutonomousScore3Options::amp) {
        commands.push_back(GetCommandForTrajectory(
            color + "note4toamp.wpilib.json", drivebase));
      }

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score3InFrontOfSpeaker(IDrivebase &drivebase,
                                            IntakeDeployment &intakeDeployment,
                                            IntakeRoller &intakeRoller,
                                            Shooter &shooter,
                                            std::string score3Option,
                                            bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      commands.push_back(
          score2Command(drivebase, intakeDeployment, intakeRoller, shooter,
                        AutonomousStartingPositions::inFrontOfSpeaker,
                        AutonomousScore2Options::inFrontOfSpeaker, isBlue));

      if (score3Option == AutonomousScore3Options::amp) {
        commands.push_back(extendThenRunIntakeWhileDriving(
            drivebase, intakeDeployment, intakeRoller,
            color + "2tonote1.wpilib.json"));
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, intakeDeployment, shooter,
            color + "note1toamp.wpilib.json", true));
        commands.push_back(
            shootingSequenceWithoutWait(intakeRoller, shooter, true));

      }

      else if (score3Option ==
               AutonomousScore3Options::inFrontOfSpeakerAmpNote) {
        commands.push_back(extendThenRunIntakeWhileDriving(
            drivebase, intakeDeployment, intakeRoller,
            color + "2tonote1.wpilib.json"));
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, intakeDeployment, shooter,
            color + "note2to2.wpilib.json", false));
        commands.push_back(
            shootingSequenceWithoutWait(intakeRoller, shooter, false));
      }

      else if (score3Option ==
               AutonomousScore3Options::inFrontOfSpeakerCenterNote) {
        commands.push_back(extendThenRunIntakeWhileDriving(
            drivebase, intakeDeployment, intakeRoller,
            color + "2tonote6.wpilib.json"));
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, intakeDeployment, shooter,
            color + "note6to2.wpilib.json", false));
        commands.push_back(
            shootingSequenceWithoutWait(intakeRoller, shooter, false));
      }

      else if (score3Option ==
               AutonomousScore3Options::inFrontOfSpeakerStageNote) {
        commands.push_back(extendThenRunIntakeWhileDriving(
            drivebase, intakeDeployment, intakeRoller,
            color + "2tonote3.wpilib.json"));
        commands.push_back(retractIntakeAndRunShooterWhileDriving(
            drivebase, intakeDeployment, shooter,
            color + "note3to2.wpilib.json", false));
        commands.push_back(
            shootingSequenceWithoutWait(intakeRoller, shooter, false));
      }

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score3InFrontOfSpeaker(IDrivebase &drivebase,
                                            std::string score3Option,
                                            bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      commands.push_back(score2Command(
          drivebase, AutonomousStartingPositions::inFrontOfSpeaker,
          AutonomousScore2Options::inFrontOfSpeaker, isBlue));

      if (score3Option == AutonomousScore3Options::amp) {
        commands.push_back(
            GetCommandForTrajectory(color + "2tonote1.wpilib.json", drivebase));
        commands.push_back(GetCommandForTrajectory(
            color + "note1toamp.wpilib.json", drivebase));
      }

      else if (score3Option ==
               AutonomousScore3Options::inFrontOfSpeakerAmpNote) {
        commands.push_back(
            GetCommandForTrajectory(color + "2tonote1.wpilib.json", drivebase));
        commands.push_back(
            GetCommandForTrajectory(color + "note2to2.wpilib.json", drivebase));
      }

      else if (score3Option ==
               AutonomousScore3Options::inFrontOfSpeakerCenterNote) {
        commands.push_back(
            GetCommandForTrajectory(color + "2tonote6.wpilib.json", drivebase));
        commands.push_back(
            GetCommandForTrajectory(color + "note6to2.wpilib.json", drivebase));
      }

      else if (score3Option ==
               AutonomousScore3Options::inFrontOfSpeakerStageNote) {
        commands.push_back(
            GetCommandForTrajectory(color + "2tonote3.wpilib.json", drivebase));
        commands.push_back(
            GetCommandForTrajectory(color + "note3to2.wpilib.json", drivebase));
      }

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score3RightOfSpeaker(IDrivebase &drivebase,
                                          IntakeDeployment &intakeDeployment,
                                          IntakeRoller &intakeRoller,
                                          Shooter &shooter,
                                          std::string score3Option,
                                          bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");
      std::vector<frc2::CommandPtr> commands;

      commands.push_back(score2Command(
          drivebase, intakeDeployment, intakeRoller, shooter,
          AutonomousStartingPositions::rightOfSpeaker,
          AutonomousScore2Options::rightOfSpeakerAllianceNote, isBlue));
      commands.push_back(extendThenRunIntakeWhileDriving(
          drivebase, intakeDeployment, intakeRoller,
          color + "3atonote8.wpilib.json"));
      commands.push_back(retractIntakeAndRunShooterWhileDriving(
          drivebase, intakeDeployment, shooter, color + "note8to3a.wpilib.json",
          false));
      commands.push_back(
          shootingSequenceWithoutWait(intakeRoller, shooter, false));

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score3RightOfSpeaker(IDrivebase &drivebase,
                                          std::string score3Option,
                                          bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;

      commands.push_back(score2Command(
          drivebase, AutonomousStartingPositions::rightOfSpeaker,
          AutonomousScore2Options::rightOfSpeakerAllianceNote, isBlue));
      commands.push_back(
          GetCommandForTrajectory(color + "3atonote8.wpilib.json", drivebase));
      commands.push_back(
          GetCommandForTrajectory(color + "note8to3a.wpilib.json", drivebase));

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score3Command(IDrivebase &drivebase,
                                   IntakeDeployment &intakeDeployment,
                                   IntakeRoller &intakeRoller, Shooter &shooter,
                                   std::string score3Option,
                                   std::string position, bool isBlue) {
      std::vector<frc2::CommandPtr> commands;
      // does not run score2Command here. score2Command is called within the
      // command for each position
      if (position == AutonomousStartingPositions::inFrontOfAmp) {
        // there are no score 3 commands that go to amp... default to score2?
        commands.push_back(
            score2InFrontOfAmp(drivebase, intakeDeployment, intakeRoller,
                               shooter, AutonomousScore2Options::amp, isBlue));

      } else if (position == AutonomousStartingPositions::leftOfSpeaker) {
        commands.push_back(score3LeftOfSpeaker(drivebase, intakeDeployment,
                                               intakeRoller, shooter,
                                               score3Option, isBlue));

      } else if (position == AutonomousStartingPositions::inFrontOfSpeaker) {
        commands.push_back(score3InFrontOfSpeaker(drivebase, intakeDeployment,
                                                  intakeRoller, shooter,
                                                  score3Option, isBlue));
      } else if (position == AutonomousStartingPositions::rightOfSpeaker) {
        commands.push_back(score3RightOfSpeaker(drivebase, intakeDeployment,
                                                intakeRoller, shooter,
                                                score3Option, isBlue));
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
                                   std::string score3Option,
                                   std::string position, bool isBlue) {
      std::vector<frc2::CommandPtr> commands;
      // DOES NOT RUN score2Command HERE
      if (position == AutonomousStartingPositions::inFrontOfAmp) {
        // there are no score 3 commands that go to amp... default to score2?
        commands.push_back(score2InFrontOfAmp(
            drivebase, AutonomousScore2Options::amp, isBlue));

      } else if (position == AutonomousStartingPositions::leftOfSpeaker) {
        commands.push_back(
            score3LeftOfSpeaker(drivebase, score3Option, isBlue));

      } else if (position == AutonomousStartingPositions::inFrontOfSpeaker) {
        commands.push_back(
            score3InFrontOfSpeaker(drivebase, score3Option, isBlue));
      } else if (position == AutonomousStartingPositions::rightOfSpeaker) {
        commands.push_back(
            score3RightOfSpeaker(drivebase, score3Option, isBlue));
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
    frc2::CommandPtr score3GTFO(IDrivebase &drivebase,
                                IntakeDeployment &intakeDeployment,
                                IntakeRoller &intakeRoller, Shooter &shooter,
                                std::string score3Option, std::string position,
                                bool isBlue) {
      std::vector<frc2::CommandPtr> commands;

      commands.push_back(score3Command(drivebase, intakeDeployment,
                                       intakeRoller, shooter, score3Option,
                                       position, isBlue));

      if (score3Option == AutonomousScore3Options::amp) {
        commands.push_back(
            GTFO(drivebase, AutonomousScore2Options::amp, isBlue));
      }

      else if (score3Option == AutonomousScore3Options::leftOfSpeaker) {
        commands.push_back(GTFO(
            drivebase, AutonomousStartingPositions::leftOfSpeaker, isBlue));
      }

      else if (score3Option ==
                   AutonomousScore3Options::inFrontOfSpeakerAmpNote ||
               score3Option ==
                   AutonomousScore3Options::inFrontOfSpeakerCenterNote ||
               score3Option ==
                   AutonomousScore3Options::inFrontOfSpeakerStageNote) {
        commands.push_back(GTFO(
            drivebase, AutonomousStartingPositions::inFrontOfSpeaker, isBlue));
      }

      else if (score3Option == AutonomousScore3Options::rightOfSpeaker) {
        commands.push_back(GTFO(
            drivebase, AutonomousStartingPositions::rightOfSpeaker, isBlue));
      }

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score3GTFO(IDrivebase &drivebase, std::string score3Option,
                                std::string position, bool isBlue) {
      std::vector<frc2::CommandPtr> commands;

      commands.push_back(
          score3Command(drivebase, score3Option, position, isBlue));

      if (score3Option == AutonomousScore3Options::amp) {
        commands.push_back(
            GTFO(drivebase, AutonomousScore2Options::amp, isBlue));
      }

      else if (score3Option == AutonomousScore3Options::leftOfSpeaker) {
        commands.push_back(GTFO(
            drivebase, AutonomousStartingPositions::leftOfSpeaker, isBlue));
      }

      else if (score3Option ==
                   AutonomousScore3Options::inFrontOfSpeakerAmpNote ||
               score3Option ==
                   AutonomousScore3Options::inFrontOfSpeakerCenterNote ||
               score3Option ==
                   AutonomousScore3Options::inFrontOfSpeakerStageNote) {
        commands.push_back(GTFO(
            drivebase, AutonomousStartingPositions::inFrontOfSpeaker, isBlue));
      }

      else if (score3Option == AutonomousScore3Options::rightOfSpeaker) {
        commands.push_back(GTFO(
            drivebase, AutonomousStartingPositions::rightOfSpeaker, isBlue));
      }

      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }
#endif

#ifdef ENABLE_FULL_ROBOT_FUNCTIONALITY
    frc2::CommandPtr score4Command(IDrivebase &drivebase,
                                   IntakeDeployment &intakeDeployment,
                                   IntakeRoller &intakeRoller, Shooter &shooter,
                                   std::string position,
                                   std::string score2Option,
                                   std::string score3Option, bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");

      std::vector<frc2::CommandPtr> commands;
      commands.push_back(score2Command(drivebase, intakeDeployment,
                                       intakeRoller, shooter, position,
                                       score2Option, isBlue));
      if (position != AutonomousStartingPositions::inFrontOfSpeaker) {
        return frc2::SequentialCommandGroup(
                   frc2::CommandPtr::UnwrapVector(std::move(commands)))
            .ToPtr();
      }

      commands.push_back(extendThenRunIntakeWhileDriving(
          drivebase, intakeDeployment, intakeRoller,
          color + "2tonote1.wpilib.json"));
      commands.push_back(retractIntakeAndRunShooterWhileDriving(
          drivebase, intakeDeployment, shooter, color + "note1to2.wpilib.json",
          false));
      commands.push_back(
          shootingSequenceWithoutWait(intakeRoller, shooter, false));
      commands.push_back(extendThenRunIntakeWhileDriving(
          drivebase, intakeDeployment, intakeRoller,
          color + "2tonote1.wpilib.json"));
      commands.push_back(retractIntakeAndRunShooterWhileDriving(
          drivebase, intakeDeployment, shooter, color + "note1to2.wpilib.json",
          false));
      commands.push_back(
          shootingSequenceWithoutWait(intakeRoller, shooter, false));
      return frc2::SequentialCommandGroup(
                 frc2::CommandPtr::UnwrapVector(std::move(commands)))
          .ToPtr();
    }

#else
    frc2::CommandPtr score4Command(IDrivebase &drivebase, std::string position,
                                   bool isBlue) {
      std::string color = (isBlue ? "blue" : "red");
      std::vector<frc2::CommandPtr> commands;
      commands.push_back(
          score2Command(drivebase, position,
                        AutonomousScore2Options::inFrontOfSpeaker, isBlue));

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
  frc2::CommandPtr GetAutonomousCommand(
      IDrivebase &drivebase, IntakeDeployment &intakeDeployment,
      IntakeRoller &intakeRoller, Shooter &shooter, std::string operationName,
      std::string position, std::string score2Option, std::string score3Option,
      bool isBlue) {
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
      commands.push_back(score2Command(drivebase, intakeDeployment,
                                       intakeRoller, shooter, position,
                                       score2Option, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score2GTFO) {
      commands.push_back(score2GTFO(drivebase, intakeDeployment, intakeRoller,
                                    shooter, position, score2Option, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score3) {
      commands.push_back(score3Command(drivebase, intakeDeployment,
                                       intakeRoller, shooter, position,
                                       score3Option, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score3GTFO) {
      commands.push_back(score3GTFO(drivebase, intakeDeployment, intakeRoller,
                                    shooter, position, score3Option, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score4) {
      commands.push_back(score4Command(drivebase, intakeDeployment,
                                       intakeRoller, shooter, position,
                                       score2Option, score3Option, isBlue));
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
                                        std::string score2Option,
                                        std::string score3Option, bool isBlue) {
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
          score2Command(drivebase, position, score2Option, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score2GTFO) {
      commands.push_back(score2GTFO(drivebase, position, score2Option, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score3) {
      commands.push_back(
          score3Command(drivebase, score3Option, position, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score3GTFO) {
      commands.push_back(score3GTFO(drivebase, score3Option, position, isBlue));
    } else if (operationName == AutonomousSelectedOperation::score4) {
      commands.push_back(score4Command(drivebase, position, isBlue));
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