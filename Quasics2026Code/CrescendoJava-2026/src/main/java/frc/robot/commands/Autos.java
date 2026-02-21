// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Trajectorygenerator.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonomousScore2Options;
import frc.robot.Constants.AutonomousScore3Options;
import frc.robot.Constants.AutonomousSelectedOperation;
import frc.robot.Constants.AutonomousStartingPositions;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TransitionRoller;

public final class Autos {
  private final static double shooterSpeed = 0.75;
  private final static double intakeSpeed = 0.5;
  private final static double transitionSpeed = 0.5;

  public static Command resetOdometryToStartingPosition(
      Drivebase drivebase, String position, String color) {
    Pose2d startingPose;
    if (position == AutonomousStartingPositions.leftOfSpeaker)
      startingPose = GetTrajectoryInitialPose(color + "1bgo");
    else if (position == AutonomousStartingPositions.inFrontOfSpeaker)
      startingPose = GetTrajectoryInitialPose(color + "2go");
    else if (position == AutonomousStartingPositions.rightOfSpeaker)
      startingPose = GetTrajectoryInitialPose(color + "3ago");
    else if (position == AutonomousStartingPositions.farField)
      startingPose = GetTrajectoryInitialPose(color + "3bgo");
    else {
      // ???
      startingPose = new Pose2d();
    }
    // return new PrintCommand("Doing nothing");
    return new SetRobotOdometry(drivebase, startingPose);
  }

  public static Command intakeHelperCommand(
      IntakeRoller intakeRoller, TransitionRoller transitionRoller) {
    return Commands.race(new RunTransitionUntilBeamBroken(
                             transitionRoller, transitionSpeed, true, true),
        new RunIntake(intakeRoller, intakeSpeed, true));
  }

  public static Command shootingSequence(
      TransitionRoller transitionRoller, Shooter shooter) {
    return Commands.parallel(transitionDelay(transitionRoller),
        new TimedRunShooter(shooter, shooterSpeed, Seconds.of(2.0), true));
  }

  public static Command transitionDelay(TransitionRoller transitionRoller) {
    return Commands.sequence(new WaitCommand(0.75),
        new TimedRunTransitionRoller(
            transitionRoller, transitionSpeed, Seconds.of(1.25), true));
  }

  public static Command shootingSequenceWithoutWait(
      TransitionRoller transitionRoller, Shooter shooter) {
    return Commands.parallel(new TimedRunTransitionRoller(transitionRoller,
                                 transitionSpeed, Seconds.of(1.25), true),
        new TimedRunShooter(shooter, shooterSpeed, Seconds.of(1.25), true));
  }

  public static Command intakeWhileDriving(Drivebase drivebase,
      IntakeRoller intakeRoller, TransitionRoller transitionRoller,
      String pathName) {
    return Commands.parallel(GetCommandForTrajectory(pathName, drivebase),
        intakeHelperCommand(intakeRoller, transitionRoller));
  }

  public static Command runShooterWhileDriving(
      Drivebase drivebase, Shooter shooter, String pathName) {
    return Commands.race(GetCommandForTrajectory(pathName, drivebase),
        new RunShooter(shooter, shooterSpeed, true));
  }

  /** Example static factory for an autonomous command. */
  public static Command GTFO(
      Drivebase drivebase, String position, String color) {
    String path = "";
    if (position == AutonomousStartingPositions.leftOfSpeaker)
      path = color + "1bgo";
    else if (position == AutonomousStartingPositions.inFrontOfSpeaker)
      path = color + "2go";
    else if (position == AutonomousStartingPositions.rightOfSpeaker)
      path = color + "3ago";
    else if (position == AutonomousStartingPositions.farField)
      path = color + "3bgo";

    return GetCommandForTrajectory(path, drivebase);
  }

  public static Command score1(
      TransitionRoller transitionRoller, Shooter shooter) {
    // nothing for amp right now
    return shootingSequence(transitionRoller, shooter);
  }

  public static Command score1GTFO(Drivebase drivebase,
      TransitionRoller transitionRoller, Shooter shooter,
      IntakeRoller intakeRoller, String position, String color) {
    return Commands.sequence(
        score1(transitionRoller, shooter), GTFO(drivebase, position, color));
  }

  public static Command score2InFrontOfSpeaker(Drivebase drivebase,
      TransitionRoller transitionRoller, Shooter shooter,
      IntakeRoller intakeRoller, String color) {
    return Commands.sequence(intakeWhileDriving(drivebase, intakeRoller,
                                 transitionRoller, color + "2tonote2"),
        runShooterWhileDriving(drivebase, shooter, color + "note2to2"),
        shootingSequenceWithoutWait(transitionRoller, shooter));
  }

  public static Command score2LeftOfSpeaker(Drivebase drivebase,
      TransitionRoller transitionRoller, Shooter shooter,
      IntakeRoller intakeRoller, String color) {
    return Commands.sequence(intakeWhileDriving(drivebase, intakeRoller,
                                 transitionRoller, color + "1btonote1"),
        runShooterWhileDriving(drivebase, shooter, color + "note1to1b"),
        shootingSequenceWithoutWait(transitionRoller, shooter));
  }

  public static Command score2RightOfSpeaker(Drivebase drivebase,
      TransitionRoller transitionRoller, Shooter shooter,
      IntakeRoller intakeRoller, String scoreOption, String color) {
    if (scoreOption == AutonomousScore2Options.rightOfSpeakerAllianceNote) {
      return Commands.sequence(intakeWhileDriving(drivebase, intakeRoller,
                                   transitionRoller, color + "3atonote3"),
          runShooterWhileDriving(drivebase, shooter, color + "note3to3a"),
          shootingSequenceWithoutWait(transitionRoller, shooter));
    } else if (scoreOption
        == AutonomousScore2Options.rightOfSpeakerCenterNote) {
      return Commands.sequence(intakeWhileDriving(drivebase, intakeRoller,
                                   transitionRoller, color + "3atonote8"),
          runShooterWhileDriving(drivebase, shooter, color + "note8to3a"),
          shootingSequenceWithoutWait(transitionRoller, shooter));
    }
    return new PrintCommand("???");
  }

  public static Command score2(Drivebase drivebase, String position,
      TransitionRoller transitionRoller, Shooter shooter,
      IntakeRoller intakeRoller, String scoreOption, String color) {
    if (position == AutonomousStartingPositions.leftOfSpeaker) {
      return Commands.sequence(score1(transitionRoller, shooter),
          score2LeftOfSpeaker(
              drivebase, transitionRoller, shooter, intakeRoller, color));
    } else if (position == AutonomousStartingPositions.inFrontOfSpeaker) {
      return Commands.sequence(score1(transitionRoller, shooter),
          score2InFrontOfSpeaker(
              drivebase, transitionRoller, shooter, intakeRoller, color));
    } else if (position == AutonomousStartingPositions.rightOfSpeaker) {
      return Commands.sequence(score1(transitionRoller, shooter),
          score2RightOfSpeaker(drivebase, transitionRoller, shooter,
              intakeRoller, scoreOption, color));
    } else if (position == AutonomousStartingPositions.farField) {
      // cant shoot from far field
      return score1(transitionRoller, shooter);
    }
    // default option?
    return score1(transitionRoller, shooter);
  }

  public static Command score3LeftOfSpeaker(Drivebase drivebase,
      TransitionRoller transitionRoller, Shooter shooter,
      IntakeRoller intakeRoller, String color) {
    return Commands.sequence(score2LeftOfSpeaker(drivebase, transitionRoller,
                                 shooter, intakeRoller, color),
        intakeWhileDriving(
            drivebase, intakeRoller, transitionRoller, color + "1btonote4"),
        runShooterWhileDriving(drivebase, shooter, color + "note4to1b"),
        shootingSequenceWithoutWait(transitionRoller, shooter));
  }

  public static Command score3InFrontOfSpeaker(Drivebase drivebase,
      TransitionRoller transitionRoller, Shooter shooter,
      IntakeRoller intakeRoller, String scoreOption, String color) {
    if (scoreOption == AutonomousScore3Options.inFrontOfSpeakerAmpNote) {
      return Commands.sequence(
          score2InFrontOfSpeaker(
              drivebase, transitionRoller, shooter, intakeRoller, color),
          intakeWhileDriving(
              drivebase, intakeRoller, transitionRoller, color + "2tonote1"),
          runShooterWhileDriving(drivebase, shooter, color + "note1to2"),
          shootingSequenceWithoutWait(transitionRoller, shooter));
    }

    else if (scoreOption
        == AutonomousScore3Options.inFrontOfSpeakerCenterNote) {
      return Commands.sequence(
          score2InFrontOfSpeaker(
              drivebase, transitionRoller, shooter, intakeRoller, color),
          intakeWhileDriving(
              drivebase, intakeRoller, transitionRoller, color + "2tonote6"),
          runShooterWhileDriving(drivebase, shooter, color + "note6to2"),
          shootingSequenceWithoutWait(transitionRoller, shooter));
    }

    else if (scoreOption == AutonomousScore3Options.inFrontOfSpeakerStageNote) {
      return Commands.sequence(
          score2InFrontOfSpeaker(
              drivebase, transitionRoller, shooter, intakeRoller, color),
          intakeWhileDriving(
              drivebase, intakeRoller, transitionRoller, color + "2tonote3"),
          runShooterWhileDriving(drivebase, shooter, color + "note3to2"),
          shootingSequenceWithoutWait(transitionRoller, shooter));
    }

    // defualt
    return score2InFrontOfSpeaker(
        drivebase, transitionRoller, shooter, intakeRoller, color);
  }

  public static Command score3RightOfSpeaker(Drivebase drivebase,
      TransitionRoller transitionRoller, Shooter shooter,
      IntakeRoller intakeRoller, String color) {
    return Commands.sequence(
        score2RightOfSpeaker(drivebase, transitionRoller, shooter, intakeRoller,
            AutonomousScore2Options.rightOfSpeakerAllianceNote, color),
        intakeWhileDriving(
            drivebase, intakeRoller, transitionRoller, color + "3atonote8"),
        runShooterWhileDriving(drivebase, shooter, color + "note8to3a"),
        shootingSequenceWithoutWait(transitionRoller, shooter));
  }

  public static Command score3(Drivebase drivebase, String position,
      TransitionRoller transitionRoller, Shooter shooter,
      IntakeRoller intakeRoller, String scoreOption, String color) {
    if (position == AutonomousStartingPositions.leftOfSpeaker) {
      return score3LeftOfSpeaker(
          drivebase, transitionRoller, shooter, intakeRoller, color);
    } else if (position == AutonomousStartingPositions.inFrontOfSpeaker) {
      return score3InFrontOfSpeaker(drivebase, transitionRoller, shooter,
          intakeRoller, scoreOption, color);
    } else if (position == AutonomousStartingPositions.rightOfSpeaker) {
      return score3RightOfSpeaker(
          drivebase, transitionRoller, shooter, intakeRoller, color);
    } else {
      return new PrintCommand("?");
    }
  }

  public static Command getAutonomousCommand(Drivebase drivebase,
      IntakeRoller intakeRoller, TransitionRoller transitionRoller,
      Shooter shooter, String overallOperation, String positionOption,
      String score2Option, String score3Option, boolean isBlue) {
    // ArrayList<Command> commands = new ArrayList<Command>();
    String color;
    if (isBlue) {
      color = "blue";
    } else {
      color = "red";
    }
    if (overallOperation == AutonomousSelectedOperation.doNothing) {
      return new PrintCommand("Doing Nothing");
    } else if (overallOperation == AutonomousSelectedOperation.GTFO) {
      return Commands.sequence(
          resetOdometryToStartingPosition(drivebase, positionOption, color),
          GTFO(drivebase, positionOption, color));
    } else if (overallOperation == AutonomousSelectedOperation.score1) {
      return Commands.sequence(
          resetOdometryToStartingPosition(drivebase, positionOption, color),
          score1(transitionRoller, shooter));
    } else if (overallOperation == AutonomousSelectedOperation.score1GTFO) {
      return Commands.sequence(
          resetOdometryToStartingPosition(drivebase, positionOption, color),
          score1GTFO(drivebase, transitionRoller, shooter, intakeRoller,
              positionOption, color));
    } else if (overallOperation == AutonomousSelectedOperation.score2) {
      return Commands.sequence(
          resetOdometryToStartingPosition(drivebase, positionOption, color),
          score2(drivebase, positionOption, transitionRoller, shooter,
              intakeRoller, score2Option, color));
    } else if (overallOperation == AutonomousSelectedOperation.score2GTFO) {
      return Commands.sequence(
          resetOdometryToStartingPosition(drivebase, positionOption, color),
          score2(drivebase, positionOption, transitionRoller, shooter,
              intakeRoller, score2Option, color),
          GTFO(drivebase, positionOption, color));
    }
    // return Commands.sequence(commands);
    return new PrintCommand("???");
  }
}
