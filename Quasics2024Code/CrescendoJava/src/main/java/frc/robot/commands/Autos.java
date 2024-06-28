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
import frc.robot.Constants.AutonomousSelectedOperation;
import frc.robot.Constants.AutonomousStartingPositions;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TransitionRoller;

public final class Autos {
  public static Command resetOdometryToStartingPosition(
      Drivebase drivebase, String position, String color) {
    Pose2d startingPose;
    if (position == AutonomousStartingPositions.inFrontOfAmp)
      startingPose = GetTrajectoryInitialPose(color + "1ago");
    else if (position == AutonomousStartingPositions.leftOfSpeaker)
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
    //return new PrintCommand("Doing nothing");
    return new SetRobotOdometry(drivebase, startingPose);
  }

  public static Command intakeHelperCommand(
      IntakeRoller intakeRoller, TransitionRoller transitionRoller) {
    return Commands.parallel(new RunTransitionRoller(transitionRoller, 0.5, true),
        new RunIntake(intakeRoller, 0.5, true));
  }

  public static Command shootingSequence(
      TransitionRoller transitionRoller, Shooter shooter, IntakeRoller intakeRoller) {
    return Commands.parallel(transitionDelay(transitionRoller, intakeRoller),
        new TimedRunShooter(shooter, 0.75, Seconds.of(2.0), true));
  }

  public static Command secondaryHelper(
      TransitionRoller transitionRoller, IntakeRoller intakeRoller) {
    return Commands.parallel(
        new TimedRunTransitionRoller(transitionRoller, .5, Seconds.of(2.0), false),
        new TimedRunIntake(intakeRoller, .6, Seconds.of(2.0), false));
  }

  public static Command transitionDelay(
      TransitionRoller transitionRoller, IntakeRoller intakeRoller) {
    return Commands.sequence(
        new WaitCommand(0.75), secondaryHelper(transitionRoller, intakeRoller));
  }

  public static Command shootingSequenceWithoutWait(
      TransitionRoller transitionRoller, Shooter shooter) {
    return Commands.parallel(
        new TimedRunTransitionRoller(transitionRoller, .5, Seconds.of(1.25), true),
        new TimedRunShooter(shooter, 0.5, Seconds.of(1.25), true));
  }

  public static Command intakeWhileDriving(Drivebase drivebase, IntakeRoller intakeRoller,
      TransitionRoller transitionRoller, String pathName) {
    return Commands.race(GetCommandForTrajectory(pathName, drivebase),
        intakeHelperCommand(intakeRoller, transitionRoller));
  }

  public static Command runShooterWhileDriving(
      Drivebase drivebase, Shooter shooter, String pathName) {
    return Commands.race(
        GetCommandForTrajectory(pathName, drivebase), new RunShooter(shooter, 0.5, true));
  }

  public static Command intakeWhileDrivingCommand(Drivebase drivebase, Shooter shooter) {
    // untested
    return Commands.race(new TimedMovementTest(drivebase, Seconds.of(1), -0.30));
  }

  /** Example static factory for an autonomous command. */
  public static Command GTFO(Drivebase drivebase, String position, String color) {
    String path = "";
    if (position == AutonomousStartingPositions.inFrontOfAmp)
      path = color + "1ago";
    else if (position == AutonomousStartingPositions.leftOfSpeaker)
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
      TransitionRoller transitionRoller, Shooter shooter, IntakeRoller intakeRoller, String color) {
    // nothing for amp right now
    return shootingSequence(transitionRoller, shooter, intakeRoller);
  }



  public static Command getAutonomousCommand(Drivebase drivebase, IntakeRoller intakeRoller,
      TransitionRoller transitionRoller, Shooter shooter, String overallOperation,
      String positionOption, String score2Option, String score3Option, boolean isBlue) {
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
      return Commands.sequence(resetOdometryToStartingPosition(drivebase, positionOption, color),
          GTFO(drivebase, positionOption, color));
    } else if (overallOperation == AutonomousSelectedOperation.score1) {
      return Commands.sequence(resetOdometryToStartingPosition(drivebase, positionOption, color),
          score1(transitionRoller, shooter, intakeRoller, color));
    } else if (overallOperation == AutonomousSelectedOperation.score1GTFO) {
      return Commands.sequence(resetOdometryToStartingPosition(drivebase, positionOption, color),
          score1(transitionRoller, shooter, intakeRoller, color), new WaitCommand(0.5),
          GTFO(drivebase, positionOption, color));
    }
    // return Commands.sequence(commands);
    return new PrintCommand("???");
  }
}
