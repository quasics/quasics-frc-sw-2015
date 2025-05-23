// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonomousSelectedOperation;
import frc.robot.Constants.AutonomousStartingPositions;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ArmRoller;
import frc.robot.subsystems.drivebase.AbstractDrivebase;
import frc.robot.subsystems.elevator.AbstractElevator;
import frc.robot.subsystems.elevator.AbstractElevator.TargetPosition;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {

  /*
   * IMPORTANT POSITIONS FOR CHOREO
   * Robot size: 0.851 m by 0.993 m
   * Blue 1: (7.556m, 6.169m, 180deg)
   * Blue 2: (7.556m, 4.026, 180deg)
   * Blue 3: (7.556m, 1.883m, 180deg)
   * Top of reef: (5.230, 5.132, -120deg)
   * Middle of reef: (5.817m, 3.937m, 180deg)
   * Bottom of reef: (5.076m, 2.831m, 120deg)
   * Barge: (7.556m (needs testing), ?m, 0deg)
   * Processor: (6.340m, 0.796m, -90deg)
   */

  static final double DIST_TO_REEF = 2.134;

  // CODE_REVIEW/FIXME: Generally, static functions don't set/change static data
  // members as you're currently doing. Usually, one of three things will happen:
  //
  // 1. The data needed by any function is explicitly passed into it, and if one
  // static function calls another, then it passes the data through as a part of
  // the call. (This is the most common pattern; see how the drivebase is
  // currently being passed around as an example.)
  //
  // 2. There's a single (or rarely, multiple) static "configuration" methods that
  // are used to set shared/static data, which must be invoked before anything
  // else.
  //
  // 3. Instead of using static functions, you'd set the class up as a "normal"
  // object with a constructor, and then allocate one of those as your factory
  // object, after which you'd call the (non-static) functions as normal.
  private static AutoFactory m_autoFactory;
  private static AbstractDrivebase m_drivebase;
  private static ArmPivot m_armPivot;
  private static AbstractElevator m_elevator;
  private static ArmRoller m_armRoller;
  private static String m_positionOption;

  private static final Angle REEF_ALGAE_ANGLE = Degrees.of(33);
  private static final Angle FIELD_ALGAE_ANGLE = Degrees.of(23);

  public static Command followPath(String pathName,
      boolean resetOdometry, boolean stopAtEnd) {

    return Commands.sequence(
        resetOdometry ? m_autoFactory.resetOdometry(pathName) : Commands.none(),
        resetOdometry ? m_autoFactory.resetOdometry(pathName) : Commands.none(),
        m_autoFactory.trajectoryCmd(pathName),
        stopAtEnd ? new InstantCommand(() -> m_drivebase.stop()) : Commands.none());
  }

  public static Command runCommandAfterTime(Command command, double time) {
    return Commands.sequence(new WaitCommand(time), command);
  }

  public static Command moveBackwards() {
    return new DriveForDistance(m_drivebase, 0.2, Meters.of(-1));
  }

  private static Command intakeThenExtake() {
    return Commands.sequence(new RunKrakenForTime(m_armRoller, -0.3, 0.2), new WaitCommand(0.1),
        new RunKrakenForTime(m_armRoller, 1.0, 0.5));
  }

  private static Command extakeInProcessor() {
    return new RunKrakenForTime(m_armRoller, 0.5, 0.5);
  }

  public static Command GTFO() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.TOP:
        return followPath("1gtfo", true, true);
      case AutonomousStartingPositions.MIDDLE:
        return followPath("2gtfo", true, true);
      case AutonomousStartingPositions.BOTTOM:
        return followPath("3gtfo", true, true);
      case AutonomousStartingPositions.VERY_TOP:
        return followPath("4gtfo", true, true);
      case AutonomousStartingPositions.VERY_BOTTOM:
        return followPath("5gtfo", true, true);
      default:
        return new PrintCommand("GTFO failed?");
    }
  }

  public static Command goToReef() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.TOP:
        return followPath("1toreef", true, true);
      case AutonomousStartingPositions.MIDDLE:
        return followPath("2toreef", true, true);
      case AutonomousStartingPositions.BOTTOM:
        return followPath("3toreef", true, true);
      default:
        return new PrintCommand("goToReef failed?");
    }
  }

  public static Command goToField() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.VERY_TOP:
        return followPath("4toalgae", true, true);
      case AutonomousStartingPositions.VERY_BOTTOM:
        return followPath("5toalgae", true, true);
      default:
        return new PrintCommand("goToField failed?");
    }
  }

  public static Command grabAlgaeFromReef() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.TOP:
        return Commands.parallel(
            followPath("1toreef", true, true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, REEF_ALGAE_ANGLE), 0.0),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kL2),
                0.4),
            runCommandAfterTime(new RunKrakenForTime(m_armRoller, -0.3, 0.5), 2.1));
      case AutonomousStartingPositions.MIDDLE:
        return Commands.parallel(
            followPath("2toreef", true, true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, REEF_ALGAE_ANGLE),
                0.0),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kL1),
                0.2),
            runCommandAfterTime(new RunKrakenForTime(m_armRoller, -0.3, 0.5), 1.6));
      case AutonomousStartingPositions.BOTTOM:
        return Commands.parallel(
            followPath("3toreef", true, true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, REEF_ALGAE_ANGLE), 0.0),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kL2),
                0.4),
            runCommandAfterTime(new RunKrakenForTime(m_armRoller, -0.3, 0.6), 2.2));
      default:
        return new PrintCommand("grabAlgaeFromReef failed?");
    }
  }

  public static Command grabAlgaeFromField() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.VERY_TOP:
        return Commands.parallel(followPath("4toalgae", true, true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, FIELD_ALGAE_ANGLE), 0.5),
            runCommandAfterTime(new RunKrakenForTime(m_armRoller, -0.3, 0.7), 2.8));
      case AutonomousStartingPositions.VERY_BOTTOM:
        return Commands.parallel(followPath("5toalgae", true, true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, FIELD_ALGAE_ANGLE), 0.5),
            runCommandAfterTime(new RunKrakenForTime(m_armRoller, -0.3, 0.7), 2.8));
      default:
        return new PrintCommand("grabAlgaeFromField failed?");
    }
  }

  public static Command scoreAtFieldIntoBarge() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.VERY_TOP:
        return Commands.parallel(followPath("verytopfieldtobarge", false, true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(95)), 1.0),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kTop), 4.5),
            Commands.sequence(
                Commands.race(runCommandAfterTime(new PulseKraken(m_armRoller, -0.3, 0.3, 0.3), 0),
                    new WaitCommand(5.0)),
                runCommandAfterTime(intakeThenExtake(), 3)));
      default:
        return new PrintCommand("scoreAtFieldIntoBarge failed?");
    }
  }

  public static Command scoreAtFieldIntoProcessor() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.VERY_BOTTOM:
        return Commands.parallel(followPath("verybottomfieldtoprocessor", false, true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(21)), 2.0),
            Commands.sequence(
                Commands.race(runCommandAfterTime(new PulseKraken(m_armRoller, -0.3, 0.3, 0.3), 0),
                    new WaitCommand(4.0)),
                runCommandAfterTime(new RunKrakenForTime(m_armRoller, .5, .5), 0.4)));
      default:
        return new PrintCommand("scoreAtFieldIntoProcessor failed?");
    }
  }

  public static Command scoreCoralInReef() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.TOP:
        return Commands.parallel(
            followPath("1toreefcoral", true, true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(0)), 0.0),
            Commands.sequence(
                runCommandAfterTime(new RunKrakenForTime(m_armRoller, 0, 2), 0),
                runCommandAfterTime(new RunKrakenForTime(m_armRoller, -0.15, 0.4), 0)));
      case AutonomousStartingPositions.MIDDLE:
        return Commands.parallel(
            followPath("2toreefcoral", true, true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(0)),
                0.0),
            Commands.sequence(
                runCommandAfterTime(new RunKrakenForTime(m_armRoller, 0, 2), 0),
                runCommandAfterTime(new RunKrakenForTime(m_armRoller, -0.15, 0.5), 0)));
      case AutonomousStartingPositions.BOTTOM:
        return Commands.parallel(
            followPath("3toreefcoral", true, true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(0)), 0.0),
            Commands.sequence(
                runCommandAfterTime(new RunKrakenForTime(m_armRoller, 0, 2), 0),
                runCommandAfterTime(new RunKrakenForTime(m_armRoller, -0.15, 0.4), 0)));
      default:
        return new PrintCommand("scoreCoralInReef failed?");
    }
  }

  public static Command scoreAtReefIntoBarge() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.TOP:
        return Commands.parallel(
            followPath("topreeftobarge", false, true),
            Commands.sequence(runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kBottom), 0.5),
                runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kTop), 3.3),
                runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kBottom), 4.1)),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(95)), 1.0),
            Commands.sequence(
                new RunKrakenForTime(m_armRoller, -0.3, 1),
                Commands.race(runCommandAfterTime(new PulseKraken(m_armRoller, -0.3, 0.3, 0.5), 0.0),
                    new WaitCommand(3.5)),
                runCommandAfterTime(intakeThenExtake(), 2.8)));
      case AutonomousStartingPositions.MIDDLE:
        return Commands.parallel(
            followPath("middlereeftobarge", false, true),
            Commands.sequence(runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kBottom), 0.5),
                runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kTop), 3.3),
                runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kBottom), 4.1)),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(95)), 1.0),
            Commands.sequence(
                Commands.race(runCommandAfterTime(new PulseKraken(m_armRoller, -0.3, 0.3, 0.5), 0.0),
                    new WaitCommand(4.5)),
                runCommandAfterTime(intakeThenExtake(), 2.7)));
      default:
        return new PrintCommand("scoreAlgaeFromReefIntoBarge failed?");
    }
  }

  public static Command scoreAtReefIntoProcessor() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.MIDDLE:
        return Commands.parallel(
            followPath("middlereeftoprocessor", false, true),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kBottom), 0.7),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(21)), 0.3),
            Commands.sequence(
                Commands.race(runCommandAfterTime(new PulseKraken(m_armRoller, -0.3, 0.3, 0.3), 0.0),
                    new WaitCommand(3.0)),
                runCommandAfterTime(extakeInProcessor(), 1.5)));
      case AutonomousStartingPositions.BOTTOM:
        return Commands.parallel(
            followPath("bottomreeftoprocessor", false, true),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kBottom), 0.1),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(21)), 0.3),
            Commands.sequence(
                new RunKrakenForTime(m_armRoller, -0.3, 1),
                Commands.race(runCommandAfterTime(new PulseKraken(m_armRoller, -0.3, 0.3, 0.3), 0.0),
                    new WaitCommand(2.5)),
                runCommandAfterTime(extakeInProcessor(), 1)));
      default:
        return new PrintCommand("scoreAlgaeFromReefIntoProcessor");
    }
  }

  public static Command scoreAlgaeFromReefIntoBarge() {
    return Commands.sequence(grabAlgaeFromReef(), scoreAtReefIntoBarge(), moveBackwards());
  }

  public static Command scoreAlgaeFromReefIntoProcessor() {
    return Commands.sequence(grabAlgaeFromReef(), scoreAtReefIntoProcessor());
  }

  public static Command scoreAlgaeFromFieldIntoBarge() {
    return Commands.sequence(grabAlgaeFromField(), scoreAtFieldIntoBarge(), moveBackwards());
  }

  public static Command scoreAlgaeFromFieldIntoProcessor() {
    return Commands.sequence(grabAlgaeFromField(), scoreAtFieldIntoProcessor());
  }

  public static Command grabAlgaeFromCoralPosition() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.TOP:
        return Commands.parallel(
            Commands.sequence(new WaitCommand(0.5), followPath("1reefcoraltoreef", false, true)),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, REEF_ALGAE_ANGLE), 0.0),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kL2), 0.0),
            runCommandAfterTime(new RunKrakenForTime(m_armRoller, -0.3, 0.6), 1.4));
      case AutonomousStartingPositions.MIDDLE:
        return Commands.parallel(
            Commands.sequence(new WaitCommand(0.5), followPath("2reefcoraltoreef", false, true)),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, REEF_ALGAE_ANGLE), 0.0),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kL1), 0.0),
            runCommandAfterTime(new RunKrakenForTime(m_armRoller, -0.3, 0.6), 1.4));
      case AutonomousStartingPositions.BOTTOM:
        return Commands.parallel(
            Commands.sequence(new WaitCommand(0.5), followPath("3reefcoraltoreef", false, true)),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, REEF_ALGAE_ANGLE), 0.0),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kL2), 0.0),
            runCommandAfterTime(new RunKrakenForTime(m_armRoller, -0.3, 0.6), 1.4));
      default:
        return new PrintCommand("grabAlgaeFromCoralPosition failed?");
    }
  }

  public static Command scoreCoralThenGrabAlgae() {
    return Commands.sequence(scoreCoralInReef(), grabAlgaeFromCoralPosition());
  }

  public static Command scoreCoralThenScoreBarge() {
    return Commands.sequence(scoreCoralThenGrabAlgae(), scoreAtReefIntoBarge(), moveBackwards());
  }

  public static Command scoreCoralThenScoreProcessor() {
    return Commands.sequence(scoreCoralThenGrabAlgae(), scoreAtReefIntoProcessor(), moveBackwards());
  }

  /** Example static factory for an autonomous command. */
  public static Command getAutonomousCommand(
      AutoFactory autoFactory,
      AbstractDrivebase drivebase,
      AbstractElevator elevator,
      ArmPivot armPivot,
      ArmRoller armRoller,
      String operation,
      String positionOption) {
    m_autoFactory = autoFactory;
    m_drivebase = drivebase;
    m_elevator = elevator;
    m_armPivot = armPivot;
    m_armRoller = armRoller;
    m_positionOption = positionOption;

    if (operation == AutonomousSelectedOperation.DO_NOTHING) {
      return new PrintCommand("Doing nothing!");
    }
    if (operation == AutonomousSelectedOperation.GO_TO_REEF_DR) {
      return Commands.sequence(
          new DriveForDistance(drivebase, 0.20, Meters.of(DIST_TO_REEF)),
          new TurnForDegrees(drivebase, 60, -0.13),
          new DriveForDistance(drivebase, 0.20, Meters.of(1.6)),
          new TurnForDegrees(drivebase, -62, -0.14));
    }

    if (operation == AutonomousSelectedOperation.GO_TO_REEF) {
      return goToReef();
    }

    if (operation == AutonomousSelectedOperation.GTFO) {
      // return new DriveForDistance(drivebase, 0.20, Meters.of(DIST_TO_REEF));
      return GTFO();
    }

    if (operation == AutonomousSelectedOperation.GRAB_ALGAE_FROM_REEF) {
      return grabAlgaeFromReef();
    }

    if (operation == AutonomousSelectedOperation.SCORE_CORAL_IN_REEF) {
      return scoreCoralInReef();
    }

    if (operation == AutonomousSelectedOperation.SCORE_ALGAE_REEF_BARGE) {
      return scoreAlgaeFromReefIntoBarge();
    }

    if (operation == AutonomousSelectedOperation.SCORE_ALGAE_REEF_PROCESSOR) {
      return scoreAlgaeFromReefIntoProcessor();
    }

    if (operation == AutonomousSelectedOperation.SCORE_CORAL_GRAB_ALGAE) {
      return scoreCoralThenGrabAlgae();
    }

    if (operation == AutonomousSelectedOperation.SCORE_CORAL_SCORE_BARGE) {
      return scoreCoralThenScoreBarge();
    }

    if (operation == AutonomousSelectedOperation.SCORE_CORAL_SCORE_PROCESSOR) {
      return scoreCoralThenScoreProcessor();
    }

    if (operation == AutonomousSelectedOperation.GRAB_ALGAE_FROM_FIELD) {
      return grabAlgaeFromField();
    }

    if (operation == AutonomousSelectedOperation.SCORE_ALGAE_FIELD_BARGE) {
      return scoreAlgaeFromFieldIntoBarge();
    }

    if (operation == AutonomousSelectedOperation.SCORE_ALGAE_FIELD_PROCESSOR) {
      return scoreAlgaeFromFieldIntoProcessor();
    }

    return new PrintCommand("Doing nothing because no operation?");
  }
}
