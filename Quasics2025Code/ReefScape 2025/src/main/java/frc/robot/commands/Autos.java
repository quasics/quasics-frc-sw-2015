// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonomousSelectedOperation;
import frc.robot.Constants.AutonomousStartingPositions;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ArmRoller;
import frc.robot.subsystems.drivebase.AbstractDrivebase;
import frc.robot.subsystems.elevator.AbstractElevator;
import frc.robot.subsystems.elevator.AbstractElevator.TargetPosition;

public final class Autos {

  /*
   * IMPORTANT POSITIONS FOR CHOREO
   * Robot size: 0.851 m by 0.993 m
   * Blue 1: (7.556m, 6.169m, 180deg)
   * Blue 2: (7.556m, 4.026m, 180deg)
   * Blue 3: (7.556m, 1.883m, 180deg)
   * Top of reef: (5.153m, 5.176m, -120deg)
   * Middle of reef: (5.817m, 4.026m, 180deg)
   * Bottom of reef: (5.153m, 2.876m, 120deg)
   * Barge: (7.556m (needs testing), ?m, 0deg)
   * Processor: (6.340m (needs testing), 0.496m, -90deg)
   */

  static final double DIST_TO_REEF = 2.134;
  private static AutoFactory m_autoFactory;
  private static AbstractDrivebase m_drivebase;
  private static ArmPivot m_armPivot;
  private static AbstractElevator m_elevator;
  private static ArmRoller m_armRoller;
  private static String m_positionOption;

  public static Command followPath(String pathName,
      boolean resetOdometry) {

    return Commands.sequence(
        resetOdometry ? m_autoFactory.resetOdometry(pathName) : Commands.none(),
        resetOdometry ? m_autoFactory.resetOdometry(pathName) : Commands.none(),
        m_autoFactory.trajectoryCmd(pathName));
  }

  public static Command runCommandAfterTime(Command command, double time) {
    return Commands.sequence(new WaitCommand(time), command);
  }

  private static Command intakeThenExtake() {
    return Commands.sequence(new RunKrakenForTime(m_armRoller, true, 0.2), new WaitCommand(0.1),
        new RunKrakenForTime(m_armRoller, false, 0.5));
  }

  private static Command extakeInProcessor() {
    return new RunKraken(m_armRoller, 0.5);
  }

  public static Command GTFO() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.TOP:
        return followPath("1gtfo", true);
      case AutonomousStartingPositions.MIDDLE:
        return followPath("2gtfo", true);
      case AutonomousStartingPositions.BOTTOM:
        return followPath("3gtfo", true);
      default:
        return new PrintCommand("GTFO failed?");
    }
  }

  public static Command goToReef() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.TOP:
        return followPath("1toreef", true);
      case AutonomousStartingPositions.MIDDLE:
        return followPath("2toreef", true);
      case AutonomousStartingPositions.BOTTOM:
        return followPath("3toreef", true);
      default:
        return new PrintCommand("goToReef failed?");
    }
  }

  public static Command grabAlgaeFromReef() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.TOP:
        return Commands.parallel(
            followPath("1toreef", true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(0)), 0.0),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kL2),
                0.5),
            runCommandAfterTime(new RunKrakenForTime(m_armRoller, true, 0.5), 1.8));
      case AutonomousStartingPositions.MIDDLE:
        return Commands.parallel(
            followPath("2toreef", true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(0)), 0.0),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kL1),
                0.5),
            runCommandAfterTime(new RunKrakenForTime(m_armRoller, true, 0.5), 1.8));
      case AutonomousStartingPositions.BOTTOM:
        return Commands.parallel(
            followPath("3toreef", true),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(0)), 0.0),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kL2),
                0.5),
            runCommandAfterTime(new RunKrakenForTime(m_armRoller, true, 0.5), 1.8));
      default:
        return new PrintCommand("grabAlgaeFromReef failed?");
    }
  }

  public static Command scoreAlgaeFromReefIntoBarge() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.TOP:
        return Commands.sequence(grabAlgaeFromReef(), Commands.parallel(
            followPath("topreeftobarge", false),
            Commands.sequence(runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kBottom), 0.2),
                runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kL2), 5)),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(95)), 1.0),
            runCommandAfterTime(intakeThenExtake(), 8)));
      case AutonomousStartingPositions.MIDDLE:
        return Commands.sequence(grabAlgaeFromReef(), Commands.parallel(
            followPath("middlereeftobarge", false),
            Commands.sequence(runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kBottom), 0.2),
                runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kL2), 5)),
            runCommandAfterTime(new MoveArmPivotToPosition(m_armPivot, Degrees.of(95)), 1.0),
            runCommandAfterTime(intakeThenExtake(), 8)));
      default:
        return new PrintCommand("scoreAlgaeFromReefIntoBarge failed?");
    }
  }

  public static Command scoreAlgaeFromReefIntoProcessor() {
    switch (m_positionOption) {
      case AutonomousStartingPositions.MIDDLE:
        return Commands.sequence(grabAlgaeFromReef(), Commands.parallel(
            followPath("middlereeftoprocessor", false),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kBottom), 0.1),
            runCommandAfterTime(extakeInProcessor(), 3.7)));
      case AutonomousStartingPositions.BOTTOM:
        return Commands.sequence(grabAlgaeFromReef(), Commands.parallel(
            followPath("bottomreeftoprocessor", false),
            runCommandAfterTime(new MoveElevatorToPosition(m_elevator, TargetPosition.kBottom), 0.1),
            runCommandAfterTime(extakeInProcessor(), 3.2)));
      default:
        return new PrintCommand("scoreAlgaeFromReefIntoProcessor");
    }
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

    if (operation == AutonomousSelectedOperation.SCORE_ALGAE_REEF_BARGE) {
      return scoreAlgaeFromReefIntoBarge();
    }

    if (operation == AutonomousSelectedOperation.SCORE_ALGAE_REEF_PROCESSOR) {
      return scoreAlgaeFromReefIntoProcessor();
    }

    return new PrintCommand("Doing nothing because no operation?");
  }
}
