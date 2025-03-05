// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonomousSelectedOperation;
import frc.robot.subsystems.armPivot.ArmPivot;
import frc.robot.subsystems.drivebase.AbstractDrivebase;
import frc.robot.subsystems.elevator.AbstractElevator;

public final class Autos {

  /*
   * IMPORTANT POSITIONS FOR CHOREO
   * Robot size: 0.851 m by 0.993 m
   * Blue 1:
   * Blue 2:
   * Blue 3:
   * Top of reef:
   * Middle of reef:
   * Bottom of reef:
   * Barge:
   * Processor:
   */

  static final double DIST_TO_REEF = 2.134;
  private static AutoFactory m_autoFactory;
  private static AbstractDrivebase m_drivebase;
  private static ArmPivot m_armPivot;
  private static AbstractElevator m_elevator;
  private static int m_position;

  public static Command followPath(String pathName,
      boolean resetOdometry) {

    return Commands.sequence(
        resetOdometry ? m_autoFactory.resetOdometry(pathName) : Commands.none(),
        resetOdometry ? m_autoFactory.resetOdometry(pathName) : Commands.none(),
        m_autoFactory.trajectoryCmd(pathName));
  }

  public static Command GTFO() {
    switch (m_position) {
      case 1:
        return followPath("1gtfo", true);
      case 2:
        return followPath("2gtfo", true);
      case 3:
        return followPath("3gtfo", true);
      default:
        return new PrintCommand("GTFO failed?");
    }
  }

  public static Command goToReef() {
    switch (m_position) {
      case 1:
        return followPath("1toreef", true);
      case 2:
        return followPath("2toreef", true);
      case 3:
        return followPath("3toreef", true);
      default:
        return new PrintCommand("goToReef failed?");
    }
  }

  public static Command grabAlgaeFromReef() {
    return new PrintCommand("TODO");
  }

  public static Command scoreAlgaeFromReefIntoBarge() {
    return new PrintCommand("TODO");
  }

  public static Command scoreAlgaeFromReefIntoProcessor() {
    switch (m_position) {
      case 1:
        return Commands.parallel(followPath("middlereeftoprocessor", true),
            runArmPivotAndElevatorDownAfterTime(m_elevator, m_armPivot, 1.0));
      case 2:
        return Commands.parallel(followPath("bottomreeftoprocessor", true),
            runArmPivotAndElevatorDownAfterTime(m_elevator, m_armPivot, 1.0));
      default:
        return new PrintCommand("scoreAlgaeFromReefIntoProcessor");
    }
  }

  public static Command runArmPivotAndElevatorDownAfterTime(AbstractElevator m_elevator,
      ArmPivot m_armPivot, double seconds) {
    return Commands.sequence(new WaitCommand(seconds), new MoveArmPivotAndElevatorToPosition(m_armPivot, m_elevator,
        Radians.of(0), AbstractElevator.TargetPosition.kBottom));
  }

  /** Example static factory for an autonomous command. */
  public static Command getAutonomousCommand(AutoFactory autoFactory,
      AbstractDrivebase drivebase, String operation, int position) {
    m_autoFactory = autoFactory;
    m_drivebase = drivebase;
    m_position = position;

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
