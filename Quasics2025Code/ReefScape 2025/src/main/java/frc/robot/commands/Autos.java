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

  public static Command followPath(
      AutoFactory autoFactory, String pathName,
      boolean resetOdometry) {

    return Commands.sequence(
        resetOdometry ? autoFactory.resetOdometry(pathName) : Commands.none(),
        resetOdometry ? autoFactory.resetOdometry(pathName) : Commands.none(),
        autoFactory.trajectoryCmd(pathName));
  }

  public static Command GTFO(AutoFactory autoFactory, int position) {
    switch (position) {
      case 1:
        return followPath(autoFactory, "1gtfo", true);
      case 2:
        return followPath(autoFactory, "2gtfo", true);
      case 3:
        return followPath(autoFactory, "3gtfo", true);
      default:
        return new PrintCommand("GTFO failed?");
    }
  }

  public static Command goToReef(AutoFactory autoFactory, int position) {
    switch (position) {
      case 1:
        return followPath(autoFactory, "1toreef", true);
      case 2:
        return followPath(autoFactory, "2toreef", true);
      case 3:
        return followPath(autoFactory, "3toreef", true);
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

  public static Command scoreAlgaeFromReefIntoProcessor(
      AutoFactory autoFactory,
      AbstractElevator elevator,
      ArmPivot armPivot,
      int position) {
    switch (position) {
      case 1:
        return Commands.parallel(followPath(autoFactory, "middlereeftoprocessor", true),
            runArmPivotAndElevatorDownAfterTime(elevator, armPivot, 1.0));
      case 2:
        return Commands.parallel(followPath(autoFactory, "bottomreeftoprocessor", true),
            runArmPivotAndElevatorDownAfterTime(elevator, armPivot, 1.0));
      default:
        return new PrintCommand("scoreAlgaeFromReefIntoProcessor");
    }
  }

  public static Command runArmPivotAndElevatorDownAfterTime(AbstractElevator elevator,
      ArmPivot armPivot, double seconds) {
    return Commands.sequence(new WaitCommand(seconds), new MoveArmPivotAndElevatorToPosition(armPivot, elevator,
        Radians.of(0), AbstractElevator.TargetPosition.kBottom));
  }

  /** Example static factory for an autonomous command. */
  public static Command getAutonomousCommand(
      AutoFactory autoFactory,
      AbstractDrivebase drivebase,
      AbstractElevator elevator,
      ArmPivot armPivot,
      String operation,
      int position) {

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
      return goToReef(autoFactory, position);
    }

    if (operation == AutonomousSelectedOperation.GTFO) {
      // return new DriveForDistance(drivebase, 0.20, Meters.of(DIST_TO_REEF));
      return GTFO(autoFactory, position);
    }

    if (operation == AutonomousSelectedOperation.GRAB_ALGAE_FROM_REEF) {
      return grabAlgaeFromReef();
    }

    if (operation == AutonomousSelectedOperation.SCORE_ALGAE_REEF_BARGE) {
      return scoreAlgaeFromReefIntoBarge();
    }

    if (operation == AutonomousSelectedOperation.SCORE_ALGAE_REEF_PROCESSOR) {
      return scoreAlgaeFromReefIntoProcessor(autoFactory, elevator, armPivot, position);
    }

    return new PrintCommand("Doing nothing because no operation?");
  }
}
