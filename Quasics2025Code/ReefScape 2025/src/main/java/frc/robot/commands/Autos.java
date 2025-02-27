// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.AutonomousSelectedOperation;
import frc.robot.subsystems.drivebase.AbstractDrivebase;

public final class Autos {
  static final double DIST_TO_REEF = 2.134;

  public static Command followPath(AutoFactory autoFactory, AbstractDrivebase drivebase, String pathName,
      boolean resetOdometry) {

    return Commands.sequence(
        resetOdometry ? autoFactory.resetOdometry(pathName) : Commands.none(),
        resetOdometry ? autoFactory.resetOdometry(pathName) : Commands.none(),
        autoFactory.trajectoryCmd(pathName));
  }

  public static Command GTFO(AutoFactory autoFactory, AbstractDrivebase drivebase, int position, boolean isBlue) {
    switch (position) {
      case 1:
        return followPath(autoFactory, drivebase, "blue1gtfo", true);
      case 2:
        return followPath(autoFactory, drivebase, "blue2gtfo", true);
      case 3:
        return followPath(autoFactory, drivebase, "blue3gtfo", true);
      default:
        return new PrintCommand("GTFO failed?");
    }
  }

  /** Example static factory for an autonomous command. */
  public static Command getAutonomousCommand(AutoFactory autoFactory,
      AbstractDrivebase drivebase, String operation, int position, boolean isBlue) {

    if (operation == AutonomousSelectedOperation.DO_NOTHING) {
      return new PrintCommand("Doing nothing!");
    }
    if (operation == AutonomousSelectedOperation.GTFO) {
      return GTFO(autoFactory, drivebase, position, isBlue);
      /*
       * return Commands.sequence(
       * new DriveForDistance(drivebase, 0.20, Meters.of(DIST_TO_REEF)),
       * new TurnForDegrees(drivebase, 64, -0.15),
       * new DriveForDistance(drivebase, 0.20, Meters.of(1.4)),
       * new TurnForDegrees(drivebase, -64, -0.15));
       */

    }

    if (operation == AutonomousSelectedOperation.GO_TO_REEF) {
      // TODO
    }

    if (operation == AutonomousSelectedOperation.SCORE_ALGAE) {
      // TODO
    }

    return new PrintCommand("Doing nothing because no operation?");
  }
}
