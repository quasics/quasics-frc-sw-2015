// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.AutonomousSelectedOperation;
import frc.robot.subsystems.drivebase.AbstractDrivebase;

public final class Autos {
  static final double DIST_TO_REEF = 2.134;
  /** Example static factory for an autonomous command. */
  public static Command getAutonomousCommand(
      AbstractDrivebase drivebase, String operation, String position, boolean isBlue) {
    // return new PathPlannerAuto("Basic test");
    if (operation == AutonomousSelectedOperation.DO_NOTHING) {
      return new PrintCommand("Doing nothing!");
    }
    if (operation == AutonomousSelectedOperation.GTFO) {
      return new DriveForDistance(drivebase, 0.20, Meters.of(DIST_TO_REEF));
    }

    return new PrintCommand("Doing nothing because no operation?");
  }
}
