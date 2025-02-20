// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousSelectedOperation;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.drivebase.AbstractDrivebase;

import edu.wpi.first.wpilibj2.command.PrintCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command getAutonomousCommand(AbstractDrivebase drivebase, String operation, String position,
      boolean isBlue) {
    // return new PathPlannerAuto("Basic test");
    if (operation == AutonomousSelectedOperation.DO_NOTHING) {
      return new PrintCommand("Doing nothing!");
    }
    if (operation == AutonomousSelectedOperation.GTFO) {
      return new DriveForDistance(drivebase, 0.20, Meters.of(2));
    }

    return new PrintCommand("Doing nothing because no operation?");

  }
}
