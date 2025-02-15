// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousSelectedOperation;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.drivebase.AbstractDrivebase;

import edu.wpi.first.wpilibj2.command.PrintCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command getAutonomousCommand(AbstractDrivebase drivebase, String operation, String position,
      boolean isBlue) {
    // return new PathPlannerAuto("Basic test");
    if (operation == AutonomousSelectedOperation.doNothing) {
      return new PrintCommand("Doing nothing!");
    }
    if (operation == AutonomousSelectedOperation.GTFO) {
      return new DriveForDistance(drivebase, 0.30, Meters.of(3));
    }

    return new PrintCommand("Doing nothing because no operation?");

  }
}
