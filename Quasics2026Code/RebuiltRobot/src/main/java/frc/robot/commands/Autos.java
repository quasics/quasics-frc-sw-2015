// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.testing.DriveForDistance;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.utils.PathPlannerHelper;

public final class Autos {

  public static Command moveForward(IDrivebase drivebase) {
    return PathPlannerHelper.getAutonomousCommand(drivebase, "MoveForward1");
  }

  /**
   * Generates a simple command sequence that could be used from either alliance,
   * anywhere on the starting line.
   * 
   * This sequence will:
   * <ul>
   * <li>Reset the robot's "known starting point" to (hopefully) match where the
   * drive team put it
   * <li>Drive 4 feet forward
   * <li>Turn and align with the aliance's hub
   * <li>Shoot for 6 seconds
   * </ul>
   */
  public static Command generateSampleStartingCommand(IDrivebase drivebase, IShooter shooter, Pose2d fieldPose) {
    return new UpdateStartingPositionData(drivebase, fieldPose)
        .andThen(new PrintCommand("Moving"))
        .andThen(new DriveForDistance(drivebase, 0.25, Feet.of(4)))
        .andThen(new PrintCommand("Aligning"))
        .andThen(new AlignToHub(drivebase))
        .andThen(new PrintCommand("Shooting"))
        .andThen(new ShootBasedOnDistanceAndTime(
            shooter, drivebase, 0.387, 2,
            Seconds.of(6)))
        .andThen(new PrintCommand("Done"));
  }

  public static Command doNothingAtHub(IDrivebase drivebase) {
    drivebase.resetOdometry(new Pose2d(new Translation2d(3.879, 3.942), new Rotation2d(0)));
    return Commands.print("Just sit there");
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
