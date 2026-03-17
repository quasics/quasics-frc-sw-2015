// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.RunIntakeExtension;
import frc.robot.commands.testing.DriveForDistance;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.utils.PathPlannerHelper;

/**
 * Helper class, which will build commands/command sequences for use in
 * autonomous mode.
 */
public final class Autos {
  private final PathPlannerHelper m_autoHelper;

  /**
   * Generates a simple command sequence that could be used from either
   * alliance, anywhere on the starting line.
   *
   * This sequence will:
   * <ul>
   * <li>Reset the robot's "known starting point" to (hopefully) match where
   * the drive team put it <li>Drive 4 feet forward <li>Turn and align with
   * the aliance's hub <li>Shoot for 6 seconds
   * </ul>
   */
  public static Command generateSampleStartingCommand(
      IDrivebase drivebase, IShooter shooter, Pose2d fieldPose) {
    return new UpdateStartingPositionData(drivebase, fieldPose)
        .andThen(new PrintCommand("Moving"))
        .andThen(new DriveForDistance(drivebase, 0.25, Feet.of(4)))
        .andThen(new PrintCommand("Aligning"))
        .andThen(new AlignToHub(drivebase))
        .andThen(new PrintCommand("Shooting"))
        .andThen(new ShootBasedOnDistanceAndTime(
            shooter, drivebase, 0.387, 2, Seconds.of(6)))
        .andThen(new PrintCommand("Done"));
  }

  // TODO: Add a sequential command group.
  public Command getAuto(
      IDrivebase drivebase, IShooter shooter, IClimber climber) {
    switch (m_autoHelper.getAutoName()) {
      // Speed values are placeholders until I know what units they use.
      case "BackOutAndShoot1":
        return new SequentialCommandGroup(m_autoHelper.getAuto(),
            new ShootBasedOnDistanceAndTime(
                shooter, drivebase, 0.387, 2, Seconds.of(5)));

      case "Climb and Shoot":
        return new SequentialCommandGroup(
            new ShootBasedOnDistanceAndTime(
                shooter, drivebase, 0.387, 2, Seconds.of(5)),
            m_autoHelper.getAuto(), new ClimberForPercentage(climber, 1));

      case "ShootAndMoveBack":
        return new SequentialCommandGroup(
            new RunShooterForTime(shooter, 5, 2, true, 5),
            m_autoHelper.getAuto());

      // case "BackOutAndGrab":
      //   return new SequentialCommandGroup(
      //       AutoBuilder.buildAuto("BackOutAndShoot1"),
      //       new RunShooterForTime(m_Shooter, 5, 2, true, 5),
      //       AutoBuilder.buildAuto("GrabbingBalls_"),
      //       new RunIntakeExtension(m_Intake, 2, true),
      //       new RunIntakeRollers(m_Intake, 5, true),
      //       AutoBuilder.buildAuto("JustBallGrabber_"));
      default:
        return null;
    }
  }

  public Autos(IDrivebase drivebase) {
    m_autoHelper = new PathPlannerHelper(drivebase);
  }
}
