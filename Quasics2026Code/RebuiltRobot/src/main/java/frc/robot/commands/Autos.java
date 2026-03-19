// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.testing.DriveForDistance;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IIndexer;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.subsystems.interfaces.IShooterHood;
import frc.robot.utils.PathPlannerHelper;
import java.util.Optional;

/**
 * Helper class, which will build commands/command sequences for use in
 * autonomous mode.
 */
public final class Autos {
  private final PathPlannerHelper m_autoHelper;
  IIndexer m_indexer;
  IIntake m_intake;
  IShooter m_shooter;
  IShooterHood m_hood;
  IDrivebase m_drivebase;
  private SendableChooser<Command> m_sequenceChooser = new SendableChooser<Command>();

  /**
   * Generates a simple command sequence that could be used from either
   * alliance, anywhere on the starting line.
   *
   * This sequence will:
   * <ul>
   * <li>Reset the robot's "known starting point" to (hopefully) match where
   * the drive team put it
   * <li>Drive 4 feet forward
   * <li>Turn and align with
   * the aliance's hub
   * <li>Shoot for 6 seconds
   * </ul>
   */
  public Command generateSampleStartingCommand(IDrivebase drivebase,
      IShooter shooter, IShooterHood hood, Angle shooterAngle,
      Pose2d fieldPose) {
    return new UpdateStartingPositionData(drivebase, fieldPose)
        .andThen(new PrintCommand("Moving"))
        .andThen(new DriveForDistance(drivebase, 0.25, Feet.of(4)))
        .andThen(new PrintCommand("Aligning"))
        .andThen(new AlignToHub(drivebase))
        .andThen(new PivotHoodToPosition(hood, 0, Degrees.of(15)))
        .andThen(new PrintCommand("Shooting"))
        .andThen(new ShootBasedOnDistanceAndTime(
            shooter, drivebase, 0.387, 2, Seconds.of(6)))
        .andThen(new PrintCommand("Done"));
  }

  public Command indexAndShoot(IDrivebase drivebase, IShooter shooter, IIndexer indexer) {
    return new ShootBasedOnDistanceAndTime(shooter, drivebase, .387, 2, Seconds.of(6))
        .alongWith(waitBeforeIndexing(indexer));
  }

  public Command waitBeforeIndexing(IIndexer indexer) {
    return new WaitCommand(2.5).andThen(new RunIndexerForTime(indexer, 0.1, true, 6));
  }

  public Command hubAuto(IDrivebase drivebase, IShooter shooter,
      IShooterHood hood, double hoodAngle, Pose2d fieldPose, IIndexer indexer) {
    return new UpdateStartingPositionData(drivebase, fieldPose)
        .andThen(new PivotHoodToPosition(hood, 0.15, Degrees.of(hoodAngle)))
        .andThen(indexAndShoot(drivebase, shooter, indexer));
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
      // return new SequentialCommandGroup(
      // AutoBuilder.buildAuto("BackOutAndShoot1"),
      // new RunShooterForTime(m_shooter, 5, 2, true, 5),
      // AutoBuilder.buildAuto("GrabbingBalls_"),
      // new RunIntakeExtension(m_intake, 2, true),
      // new RunIntakeRollers(m_intake, 5, true),
      // AutoBuilder.buildAuto("JustBallGrabber_"));
      default:
        return null;
    }
  }

  public void configureSequenceSelector() {
    Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
    if (optionalAlliance.isPresent()) {
      Alliance alliance = optionalAlliance.get();
      System.out.println("Alliance is " + alliance);
      Pose2d leftTrenchPose = switch (alliance) {
        case Red ->
          new Pose2d(
              new Translation2d(12.57, 0.634), new Rotation2d(Degrees.of(0)));
        case Blue ->
          new Pose2d(
              new Translation2d(3.971, 7.436), new Rotation2d(Degrees.of(180)));
      };
      m_sequenceChooser.addOption("Left Trench",
          generateSampleStartingCommand(
              m_drivebase, m_shooter, m_hood, Degrees.of(15), leftTrenchPose));

      Pose2d leftBumpPose = switch (alliance) {
        case Red ->
          new Pose2d(
              new Translation2d(12.989, 2.011), new Rotation2d(Degrees.of(0)));
        case Blue ->
          new Pose2d(
              new Translation2d(3.552, 6.059), new Rotation2d(Degrees.of(180)));
      };
      m_sequenceChooser.addOption("Left Bump",
          generateSampleStartingCommand(
              m_drivebase, m_shooter, m_hood, Degrees.of(15), leftBumpPose));

      Pose2d hubPose = switch (alliance) {
        case Red ->
          new Pose2d(new Translation2d(12.989, 4.035),
              new Rotation2d(Degrees.of(180)));
        case Blue ->
          new Pose2d(
              new Translation2d(3.552, 4.035), new Rotation2d(Degrees.of(0)));
      };
      m_sequenceChooser.addOption(
          "Hub", hubAuto(m_drivebase, m_shooter, m_hood, 15, hubPose, m_indexer));

      Pose2d rightBumpPose = switch (alliance) {
        case Red ->
          new Pose2d(
              new Translation2d(12.989, 6.059), new Rotation2d(Degrees.of(0)));
        case Blue ->
          new Pose2d(
              new Translation2d(3.552, 2.011), new Rotation2d(Degrees.of(180)));
      };
      m_sequenceChooser.addOption("Right Bump",
          generateSampleStartingCommand(
              m_drivebase, m_shooter, m_hood, Degrees.of(15), rightBumpPose));

      Pose2d rightTrenchPose = switch (alliance) {
        case Red ->
          new Pose2d(
              new Translation2d(12.57, 7.436), new Rotation2d(Degrees.of(0)));
        case Blue ->
          new Pose2d(
              new Translation2d(3.971, 0.634), new Rotation2d(Degrees.of(180)));
      };
      m_sequenceChooser.addOption("Right Trench",
          generateSampleStartingCommand(
              m_drivebase, m_shooter, m_hood, Degrees.of(15), rightTrenchPose));
    }
  }

  public Autos(IDrivebase drivebase, IShooter shooter, IShooterHood hood, IIndexer indexer) {
    m_autoHelper = new PathPlannerHelper(drivebase);
    m_drivebase = drivebase;
    m_shooter = shooter;
    m_hood = hood;
    m_indexer = indexer;
    configureSequenceSelector();
    SmartDashboard.putData("Sequence Chooser", m_sequenceChooser);
  }

  public Command getSequenceAuto() {
    Command autoCommand = null;
    autoCommand = m_sequenceChooser.getSelected();
    return autoCommand;
  }
}
