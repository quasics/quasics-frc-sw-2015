// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.testing.DriveForDistance;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.subsystems.interfaces.IShooterHood;
import frc.robot.utils.PathPlannerHelper;

/**
 * Helper class, which will build commands/command sequences for use in
 * autonomous mode.
 */
public final class Autos {
  private final PathPlannerHelper m_autoHelper;
  IIntake m_Intake;
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
  public Command generateSampleStartingCommand(
      IDrivebase drivebase, IShooter shooter, IShooterHood hood, Angle shooterAngle, Pose2d fieldPose) {
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

  public Command hubAuto(IDrivebase drivebase, IShooter shooter,
      IShooterHood hood, double hoodAngle, Pose2d fieldPose) {
    return new UpdateStartingPositionData(drivebase, fieldPose)
        .andThen(new PivotHoodToPosition(hood, 0.15, Degrees.of(hoodAngle)))
        .andThen(new ShootBasedOnDistanceAndTime(shooter, drivebase, 0.387, 1, Seconds.of(6)));
  }

  // TODO: Add a sequential command group.
  public Command getAuto() {
    Command autoCommand;
    switch (m_autoHelper.getAutoName()) {
      case "BackOutAndShoot1":
        autoCommand = new RunShooterForTime(m_shooter, 5, 2, true, 5);

      default:
        autoCommand = null;
    }

    return new SequentialCommandGroup(m_autoHelper.getAuto(), autoCommand);
  }

  public void configureSequenceSelector() {
    Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
    if (optionalAlliance.isPresent()) {
      Alliance alliance = optionalAlliance.get();
      System.out.println("Alliance is " + alliance);
      Pose2d leftTrenchPose = switch (alliance) {
        case Red -> new Pose2d(new Translation2d(12.57, 0.634), new Rotation2d(Degrees.of(0)));
        case Blue -> new Pose2d(new Translation2d(3.971, 7.436), new Rotation2d(Degrees.of(180)));
      };
      m_sequenceChooser.addOption("Left Trench",
          generateSampleStartingCommand(m_drivebase, m_shooter, m_hood, Degrees.of(15), leftTrenchPose));

      Pose2d leftBumpPose = switch (alliance) {
        case Red -> new Pose2d(new Translation2d(12.989, 2.011), new Rotation2d(Degrees.of(0)));
        case Blue -> new Pose2d(new Translation2d(3.552, 6.059), new Rotation2d(Degrees.of(180)));
      };
      m_sequenceChooser.addOption("Left Bump", generateSampleStartingCommand(m_drivebase, m_shooter, m_hood,
          Degrees.of(15), leftBumpPose));

      Pose2d hubPose = switch (alliance) {
        case Red -> new Pose2d(new Translation2d(12.989, 4.035), new Rotation2d(Degrees.of(180)));
        case Blue -> new Pose2d(new Translation2d(3.552, 4.035), new Rotation2d(Degrees.of(0)));
      };
      m_sequenceChooser.addOption("Hub", hubAuto(m_drivebase, m_shooter, m_hood, 15, hubPose));

      Pose2d rightBumpPose = switch (alliance) {
        case Red -> new Pose2d(new Translation2d(12.989, 6.059), new Rotation2d(Degrees.of(0)));
        case Blue -> new Pose2d(new Translation2d(3.552, 2.011), new Rotation2d(Degrees.of(180)));
      };
      m_sequenceChooser.addOption("Right Bump", generateSampleStartingCommand(m_drivebase, m_shooter, m_hood,
          Degrees.of(15), rightBumpPose));

      Pose2d rightTrenchPose = switch (alliance) {
        case Red -> new Pose2d(new Translation2d(12.57, 7.436), new Rotation2d(Degrees.of(0)));
        case Blue -> new Pose2d(new Translation2d(3.971, 0.634), new Rotation2d(Degrees.of(180)));
      };
      m_sequenceChooser.addOption("Right Trench", generateSampleStartingCommand(m_drivebase, m_shooter, m_hood,
          Degrees.of(15), rightTrenchPose));
    }
  }

  public Autos(IDrivebase drivebase, IShooter shooter, IShooterHood hood) {
    m_autoHelper = new PathPlannerHelper(drivebase);
    m_drivebase = drivebase;
    m_shooter = shooter;
    m_hood = hood;
    configureSequenceSelector();
    SmartDashboard.putData("Sequence Chooser", m_sequenceChooser);
  }

  public Command getSequenceAuto() {
    Command autoCommand = null;
    autoCommand = m_sequenceChooser.getSelected();
    return autoCommand;
  }
}
