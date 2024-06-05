// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;



import frc.robot.Constants.AutonomousSelectedOperation;
import frc.robot.Constants.AutonomousStartingPositions;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;

import frc.robot.commands.Autos;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.MoveClimbers;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTransitionRoller;
import frc.robot.commands.TimedRunShooter;
import frc.robot.commands.TimedRunIntake;
import frc.robot.commands.TimedRunTransitionRoller;


import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.TransitionRoller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import static edu.wpi.first.units.Units.Seconds;
import frc.robot.Trajectorygenerator;
import frc.robot.subsystems.TransitionRoller;

import frc.robot.commands.Autos;
import frc.robot.commands.TankDrive;
import frc.robot.Constants.PathWeaverConstantsMargert;
import frc.robot.Constants.PathWeaverConstantsSally;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.MoveClimbers;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTransitionRoller;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.locks.Condition;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.TransitionRoller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Trajectorygenerator.*;

import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;




public final class Autos {
  public static Command resetOdometryToStartingPosition(Drivebase drivebase, String position, String color) {
    Pose2d startingPose;

    if (position == AutonomousStartingPositions.inFrontOfAmp)
      startingPose = GetTrajectoryInitialPose(color + "1ago");
    else if (position == AutonomousStartingPositions.leftOfSpeaker)
      startingPose = GetTrajectoryInitialPose(color + "1bgo");
    else if (position == AutonomousStartingPositions.inFrontOfSpeaker)
      startingPose = GetTrajectoryInitialPose(color + "2go");
    else if (position == AutonomousStartingPositions.rightOfSpeaker)
      startingPose = GetTrajectoryInitialPose(color + "3ago");
    else if (position == AutonomousStartingPositions.farField)
      startingPose = GetTrajectoryInitialPose(color + "3bgo");
    else {
      // ???
      startingPose = new Pose2d();
    }

    return new SetRobotOdometry(drivebase, startingPose);
  }

  public static Command intakeHelperCommand(IntakeRoller intakeRoller, TransitionRoller transitionRoller) {
    return Commands.parallel(
      new RunTransitionRoller(transitionRoller, 0.5, true),
      new RunIntake(intakeRoller, 0.5, true));
  }

  public static Command shootingSequence(TransitionRoller transitionRoller, Shooter shooter){
    return Commands.parallel(
      transitionDelay(transitionRoller),
      new TimedRunShooter(shooter, 0.5, Seconds.of(2.0), true));
  }

  public static Command transitionDelay(TransitionRoller transitionRoller){
    return Commands.sequence(
      new WaitCommand(0.75),
      new TimedRunTransitionRoller(  transitionRoller, .5, Seconds.of(1.25),true));
  }

  public static Command shootingSequenceWithoutWait(TransitionRoller transitionRoller, Shooter shooter) {
    return Commands.parallel(
      new TimedRunTransitionRoller(transitionRoller, .5, Seconds.of(1.25), true),
      new TimedRunShooter(shooter, 0.5, Seconds.of(1.25), true));
  }

  public static Command intakeWhileDriving(Drivebase drivebase, IntakeRoller intakeRoller, TransitionRoller transitionRoller, String pathName) {
    return Commands.race(
      GetCommandForTrajectory(pathName, drivebase),
      intakeHelperCommand(intakeRoller, transitionRoller)
    );
  }

  public static Command runShooterWhileDriving(Drivebase drivebase, Shooter shooter, String pathName) {
    return Commands.race(
      GetCommandForTrajectory(pathName, drivebase),
      new RunShooter(shooter, 0.5, true)
    );
  }

  /** Example static factory for an autonomous command. */
  public static Command GTFO(Drivebase drivebase, String position, String color) {
    //return new TimedMovementTest(drivebase, Seconds.of(2), 0.50);
    String path = "";
    if (position == AutonomousStartingPositions.inFrontOfAmp)
      path = color + "1ago";
    else if (position == AutonomousStartingPositions.leftOfSpeaker)
      path = color + "1ago";
    else if (position == AutonomousStartingPositions.inFrontOfSpeaker)
      path = color + "1ago";
    else if (position == AutonomousStartingPositions.rightOfSpeaker)
      path = color + "1ago";
    else if (position == AutonomousStartingPositions.farField)
      path = color + "1ago";

    return GetCommandForTrajectory(path, drivebase);
  }

  public static Command score1(TransitionRoller transitionRoller, Shooter shooter, String color) {
    // nothing for amp right now
    return shootingSequence(transitionRoller, shooter);
  }

  
  public static Command getAutonomousCommand(Drivebase drivebase, IntakeRoller intakeRoller, TransitionRoller transitionRoller, Shooter shooter, String overallOperation, String positionOption, String score2Option, String score3Option, boolean isBlue) {
    //ArrayList<Command> commands = new ArrayList<Command>();
    String color;
    if (isBlue) {
      color = "blue";
    }
    else {
      color = "red";
    }

    if (overallOperation == AutonomousSelectedOperation.doNothing){
      return new PrintCommand("Doing Nothing");
    } else if (overallOperation == AutonomousSelectedOperation.GTFO){
      return GTFO(drivebase, positionOption, color);
    } else if (overallOperation == AutonomousSelectedOperation.score1) {
      return score1(transitionRoller, shooter, color);
    }
    //return Commands.sequence(commands);
    return new PrintCommand("???");
  }
}
