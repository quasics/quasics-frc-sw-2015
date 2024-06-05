package frc.robot;

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

public class Trajectorygenerator {
    public static Command GetCommandForTrajectory(String fileToLoad, Drivebase drivebase) {
        final Measure<Distance> TRACK_WIDTH_METERS = Meters.of(0.5588);
        final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    
        final Measure<Velocity<Distance>> kMaxSpeed = MetersPerSecond.of(3.0);
        final Measure<Velocity<Velocity<Distance>>> kMaxAcceleration = MetersPerSecondPerSecond.of(3.0);

        final double kRamseteB = 2;
        final double kRamseteZeta = 0.7;

        DifferentialDriveVoltageConstraint autoVoltageConstraint;
        
        // Create a voltage constraint to ensure we don't accelerate too fast
        if (ConditionalConstants.SALLY) {
            autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                        PathWeaverConstantsSally.kS,
                        PathWeaverConstantsSally.kV,
                        PathWeaverConstantsSally.kA),
                    kDriveKinematics,
                    10);
        }
        else { // margeaert
            autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                        PathWeaverConstantsMargert.kS,
                        PathWeaverConstantsMargert.kV,
                        PathWeaverConstantsMargert.kA),
                    kDriveKinematics,
                    10);
        }

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    kMaxSpeed,
                    kMaxAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
        /*
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);
        */
        String pathName = "output/" + fileToLoad + ".wpilib.json";
        Trajectory trajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fileToLoad);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathName, ex.getStackTrace());
        }

        RamseteCommand ramseteCommand;

        if (ConditionalConstants.SALLY) {
            ramseteCommand =
                new RamseteCommand(
                    trajectory,
                    drivebase::getPose,
                    new RamseteController(kRamseteB, kRamseteZeta),
                    new SimpleMotorFeedforward(
                        PathWeaverConstantsSally.kS,
                        PathWeaverConstantsSally.kV,
                        PathWeaverConstantsSally.kA),
                    kDriveKinematics,
                    drivebase::getWheelSpeeds,
                    new PIDController(PathWeaverConstantsSally.kP, 0, 0),
                    new PIDController(PathWeaverConstantsSally.kP, 0, 0),
                    // RamseteCommand passes volts to the callback
                    drivebase::setVoltages,
                    drivebase);
        }
        else { // margaret
            ramseteCommand =
                new RamseteCommand(
                    trajectory,
                    drivebase::getPose,
                    new RamseteController(kRamseteB, kRamseteZeta),
                    new SimpleMotorFeedforward(
                        PathWeaverConstantsMargert.kS,
                        PathWeaverConstantsMargert.kV,
                        PathWeaverConstantsMargert.kA),
                    kDriveKinematics,
                    drivebase::getWheelSpeeds,
                    new PIDController(PathWeaverConstantsMargert.kP, 0, 0),
                    new PIDController(PathWeaverConstantsMargert.kP, 0, 0),
                    // RamseteCommand passes volts to the callback
                    drivebase::setVoltages,
                    drivebase);
        }

        // Reset odometry to the initial pose of the trajectory, run path following
        // command, then stop at the end.
        return Commands.runOnce(() -> drivebase.stop())
            .andThen(ramseteCommand)
            // Because Mr. Healy is professionally paranoid...
        .andThen(Commands.runOnce(() -> drivebase.stop()));

    }

    

    public static Pose2d GetTrajectoryInitialPose(String fileToLoad) {
        String pathName = "output/" + fileToLoad + ".wpilib.json";
        Trajectory trajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fileToLoad);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathName, ex.getStackTrace());
        }
        
        return trajectory.getInitialPose();
    }

    public static Pose2d GetTrajectoryFinalPose(String fileToLoad) {
        String pathName = "output/" + fileToLoad + ".wpilib.json";
        Trajectory trajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fileToLoad);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathName, ex.getStackTrace());
        }
        
        List<State> states = trajectory.getStates();
        return states.get(states.size() - 1).poseMeters; // last state
    }
}
