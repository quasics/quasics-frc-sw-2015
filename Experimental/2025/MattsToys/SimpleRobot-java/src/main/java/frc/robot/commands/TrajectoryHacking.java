// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IDrivebasePlus;
import frc.robot.utils.RobotConfigs;
import frc.robot.utils.TrajectoryCommandGenerator;
import java.util.ArrayList;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

/**
 * Starting to play around with dynamic trajectory generation, using
 * Team 3044's Oxplorer.
 *
 * This is still not working right: we're consistently overshooting while driving under Ramsete
 * control, which suggests that the generation is somehow not using the correct characterization of
 * the robot, PID is screwy (e.g., kP too high), bad odometry, Ramsete params wrong (seems
 * unlikely).
 */
public class TrajectoryHacking extends Command {
  /// Values controlling for a RAMSETE follower in units of meters and
  /// seconds.
  ///
  /// If you don't understand these, don't worry too much about it. Just
  /// treat the defaults as "magic constants underpinning the universe",
  /// and you'll be fine. (Or you can dive into the docs, but the water
  /// gets pretty deep awfully fast.)
  private class RamseteConfig {
    // Tuning parameter (b > 0) for which larger values make
    // convergence more aggressive like a proportional term.
    double kRamseteB = 2;
    // Tuning parameter (0 < zeta < 1) for which larger values provide
    // more damping in response.
    double kRamseteZeta = 0.7;
  };

  /**
   * The maximum voltage available to the motors while following the path. Should
   * be somewhat less than the nominal battery voltage (12V) to account for
   * "voltage sag" due to current draw.
   */
  public static final double MAX_VOLTAGE = 10;

  final IDrivebasePlus m_drivebase;
  final RobotConfigs.RobotConfig m_robotConfig;
  final RamseteConfig m_ramseteConfig = new RamseteConfig();

  public TrajectoryHacking(IDrivebasePlus drivebase, RobotConfigs.RobotConfig robotConfig) {
    m_drivebase = drivebase;
    m_robotConfig = robotConfig;
    addRequirements(m_drivebase.asSubsystem());
  }

  private void dumpTrajectory(Trajectory trajectory) {
    for (var state : trajectory.getStates()) {
      System.out.println("-> " + state);
    }
  }

  private Command buildCommandForTrajectory(Trajectory trajectory) {
    SimpleMotorFeedforward feedforward =
        TrajectoryCommandGenerator.getMotorFeedforward(m_robotConfig);
    PIDController leftController = new PIDController(0, 0, 0);
    PIDController rightController = new PIDController(0, 0, 0);

    RamseteCommand ramseteCommand = new RamseteCommand(
        // Trajectory to be followed
        trajectory,
        // Pose supplier
        ()
            -> {
          Pose2d p = m_drivebase.getPose();
          System.out.println("Currently at: " + p);
          return p;
        },
        // Ramsete controller
        new RamseteController(m_ramseteConfig.kRamseteB, m_ramseteConfig.kRamseteZeta),
        // SimpleMotorFeedforward
        feedforward,
        // Drive base kinematics
        m_drivebase.getKinematics(),
        // Wheel speed supplier
        m_drivebase::getWheelSpeeds,
        // Left/right PID controller
        leftController, rightController,
        // RamseteCommand passes volts (as doubles) to the callback
        (leftVolts, rightVolts)
            -> { m_drivebase.setMotorVoltages(Volts.of(leftVolts), Volts.of(rightVolts)); },
        // Required subsystems
        m_drivebase.asSubsystem());
    return ramseteCommand;
  }

  private void runTrajectory(Trajectory trajectory) {
    var states = trajectory.getStates();
    System.out.println("Driving:\n   from " + trajectory.getInitialPose() + " (" + states.get(0)
        + ")\n   to " + states.get(states.size() - 1));

    dumpTrajectory(trajectory);

    Command ramseteCommand = buildCommandForTrajectory(trajectory);

    var actualCommand = ramseteCommand
                            // Stop the motors
                            .andThen(() -> { m_drivebase.stop(); }, m_drivebase.asSubsystem())
                            // Display where we wound up
                            .andThen(() -> {
                              System.out.println("Final position: " + m_drivebase.getPose());
                            }, m_drivebase.asSubsystem());
    actualCommand.schedule();
  }

  private void setupAlternateTrajectory() {
    Pose2d currentPose = m_drivebase.getPose();
    Transform2d transform = new Transform2d(2, 0, currentPose.getRotation().unaryMinus());
    Pose2d destinationPose = currentPose.plus(transform);
    System.out.println("Trying to route:\n   from " + currentPose + "\n   to " + destinationPose);

    TrajectoryConfig trajectoryConfig =
        new TrajectoryConfig(IDrivebase.MAX_SPEED.in(MetersPerSecond), 0.5);
    final DifferentialDriveKinematics kDriveKinematics = m_drivebase.getKinematics();

    final SimpleMotorFeedforward feedforward =
        TrajectoryCommandGenerator.getMotorFeedforward(m_robotConfig);
    // TrajectoryCommandGenerator.getMotorFeedforward(m_robotConfig);
    final DifferentialDriveVoltageConstraint voltageConstraints =
        new DifferentialDriveVoltageConstraint(feedforward, kDriveKinematics, MAX_VOLTAGE);
    TrajectoryConfig actualTrajectoryConfig = new TrajectoryConfig(
        trajectoryConfig.getMaxVelocity(), trajectoryConfig.getMaxAcceleration());
    actualTrajectoryConfig.setKinematics(kDriveKinematics);
    actualTrajectoryConfig.addConstraint(voltageConstraints);

    var trajectory = TrajectoryGenerator.generateTrajectory(
        currentPose, new ArrayList<Translation2d>(), destinationPose, actualTrajectoryConfig);

    runTrajectory(trajectory);
  }

  private void setupOexplorePath() {
    Pathfinder pathfinder = new PathfinderBuilder(Field.REEFSCAPE_2025).build();

    try {
      Pose2d currentPose = m_drivebase.getPose();
      Transform2d transform = new Transform2d(2, 0, currentPose.getRotation().unaryMinus());
      Pose2d destinationPose = currentPose.plus(transform);
      System.out.println("Trying to route:\n   from " + currentPose + "\n   to " + destinationPose);

      final double maxLinear = 1;
      final double maxRotational = 1;
      TrajectoryConfig config = new TrajectoryConfig(maxLinear, maxRotational);
      Trajectory myPath = pathfinder.generateTrajectory(currentPose, destinationPose, config);

      runTrajectory(myPath);
    } catch (ImpossiblePathException e) {
      e.printStackTrace();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // setupOexplorePath();
    setupAlternateTrajectory();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
