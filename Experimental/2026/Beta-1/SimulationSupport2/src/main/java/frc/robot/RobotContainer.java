// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.TankDrive;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.interfaces.IDrivebasePlus;
import frc.robot.subsystems.interfaces.IElevator;
import frc.robot.subsystems.interfaces.IElevator.ElevatorPosition;
import frc.robot.subsystems.interfaces.ISingleJointArm;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.subsystems.simulated.CameraSimulator;
import frc.robot.subsystems.simulated.SimDrivebase;
import frc.robot.util.DriverJoystickWrapper;
import frc.robot.util.RobotConfigLibrary;
import frc.robot.util.SysIdGenerator;
import frc.robot.util.SysIdGenerator.DrivebaseProfilingMode;
import java.util.List;

/**
 * Core definitions for sample robot code (focused on demonstrating simulation support).
 *
 * This is where the robot's subsystems, operator interfaces (joysticks, gamepads, etc.), and the
 * high-level commands that link them together are defined and configured. It serves as the central
 * hub for the declarative setup of a command-based robot project using the WPILib software library
 */
public class RobotContainer {
  /** Whether to use arcade drive or tank drive for robot navigation. */
  private static final boolean USE_ARCADE_DRIVE = true;

  /** The drivebase subsystem. */
  final IDrivebasePlus m_drivebase = Robot.isReal() ? new Drivebase() : new SimDrivebase();

  /** The elevator subsystem.  (At present, always simulated.) */
  final IElevator m_elevator = new frc.robot.subsystems.simulated.SimElevator();

  /** The arm subsystem.  (At present, always simulated.) */
  final ISingleJointArm m_arm = new frc.robot.subsystems.simulated.SimArm();

  final frc.robot.util.RobotConfigs.RobotConfig RobotConfig =
      RobotConfigLibrary.getConfig(RobotConfigLibrary.Robot.Simulation);

  final IVision m_vision = new frc.robot.subsystems.PhotonVision(
      RobotConfigLibrary.getConfig(RobotConfigLibrary.Robot.Simulation).cameras().get(0));

  /** The driver joystick wrapper. */
  final DriverJoystickWrapper m_driverWrapper =
      new DriverJoystickWrapper(OperatorConstants.DRIVER_JOYSTICK_ID,
          // Only load from/save to preferences when in simulation
          Robot.isSimulation());

  /** The autonomous command chooser. */
  private final SendableChooser<Command> m_autoCommandChooser = new SendableChooser<Command>();

  /** Constructor. */
  public RobotContainer() {
    configureDriving();
    setupAutonomousChooser();
    configureSysIdCommands();
    configureElevatorCommands();
    configureArmCommands();
    configureBindings();

    if (Robot.isSimulation()) {
      new CameraSimulator(RobotConfig, (PhotonVision) m_vision);
    }
  }

  /**
   * Adds various sample commands for controlling the single-joint arm to the dashboard.
   */
  private void configureArmCommands() {
    // Note that the delays will not be enough to get the arm *absolutely* into position,
    // but that's OK: I'm just trying to make it wave back and forth....
    Command waveCommand = new SequentialCommandGroup(
        new InstantCommand(
            () -> { m_arm.setTargetPosition(m_arm.getArmOutAngle()); }, m_arm.asSubsystem()),
        // Wait for some motion
        new WaitCommand(2),
        new InstantCommand(
            () -> { m_arm.setTargetPosition(m_arm.getArmUpAngle()); }, m_arm.asSubsystem()),
        // Wait for some motion
        new WaitCommand(2))
                              .repeatedly();
    SmartDashboard.putData("Cmd: Arm out", new InstantCommand(() -> {
      m_arm.setTargetPosition(m_arm.getArmOutAngle());
    }, m_arm.asSubsystem()));
    SmartDashboard.putData("Cmd: Arm up", new InstantCommand(() -> {
      m_arm.setTargetPosition(m_arm.getArmUpAngle());
    }, m_arm.asSubsystem()));
    SmartDashboard.putData("Cmd: Arm wave", waveCommand);
    SmartDashboard.putData(
        "Cmd: Arm stop", new InstantCommand(() -> { m_arm.stop(); }, m_arm.asSubsystem()));
  }

  /**
   * Adds various sample commands for controlling the elevator to the dashboard.
   */
  private void configureElevatorCommands() {
    SmartDashboard.putData("Cmd: Elevator up", new InstantCommand(() -> {
      m_elevator.setTargetPosition(ElevatorPosition.HIGH);
    }, m_elevator.asSubsystem()));
    SmartDashboard.putData("Cmd: Elevator down", new InstantCommand(() -> {
      m_elevator.setTargetPosition(ElevatorPosition.BOTTOM);
    }, m_elevator.asSubsystem()));
    SmartDashboard.putData("Cmd: Elevator stop",
        new InstantCommand(() -> { m_elevator.stop(); }, m_elevator.asSubsystem()));
  }

  /** Configures the driving behavior. */
  private void configureDriving() {
    m_driverWrapper.setDeadbandThreshold(OperatorConstants.DEADBAND_THRESHOLD);
    SlewRateLimiter limiter1 = new SlewRateLimiter(OperatorConstants.MAX_SLEW_RATE);
    SlewRateLimiter limiter2 = new SlewRateLimiter(OperatorConstants.MAX_SLEW_RATE);
    if (USE_ARCADE_DRIVE) {
      m_drivebase.asSubsystem().setDefaultCommand(new ArcadeDrive(m_drivebase,
          ()
              -> limiter1.calculate(m_driverWrapper.getArcadeForward()),
          () -> limiter2.calculate(m_driverWrapper.getArcadeRotation())));
    } else {
      m_drivebase.asSubsystem().setDefaultCommand(new TankDrive(m_drivebase,
          ()
              -> limiter1.calculate(m_driverWrapper.getTankLeft()),
          () -> limiter2.calculate(m_driverWrapper.getTankRight())));
    }
  }

  /** Adds commands for profiling the drive base to the dashboard. */
  private void configureSysIdCommands() {
    // Dynamic and quasistatic commands for linear drivebase profiling
    SmartDashboard.putData("Cmd: DynamicFwd",
        SysIdGenerator.sysIdDynamic(
            m_drivebase, DrivebaseProfilingMode.Linear, Direction.kForward));
    SmartDashboard.putData("Cmd: DynamicRev",
        SysIdGenerator.sysIdDynamic(
            m_drivebase, DrivebaseProfilingMode.Linear, Direction.kReverse));
    SmartDashboard.putData("Cmd: QStaticFwd",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, DrivebaseProfilingMode.Linear, Direction.kForward));
    SmartDashboard.putData("Cmd: QStaticRev",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, DrivebaseProfilingMode.Linear, Direction.kReverse));

    // Dynamic and quasistatic commands for angular drivebase profiling
    SmartDashboard.putData("Cmd: DynamicFwd - Angular",
        SysIdGenerator.sysIdDynamic(
            m_drivebase, DrivebaseProfilingMode.Angular, Direction.kForward));
    SmartDashboard.putData("Cmd: DynamicRev - Angular",
        SysIdGenerator.sysIdDynamic(
            m_drivebase, DrivebaseProfilingMode.Angular, Direction.kReverse));
    SmartDashboard.putData("Cmd: QStaticFwd - Angular",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, DrivebaseProfilingMode.Angular, Direction.kForward));
    SmartDashboard.putData("Cmd: QStaticRev - Angular",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, DrivebaseProfilingMode.Angular, Direction.kReverse));
  }

  /** Configures the autonomous command chooser and adds it to the dashboard. */
  private void setupAutonomousChooser() {
    m_autoCommandChooser.setDefaultOption(
        "No Auto", Commands.print("No autonomous command configured"));
    m_autoCommandChooser.addOption("Do something", Commands.print("Do something"));
    m_autoCommandChooser.addOption("Trajectory (Linear)",
        new FollowTrajectoryCommand(m_drivebase, generateTrajectory(TrajectoryShape.Linear)));
    m_autoCommandChooser.addOption("Trajectory (Curved)",
        new FollowTrajectoryCommand(m_drivebase, generateTrajectory(TrajectoryShape.SimpleCurve)));
    m_autoCommandChooser.addOption("Trajectory (Circle)",
        new FollowTrajectoryCommand(m_drivebase, generateTrajectory(TrajectoryShape.Circle)));
    m_autoCommandChooser.addOption("Trajectory (S-curve)",
        new FollowTrajectoryCommand(m_drivebase, generateTrajectory(TrajectoryShape.SCurve)));
    SmartDashboard.putData("Autonomous Command", m_autoCommandChooser);
  }

  /** Configures any additional bindings that are needed. */
  private void configureBindings() {
    // Add any additional bindings here.
  }

  /** Returns the command to run in autonomous mode. */
  public Command getAutonomousCommand() {
    if (m_autoCommandChooser.getSelected() != null) {
      return m_autoCommandChooser.getSelected();
    }
    return Commands.print("No selection found for autonomous command");
  }

  //
  // Trajectory-following stuff
  //

  /** Defines shapes supported for trajectory-following example commands. */
  enum TrajectoryShape { Linear, SimpleCurve, SCurve, Circle }

  /** Maximum desired voltage draw when performing trajectory-following. */
  private static final double kMaxVoltageForSampleTrajectories = 10;

  /** Maximum desired acceleration when performing trajectory-following. */
  private static final LinearAcceleration maxAccelerationForSampleTrajectories =
      MetersPerSecondPerSecond.of(3);

  /** Configuration for use in generating sample trajectories. */
  private final TrajectoryConfig m_trajectoryConfig =
      // Base configuration (max speed/accelleration)
      new TrajectoryConfig(m_drivebase.getMaxLinearSpeed(), maxAccelerationForSampleTrajectories)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(m_drivebase.getKinematics())
          // Apply a voltage constraint to ensure we don't accelerate too fast (and brown us out)
          .addConstraint(
              new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(m_drivebase.getKs(),
                                                         m_drivebase.getKv(), m_drivebase.getKa()),
                  m_drivebase.getKinematics(), kMaxVoltageForSampleTrajectories));

  /**
   * Generates a robot-relative trajectory for a given shape.
   *
   * @param shape the desired shape of the trajectory
   */
  Trajectory generateTrajectory(TrajectoryShape shape) {
    return switch (shape) {
      case Linear ->
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No interior waypoints - just a straight line
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            m_trajectoryConfig);

      case SimpleCurve ->
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No interior waypoints - just a straight line
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 3, new Rotation2d(Degrees.of(90))),
            // Pass config
            m_trajectoryConfig);

      case SCurve ->
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(2, 1), new Translation2d(4, -1)),
            // End 6 meters straight ahead of where we started, facing forward
            new Pose2d(6, 0, new Rotation2d(0)),
            // Pass config
            m_trajectoryConfig);

      case Circle ->
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these three interior waypoints along the perimeter of a circle
            List.of(new Translation2d(2, 2), new Translation2d(0, 4), new Translation2d(-2, 2)),
            // End back where we started, facing forward
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass config
            m_trajectoryConfig);
    };
  }
}
