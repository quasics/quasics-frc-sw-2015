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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.TankDrive;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.interfaces.IDrivebasePlus;
import frc.robot.subsystems.interfaces.IElevator;
import frc.robot.subsystems.interfaces.IElevator.ElevatorPosition;
import frc.robot.subsystems.interfaces.ISingleJointArm;
import frc.robot.subsystems.simulated.SimDrivebase;
import frc.robot.util.DriverJoystickWrapper;
import frc.robot.util.SysIdGenerator;
import frc.robot.util.SysIdGenerator.DrivebaseProfilingMode;
import java.util.List;

public class RobotContainer {
  /** Whether to use arcade drive or tank drive for robot navigation. */
  private static final boolean USE_ARCADE_DRIVE = true;

  /** The drivebase subsystem. */
  private final IDrivebasePlus m_drivebase = Robot.isReal() ? new Drivebase() : new SimDrivebase();

  /** The elevator subsystem.  (At present, always simulated.) */
  private final IElevator m_elevator = new frc.robot.subsystems.simulated.SimElevator();

  /** The arm subsystem.  (At present, always simulated.) */
  private final ISingleJointArm m_arm = new frc.robot.subsystems.simulated.SimArm();

  /** The driver joystick wrapper. */
  private final DriverJoystickWrapper m_driverWrapper =
      new DriverJoystickWrapper(OperatorConstants.DRIVER_JOYSTICK_ID,
          // Only load from/save to preferences when in simulation
          Robot.isSimulation());

  /** The autonomous command chooser. */
  private final SendableChooser<Command> m_autoCommandChooser = new SendableChooser<Command>();

  /** Constructor. */
  public RobotContainer() {
    configureDriving();
    configureBindings();
    configureSysIdCommands();
    configureElevatorCommands();
    configureArmCommands();
  }

  private void configureArmCommands() {
    // Note that the delays will not be enough to get the arm *absolutely* into position,
    // but that's OK: I'm just trying to make it wave back and forth....
    Command waveCommand =
        // Move out
        new SequentialCommandGroup(
            new InstantCommand(
                () -> { m_arm.setTargetPosition(m_arm.getArmOutAngle()); }, m_arm.asSubsystem()),
            new PrintCommand("Waiting for out"), new WaitCommand(2),
            new InstantCommand(
                () -> { m_arm.setTargetPosition(m_arm.getArmUpAngle()); }, m_arm.asSubsystem()),
            new PrintCommand("Waiting for up"), new WaitCommand(2))
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

  private void configureSysIdCommands() {
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
  }

  private void configureBindings() {
    // Set up autonomous command chooser
    m_autoCommandChooser.setDefaultOption(
        "No Auto", Commands.print("No autonomous command configured"));
    m_autoCommandChooser.addOption("Do something", Commands.print("Do something"));
    m_autoCommandChooser.addOption("Trajectory (Linear)",
        new FollowTrajectoryCommand(m_drivebase, generateTrajectory(TrajectoryShape.Linear)));
    m_autoCommandChooser.addOption("Trajectory (Curved)",
        createTrajectoryCommand(generateTrajectory(TrajectoryShape.SimpleCurve)));
    m_autoCommandChooser.addOption(
        "Trajectory (Circle)", createTrajectoryCommand(generateTrajectory(TrajectoryShape.Circle)));
    m_autoCommandChooser.addOption("Trajectory (S-curve)",
        createTrajectoryCommand(generateTrajectory(TrajectoryShape.SCurve)));
    SmartDashboard.putData("Autonomous Command", m_autoCommandChooser);
  }

  /** Returns the command to run in autonomous mode. */
  public Command getAutonomousCommand() {
    if (m_autoCommandChooser.getSelected() != null) {
      return m_autoCommandChooser.getSelected();
    }
    return Commands.print("No selection found for autonomous command");
  }

  enum TrajectoryShape { Linear, SimpleCurve, SCurve, Circle }

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

  private final static double kMaxVoltageForSampleTrajectories = 10;
  private final static LinearAcceleration maxAccelerationForSampleTrajectories =
      MetersPerSecondPerSecond.of(3);

  private final TrajectoryConfig m_trajectoryConfig =
      new TrajectoryConfig(m_drivebase.getMaxLinearSpeed(), maxAccelerationForSampleTrajectories)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(m_drivebase.getKinematics())
          // Apply a voltage constraint to ensure we don't accelerate too fast
          .addConstraint(
              new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(m_drivebase.getKs(),
                                                         m_drivebase.getKv(), m_drivebase.getKa()),
                  m_drivebase.getKinematics(), kMaxVoltageForSampleTrajectories));

  /**
   * Generates a command to drive along a trajectory of the specified shape.
   *
   * @param shape the desired shape of the trajectory
   * @return
   */
  Command createTrajectoryCommand(Trajectory trajectory) {
    return new FollowTrajectoryCommand(m_drivebase, generateTrajectory(shape));
  }
}