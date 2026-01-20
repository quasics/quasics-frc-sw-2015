// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

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
import frc.robot.commands.TargetingSupportCommand;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.games.ReefscapeConstants;
import frc.robot.misc.FieldPlacementColorFunction;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.LightingBuffer;
import frc.robot.subsystems.PhotonVisionSingleCamera;
import frc.robot.subsystems.interfaces.ICandle;
import frc.robot.subsystems.interfaces.IDrivebasePlus;
import frc.robot.subsystems.interfaces.IElevator;
import frc.robot.subsystems.interfaces.IElevator.ElevatorPosition;
import frc.robot.subsystems.interfaces.ILighting.StockColor;
import frc.robot.subsystems.interfaces.ILighting;
import frc.robot.subsystems.interfaces.IPhotonVision;
import frc.robot.subsystems.interfaces.ISingleJointArm;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.subsystems.live.CANSparkMaxDrivebase;
import frc.robot.subsystems.simulated.CameraSimulator;
import frc.robot.subsystems.simulated.SimCandle;
import frc.robot.subsystems.simulated.SimDrivebase;
import frc.robot.util.DriverJoystickWrapper;
import frc.robot.util.RobotConfigLibrary;
import frc.robot.util.RobotConfigs;
import frc.robot.util.RobotConfigs.RobotConfig;
import frc.robot.util.SpeedMode;
import frc.robot.util.SpeedModeScaler;
import frc.robot.util.SysIdGenerator;
import frc.robot.util.SysIdGenerator.DrivebaseProfilingMode;
import java.util.List;
import java.util.function.Supplier;

/**
 * Core definitions for sample robot code (focused on demonstrating simulation
 * support).
 *
 * This is where the robot's subsystems, operator interfaces (joysticks,
 * gamepads, etc.), and the high-level commands that link them together are
 * defined and configured. It serves as the central hub for the declarative
 * setup of a command-based robot project using the WPILib software library
 */
public class RobotContainer {
  /** Whether to use arcade drive or tank drive for robot navigation. */
  private static final boolean USE_ARCADE_DRIVE = true;

  /**
   * If true, override the default configuration used for the lighting subsystem
   * while we're disabled. This provides an example of how the robot can provide
   * information to the drive team to indicate if it is correctly positioned on
   * the field, based on where a trajectory that will be run in Auto mode will
   * start.
   */
  private static final boolean OVERRIDE_DEFAULT_LIGHTING_WHILE_DISABLED = false;

  private static final RobotConfigLibrary.Robot DEFAULT_SIMULATION_ROBOT = RobotConfigLibrary.Robot.Simulation;
  private static final RobotConfigLibrary.Robot DEFAULT_LIVE_ROBOT = RobotConfigLibrary.Robot.Simulation;

  /**
   * The robot being targeted.
   *
   * TODO: Add selector support to the smart dashboard (and saving), along with
   * a notification that it won't take affect until restart (e.g., using
   * SmartDashboard.reportWarning()), or at least an indicator for the current
   * value.
   */
  final RobotConfigLibrary.Robot m_robotSelection = Robot.isReal() ? DEFAULT_LIVE_ROBOT : DEFAULT_SIMULATION_ROBOT;

  /** Selected robot's configuration data. */
  final RobotConfig m_robotConfig = RobotConfigLibrary.getConfig(m_robotSelection);

  /** The drivebase subsystem. */
  final IDrivebasePlus m_drivebase = allocateDrivebase(m_robotConfig);

  /** The elevator subsystem. (At present, always simulated.) */
  final IElevator m_elevator = new frc.robot.subsystems.simulated.SimElevator();

  /** The arm subsystem. (At present, always simulated.) */
  final ISingleJointArm m_arm = new frc.robot.subsystems.simulated.SimArm();

  /** Vision-processing subsystem. */
  final IVision m_vision = new PhotonVisionSingleCamera(
      RobotConfigLibrary.getConfig(RobotConfigLibrary.Robot.Simulation)
          .cameras()
          .get(0));

  /** Lighting subystem. */
  final ILighting m_lighting = new Lighting(m_robotConfig);

  /** CANdle */
  final ICandle m_candle = allocateCandle(m_robotConfig, m_lighting);

  /** Left-side righting. */
  final ILighting m_leftLighting = allocateSideLighting(m_robotConfig, m_lighting, true);

  /** Right-side righting. */
  final ILighting m_rightLighting = allocateSideLighting(m_robotConfig, m_lighting, false);

  /** The driver joystick wrapper. */
  final DriverJoystickWrapper m_driverWrapper = new DriverJoystickWrapper(OperatorConstants.DRIVER_JOYSTICK_ID,
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
    configureLightingCommands();
    maybeConfigureLightingWhenDisabled();
    configureBindings();

    if (Robot.isSimulation()) {
      new CameraSimulator(m_robotConfig, (IPhotonVision) m_vision);
    }
  }

  /**
   * Allocates the drivebase for the robot.
   * 
   * @param config robot configuration
   * @return the drivebase object to use for the robot
   */
  private static IDrivebasePlus allocateDrivebase(RobotConfig config) {
    if (config.drive() == null) {
      return new IDrivebasePlus.NullDrivebase();
    }

    return switch (config.drive().driveType()) {
      case Simulated ->
        new SimDrivebase(config.drive());
      case CanSparkMax ->
        new CANSparkMaxDrivebase(config.drive());
    };
  }

  /** Adds lighting commands to the SmartDashboard. */
  private void configureLightingCommands() {
    m_leftLighting.SetStripColor(StockColor.Gold);
    m_rightLighting.SetStripColor(StockColor.Maroon);

    SmartDashboard.putData("Target lighting",
        new TargetingSupportCommand(m_lighting,
            new Pose2d(ReefscapeConstants.MIDLINE,
                ReefscapeConstants.FIELD_WIDTH.div(2), new Rotation2d()),
            Meters.of(2), Meters.of(1), Degrees.of(5)));
  }

  /**
   * Optionally configures the lighting when the robot is disabled, so that it
   * indicates location vs. a "planned starting point", allowing the drive team
   * to ensure that the robot is correctly placed for Auto mode.
   */
  private void maybeConfigureLightingWhenDisabled() {
    if (m_lighting == null) {
      return;
    }

    if (OVERRIDE_DEFAULT_LIGHTING_WHILE_DISABLED) {
      // Repeating the definition for "Blue1" from the SimDrivebase options.
      // Note that for real use in positioning based on a trajectory to be
      // followed in auto mode, we might actually... you know, use the first
      // position in that trajectory.
      final Pose2d BLUE_1_POSE = new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters),
          ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters),
          new Rotation2d(ReefscapeConstants.FACING_BLUE));

      m_lighting.SetDisabledSupplier(new FieldPlacementColorFunction(
          // targetPoseSupplier
          () -> BLUE_1_POSE,
          // currentPoseSupplier
          //
          // Note: this should actually be coming from *vision* pose estimation,
          // since (while we're in "disabled" mode) we'll have no actual
          // odometry data to use for pose estimation. In this simulation
          // example, we *do*, because the selector used to start the robot at
          // different points on the field is also (currently) forcibly updating
          // the odometry data so that the field simulation will work; however,
          // this won't be the case for *real* robots.
          () -> IDrivebasePlus.getPublishedLastPoseFromOdometry()));
    }
  }

  /**
   * Adds various sample commands for controlling the single-joint arm to the
   * dashboard.
   */
  private void configureArmCommands() {
    // Note that the delays will not be enough to get the arm *absolutely* into
    // position, but that's OK: I'm just trying to make it wave back and
    // forth....
    Command waveCommand = new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              m_arm.setTargetPosition(m_arm.getArmOutAngle());
            },
            m_arm.asSubsystem()),
        // Wait for some motion
        new WaitCommand(2),
        new InstantCommand(
            () -> {
              m_arm.setTargetPosition(m_arm.getArmUpAngle());
            },
            m_arm.asSubsystem()),
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
    SmartDashboard.putData("Cmd: Arm stop",
        new InstantCommand(() -> {
          m_arm.stop();
        }, m_arm.asSubsystem()));
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
    SmartDashboard.putData("Cmd: Elevator stop", new InstantCommand(() -> {
      m_elevator.stop();
    }, m_elevator.asSubsystem()));
  }

  /** Basic value for "turtle" scaling. */
  static final double TURTLE_DRIVE_SCALING = 0.250;
  /** Basic value for "normal" scaling. */
  static final double NORMAL_DRIVE_SCALING = 0.50;
  /** Basic value for "turbo" scaling. */
  static final double TURBO_DRIVE_SCALING = 0.80;

  /** Configures the driving behavior. */
  private void configureDriving() {
    m_driverWrapper.setDeadbandThreshold(OperatorConstants.DEADBAND_THRESHOLD);

    // Slew rate controls: don't let things ramp up too quickly.
    SlewRateLimiter limiter1 = new SlewRateLimiter(OperatorConstants.MAX_SLEW_RATE);
    SlewRateLimiter limiter2 = new SlewRateLimiter(OperatorConstants.MAX_SLEW_RATE);

    // Drive "speed mode" decisions/scaling.
    Supplier<SpeedMode> speedModeSupplier = () -> {
      if (m_driverWrapper.isXButtonPressed()) {
        return SpeedMode.Turbo;
      } else if (m_driverWrapper.isYButtonPressed()) {
        return SpeedMode.Turtle;
      } else {
        return SpeedMode.Normal;
      }
    };
    final SpeedModeScaler scaler = new SpeedModeScaler(speedModeSupplier,
        NORMAL_DRIVE_SCALING, TURBO_DRIVE_SCALING, TURTLE_DRIVE_SCALING);

    // Order of application: raw values are speed scaled, and then filtered by
    // slew limits.
    if (USE_ARCADE_DRIVE) {
      m_drivebase.asSubsystem().setDefaultCommand(new ArcadeDrive(m_drivebase,
          () -> limiter1.calculate(
              scaler.apply(m_driverWrapper.getArcadeForward())),
          () -> limiter2.calculate(
              scaler.apply(m_driverWrapper.getArcadeRotation()))));
    } else {
      m_drivebase.asSubsystem().setDefaultCommand(new TankDrive(m_drivebase,
          () -> limiter1.calculate(
              scaler.apply(m_driverWrapper.getTankLeft())),
          () -> limiter2.calculate(
              scaler.apply(m_driverWrapper.getTankRight()))));
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
    m_autoCommandChooser.addOption(
        "Do something", Commands.print("Do something"));
    m_autoCommandChooser.addOption("Trajectory (Linear)",
        new FollowTrajectoryCommand(
            m_drivebase, generateTrajectory(TrajectoryShape.Linear)));
    m_autoCommandChooser.addOption("Trajectory (Curved)",
        new FollowTrajectoryCommand(
            m_drivebase, generateTrajectory(TrajectoryShape.SimpleCurve)));
    m_autoCommandChooser.addOption("Trajectory (Circle)",
        new FollowTrajectoryCommand(
            m_drivebase, generateTrajectory(TrajectoryShape.Circle)));
    m_autoCommandChooser.addOption("Trajectory (S-curve)",
        new FollowTrajectoryCommand(
            m_drivebase, generateTrajectory(TrajectoryShape.SCurve)));
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
  enum TrajectoryShape {
    Linear, SimpleCurve, SCurve, Circle
  }

  /** Maximum desired voltage draw when performing trajectory-following. */
  private static final double kMaxVoltageForSampleTrajectories = 10;

  /** Maximum desired acceleration when performing trajectory-following. */
  private static final LinearAcceleration maxAccelerationForSampleTrajectories = MetersPerSecondPerSecond.of(3);

  /** Configuration for use in generating sample trajectories. */
  private final TrajectoryConfig m_trajectoryConfig =
      // Base configuration (max speed/accelleration)
      new TrajectoryConfig(
          m_drivebase.getMaxLinearSpeed(), maxAccelerationForSampleTrajectories)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(m_drivebase.getKinematics())
          // Apply a voltage constraint to ensure we don't accelerate too fast
          // (and brown us out)
          .addConstraint(new DifferentialDriveVoltageConstraint(
              m_drivebase.getFeedForward(), m_drivebase.getKinematics(),
              kMaxVoltageForSampleTrajectories));

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
            // Pass through these two interior waypoints, making an 's' curve
            // path
            List.of(new Translation2d(2, 1), new Translation2d(4, -1)),
            // End 6 meters straight ahead of where we started, facing forward
            new Pose2d(6, 0, new Rotation2d(0)),
            // Pass config
            m_trajectoryConfig);

      case Circle ->
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these three interior waypoints along the perimeter
            // of a circle
            List.of(new Translation2d(2, 2), new Translation2d(0, 4),
                new Translation2d(-2, 2)),
            // End back where we started, facing forward
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass config
            m_trajectoryConfig);
    };
  }

  /**
   * Allocates an ICandle subsystem object.
   *
   * @param config the target robot's configuration
   * @return an ICandle subsystem for this robot (may be trivial)
   */
  private static ICandle allocateCandle(RobotConfigs.RobotConfig config, ILighting lighting) {
    if (!config.hasCandle()) {
      return new ICandle.NullCandle();
    }

    if (Robot.isReal()) {
      return new Candle(config);
    } else {
      Lighting realSubsystem = (Lighting) lighting;
      if (realSubsystem.getSubViews().isEmpty()) {
        return new ICandle.NullCandle();
      }
      return new SimCandle(realSubsystem.getSubViews().get(0));
    }
  }

  private static ILighting allocateSideLighting(RobotConfigs.RobotConfig config, ILighting lighting, boolean leftSide) {
    Lighting realSubsystem = (Lighting) lighting;
    final boolean simulatingCandle = (config.hasCandle() && config.candle().simulated());
    // Sub-view 0 is CANdle (if enabled); next sub-view is left, then right
    final int viewIndex = (simulatingCandle ? 1 : 0) + (leftSide ? 0 : 1);
    if (realSubsystem.getSubViews().size() < (viewIndex + 1)) {
      return new ILighting.NullLighting();
    }
    return new LightingBuffer(realSubsystem.getSubViews().get(viewIndex));
  }
}
