// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.LogitechDualshock;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmWaveCommand;
import frc.robot.commands.DriveTeamShootingSupport;
import frc.robot.commands.DriveToTarget;
import frc.robot.commands.MoveArmToAngle;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.RainbowLighting;
import frc.robot.commands.SimpleElevatorMover;
import frc.robot.commands.TurnToTarget;
import frc.robot.subsystems.interfaces.ICandle;
import frc.robot.subsystems.interfaces.IElevator;
import frc.robot.subsystems.interfaces.ILighting;
import frc.robot.subsystems.interfaces.ISingleJointArm;
import frc.robot.subsystems.interfaces.drivebase.IDrivebase;
import frc.robot.subsystems.interfaces.drivebase.IDrivebasePlus;
import frc.robot.subsystems.interfaces.vision.IVision;
import frc.robot.subsystems.interfaces.vision.IVisionPlus;
import frc.robot.subsystems.live.Arm;
import frc.robot.subsystems.live.BetterVision;
import frc.robot.subsystems.live.Candle;
import frc.robot.subsystems.live.Drivebase;
import frc.robot.subsystems.live.Elevator;
import frc.robot.subsystems.live.Lighting;
import frc.robot.subsystems.simulations.CameraSimulator;
import frc.robot.subsystems.simulations.SimCandle;
import frc.robot.subsystems.simulations.SimDrivebase;
import frc.robot.subsystems.simulations.SimulatedElevator;
import frc.robot.subsystems.simulations.SimulatedSingleJointArm;
import frc.robot.subsystems.simulations.SimulationUxSupport;
import frc.robot.utils.DeadbandEnforcer;
import frc.robot.utils.RobotConfigs;
import frc.robot.utils.RobotConfigs.RobotConfig;
import frc.robot.utils.StateChangeExecutor;
import frc.robot.utils.SysIdGenerator;
import frc.robot.utils.logging.EventLogger;
import frc.robot.utils.logging.StringEventLogger;
import java.util.function.Supplier;

/**
 * This class serves as the central hub for the declarative setup of our
 * "command-based" robot project, under the standard WPILib definition for this
 * construct.
 *
 * @see
 *     https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html#robotcontainer
 */
public class RobotContainer {
  /** Iff true, allow Choreo to handle flipping any paths used with it. */
  static final private boolean CHOREO_SHOULD_HANDLE_PATH_FLIPPING = false;

  /**
   * Iff true, use the CANdl hardware to show when the robot is in position to
   * take a shot at the barge.
   */
  public static final boolean CANDL_SHOWS_SHOOTING_READY = true;

  /** Defines options for selecting auto mode commands. */
  enum AutoModeOperation {
    /** Do nothing in auto mode. */
    eDoNothing,
    /** Move forward and raise the elevator in auto mode. */
    eMoveAndRaise,
    /**
     * Follow a Choreo-based trajectory in auto mode, based on starting position.
     */
    eChoreo,
  }

  /** Option to be used for auto mode actions. */
  static final AutoModeOperation AUTO_MODE_OPTION = AutoModeOperation.eChoreo;

  /** Indicates the robot we are going to target. */
  final RobotConfigs.Robot DEPLOYED_ON = RobotConfigs.Robot.Simulation;

  /** Configuration data for the targeted robot. */
  final RobotConfig m_robotConfig = RobotConfigs.getConfig(DEPLOYED_ON);

  // Subsystems
  /** Interface to drive base. */
  final private IDrivebase m_drivebase = allocateDrivebase(m_robotConfig);

  /** Interface to elevator. */
  final private IElevator m_elevator = allocateElevator(m_robotConfig);

  /** Interface to arm. */
  final private ISingleJointArm m_arm = allocateArm(m_robotConfig);

  /** Interface to lighting subsystem. */
  final private ILighting m_lighting = allocateLighting(m_robotConfig);

  /** Interface to vision subsystem. */
  final private IVision m_vision = allocateVision(m_robotConfig);

  /** Interface to CANdl hardware. */
  final private ICandle m_candle = allocateCandle(m_robotConfig, m_lighting);

  /** Camera simulation injector (if running in simulation mode). */
  @SuppressWarnings("unused") // Camera simulator is pure data injection
  final private CameraSimulator m_cameraSimulator =
      maybeAllocateCameraSimulator(m_robotConfig, m_vision);

  /** Primary logger. */
  final EventLogger m_eventLogger = new StringEventLogger();

  /**
   * Controller for the drive base.
   *
   * Note that we can also consider using CommandJoystick class instead of
   * Joystick. This would allow explicitly bind specific channels for X/Y (e.g.,
   * possibly simplifying live vs simulation handling by not requiring custom
   * value inversion), as well as directly providing "trigger factories" for
   * commands.
   *
   * Note also that live joysticks generally follow a different
   * orientation/coordinate system than the one used for the robot.
   *
   * @see
   *     https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#joystick-and-controller-coordinate-system
   */
  private final Joystick m_driveController = new Joystick(Constants.DriveTeam.DRIVER_JOYSTICK_ID);

  /** Normal cycle time on command-handling (50 Hz). */
  private static final Time COMMAND_CYCLE_PERIOD = Seconds.of(1.0 / 5.0);

  /**
   * Constructor.
   *
   * @param robot the robot object to which this container is attached; used to
   *              (optionally) set up a periodic operation to update the battery
   *              data under simulation
   */
  public RobotContainer(TimedRobot robot) {
    EventLogger.setDefaultLogger(m_eventLogger);

    configureArcadeDrive();
    configureDashboard();
    configureBindings();
    configurePeriodicOperations(robot);

    m_lighting.asSubsystem().setDefaultCommand(new RainbowLighting(m_lighting));

    if (CANDL_SHOWS_SHOOTING_READY) {
      m_candle.asSubsystem().setDefaultCommand(new DriveTeamShootingSupport(m_candle));
    }
  }

  /**
   * Configure anything that we want to have happen on a recurring process, which
   * isn't bound to a specific subsystem (or command).
   *
   * @param robot the robot whose overall scheduling is being hooked into
   */
  private void configurePeriodicOperations(TimedRobot robot) {
    // Once per cycle (under simulation), update the battery voltage based on the
    // current draw.
    if (Robot.isSimulation()) {
      robot.addPeriodic(() -> {
        SimulationUxSupport.instance.updateBatteryVoltageFromDraws();
      }, COMMAND_CYCLE_PERIOD);
    }

    // Testing event logging: dump the contents whenever we're disabled
    if (m_eventLogger != null && m_eventLogger instanceof StringEventLogger) {
      final StateChangeExecutor executor = new StateChangeExecutor(
          // State supplier
          ()
              -> { return DriverStation.isDisabled(); },
          // Assumed initial state (i.e., assume we're disabled on startup)
          true,
          // Action
          ()
              -> {
            System.err.println("Dumping event log:");
            System.err.println(((StringEventLogger) m_eventLogger).getContents());
          },
          // Triggering mode
          StateChangeExecutor.Mode.GoesTrue);
      robot.addPeriodic(() -> { executor.check(); }, COMMAND_CYCLE_PERIOD);
    }
  }

  /**
   * Adds "SysID"-related commands to the dashboard, to support robot
   * characterization.
   */
  private void addSysIdControlsToDashboard() {
    // SysId commands for linear motion of the drive base. (Basic speed control.)
    SmartDashboard.putData("Drive SysID: Quasistatic(fwd)",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, SysIdGenerator.DrivebaseProfilingMode.Linear, Direction.kForward));
    SmartDashboard.putData("Drive SysID: Quasistatic(rev)",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, SysIdGenerator.DrivebaseProfilingMode.Linear, Direction.kReverse));
    SmartDashboard.putData("Drive SysID: Dynamic(fwd)",
        SysIdGenerator.sysIdDynamic(
            m_drivebase, SysIdGenerator.DrivebaseProfilingMode.Linear, Direction.kForward));
    SmartDashboard.putData("Drive SysID: Dynamic(rev)",
        SysIdGenerator.sysIdDynamic(
            m_drivebase, SysIdGenerator.DrivebaseProfilingMode.Linear, Direction.kReverse));

    // SysId commands for rotational actions (used to calculate kA-angular), for use
    // in estimating the moment of inertia (MOI).
    // See: https://choreo.autos/usage/estimating-moi/
    SmartDashboard.putData("Drive SysID(rot): Quasistatic(fwd)",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, SysIdGenerator.DrivebaseProfilingMode.Rotating, Direction.kForward));
    SmartDashboard.putData("Drive SysID(rot): Quasistatic(rev)",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, SysIdGenerator.DrivebaseProfilingMode.Rotating, Direction.kReverse));
    SmartDashboard.putData("Drive SysID(rot): Dynamic(fwd)",
        SysIdGenerator.sysIdDynamic(
            m_drivebase, SysIdGenerator.DrivebaseProfilingMode.Rotating, Direction.kForward));
    SmartDashboard.putData("Drive SysID(rot): Dynamic(rev)",
        SysIdGenerator.sysIdDynamic(
            m_drivebase, SysIdGenerator.DrivebaseProfilingMode.Rotating, Direction.kReverse));

    // SysId commands for linear motion of the elevator. (Basic speed control.)
    SmartDashboard.putData("Elevator SysID: Quasistatic(fwd)",
        SysIdGenerator.sysIdQuasistatic(m_elevator, Direction.kForward));
    SmartDashboard.putData("Elevator SysID: Quasistatic(rev)",
        SysIdGenerator.sysIdQuasistatic(m_elevator, Direction.kReverse));
    SmartDashboard.putData("Elevator SysID: Dynamic(fwd)",
        SysIdGenerator.sysIdDynamic(m_elevator, Direction.kForward));
    SmartDashboard.putData("Elevator SysID: Dynamic(rev)",
        SysIdGenerator.sysIdDynamic(m_elevator, Direction.kReverse));
  }

  /**
   * Add commands to the dashboard that depend on advanced vision capabilities.
   */
  private void maybeAddVisionCommandsToDashboard() {
    if (!(m_vision instanceof IVisionPlus)) {
      return;
    }
    final IVisionPlus visionPlus = ((IVisionPlus) m_vision);

    final int targetId = 17;
    SmartDashboard.putData(
        "Turn to target " + targetId, new TurnToTarget(visionPlus, m_drivebase, targetId));
    SmartDashboard.putData("Turn & Drive to target " + targetId,
        new SequentialCommandGroup(
            // First, turn until it's in view...
            new TurnToTarget(
                visionPlus, m_drivebase, targetId, TurnToTarget.OpMode.TargetInView, false),
            // ...and then drive to it.
            new DriveToTarget(visionPlus, m_drivebase, targetId, true)));
  }

  /**
   * Add commands to the dashboard for testing some commands I'm hacking together
   * (if they can be supported).
   */
  private void maybeAddHackingCommandsToDashboard() {
    if (!(m_drivebase instanceof IDrivebasePlus)) {
      return;
    }

    // final IDrivebasePlus drivebasePlus = ((IDrivebasePlus) m_drivebase);
    // SmartDashboard.putData("TrajHacking", new TrajectoryHacking(drivebasePlus, m_robotConfig));
  }

  /**
   * Configures the buttons on the dashboard.
   */
  private void configureDashboard() {
    addSysIdControlsToDashboard();
    maybeAddVisionCommandsToDashboard();
    maybeAddHackingCommandsToDashboard();

    SmartDashboard.putData("Wave arm", new ArmWaveCommand(m_arm));
    SmartDashboard.putData("Arm out", new MoveArmToAngle(m_arm, m_arm.getArmOutAngle()));
    SmartDashboard.putData("Arm up", new MoveArmToAngle(m_arm, m_arm.getArmUpAngle()));
    SmartDashboard.putData("Raise elevator (wait)",
        new MoveElevatorToPosition(m_elevator, IElevator.TargetPosition.Top, true));
    SmartDashboard.putData("Raise elevator (nowait)",
        new MoveElevatorToPosition(m_elevator, IElevator.TargetPosition.Top, false));

    SmartDashboard.putData(
        "Elevator up", new SimpleElevatorMover(m_elevator, SimpleElevatorMover.Direction.UP));
    SmartDashboard.putData(
        "Elevator down", new SimpleElevatorMover(m_elevator, SimpleElevatorMover.Direction.DOWN));
  }

  /** Sets "arcade drive" as the default operation for the drivebase. */
  private void configureArcadeDrive() {
    final DeadbandEnforcer deadbandEnforcer =
        new DeadbandEnforcer(Constants.DriveTeam.DRIVER_DEADBAND);
    Supplier<Double> forwardSupplier;
    Supplier<Double> rotationSupplier;

    // Limiting the rate-of-change for velocity (i.e., acceleration) to constrain us
    // from getting to
    // 100% in anything less than 1/MAX_SLEW_RATE seconds.
    SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(Constants.Driving.MAX_SLEW_RATE);
    SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(Constants.Driving.MAX_SLEW_RATE);

    if (Robot.isReal()) {
      // Configure the real robot.
      //
      // Note that we're inverting the values because Xbox controllers return
      // negative values when we push forward.
      forwardSupplier = ()
          ->
          - forwardSlewRateLimiter.calculate(
              deadbandEnforcer.limit(m_driveController.getRawAxis(LogitechDualshock.LeftYAxis)));
      rotationSupplier = ()
          ->
          - rotationSlewRateLimiter.calculate(
              deadbandEnforcer.limit(m_driveController.getRawAxis(LogitechDualshock.RightXAxis)));
    } else {
      // Configure the simulated robot
      //
      // Note that we're assuming a keyboard-based controller is actually being
      // used in the simulation environment (for now), and thus we want to use
      // axis 0&1 (from the "Keyboard 0" configuration).
      forwardSupplier = ()
          -> forwardSlewRateLimiter.calculate(
              deadbandEnforcer.limit(m_driveController.getRawAxis(0)));
      rotationSupplier = ()
          ->
          - rotationSlewRateLimiter.calculate(
              deadbandEnforcer.limit(m_driveController.getRawAxis(1)));
    }

    m_drivebase.asSubsystem().setDefaultCommand(
        new ArcadeDrive(m_drivebase, forwardSupplier, rotationSupplier));
  }

  /**
   * Configures any additional command bindings (e.g., to buttons on game
   * controllers, etc.).
   */
  private void configureBindings() {
    // No-op at present; retained for future use if needed.
  }

  ////////////////////////////////////////////////////////////////////////////////
  //
  // Factory methods for commands to be executed by the rbot.
  //
  ////////////////////////////////////////////////////////////////////////////////

  /**
   * Returns the command to be run in Auto mode, based on the configured option.
   *
   * @return a command to be run in Auto mode
   *
   * @see #AUTO_MODE_OPTION
   */
  public Command getAutonomousCommand() {
    return switch (AUTO_MODE_OPTION) {
      case eDoNothing -> Commands.print("No autonomous command configured");
      case eChoreo -> Commands.print("Choreo-based autonomous not yet implemented");
      case eMoveAndRaise ->
        new ParallelCommandGroup(
            new frc.robot.commands.DriveForDistance(m_drivebase, .50, Units.Meters.of(3)),
            new frc.robot.commands.MoveElevatorToExtreme(m_elevator, true));
    };
  }

  ////////////////////////////////////////////////////////////////////////////////
  //
  // Factory methods for our subsystems
  //
  ////////////////////////////////////////////////////////////////////////////////

  /**
   * Allocates an elevator subsystem object.
   *
   * @param config the target robot's configuration
   * @return an elevator subsystem for this robot (may be trivial)
   */
  private static IElevator allocateElevator(RobotConfigs.RobotConfig config) {
    if (!config.hasElevator()) {
      return new IElevator.NullElevator();
    }

    if (Robot.isReal()) {
      return new Elevator(config);
    } else {
      return new SimulatedElevator(config);
    }
  }

  /**
   * Allocates a camera simulation injector, if needed.
   */
  private static CameraSimulator maybeAllocateCameraSimulator(
      RobotConfigs.RobotConfig config, IVision vision) {
    assert vision != null : "Vision subsystem must be allocated before camera simulation setup";
    if (vision == null) {
      throw new IllegalArgumentException(
          "Vision subsystem must be allocated before camera simulation setup");
    }

    if (Robot.isReal()) {
      return null;
    }

    return new CameraSimulator(config, vision);
  }

  /**
   * Allocates a Vision subsystem object.
   *
   * @param config the target robot's configuration
   * @return a Vision subsystem for this robot (may be trivial)
   */
  private static IVision allocateVision(RobotConfigs.RobotConfig config) {
    if (!config.hasCamera()) {
      return new IVision.NullVision();
    }

    // TODO: Consider adding code to switch between SimpleVision and BetterVision
    // instances, depending on the number of cameras, so that both can be tested.
    return new BetterVision(config);
  }

  /**
   * Allocates a drive base subsystem object.
   *
   * @param config the target robot's configuration
   * @return a drive base subsystem for this robot (may be trivial)
   */
  private static IDrivebasePlus allocateDrivebase(RobotConfigs.RobotConfig config) {
    if (!config.hasDrive()) {
      return new IDrivebasePlus.NullDrivebase();
    }

    if (Robot.isReal()) {
      return new Drivebase(config);
    } else {
      return new SimDrivebase(config);
    }
  }

  /**
   * Allocates a lighting subsystem object.
   *
   * @param config the target robot's configuration
   * @return a lighting subsystem for this robot (may be trivial)
   */
  private static ILighting allocateLighting(RobotConfigs.RobotConfig config) {
    if (!config.hasLighting()) {
      return new ILighting.NullLighting();
    }

    return new Lighting(config);
  }

  /**
   * Allocates a single-joint arm subsystem object.
   *
   * @param config the target robot's configuration
   * @return a single-joint arm subsystem for this robot (may be trivial)
   */
  private static ISingleJointArm allocateArm(RobotConfigs.RobotConfig config) {
    if (!config.hasArm()) {
      return new ISingleJointArm.NullArm();
    }

    if (Robot.isReal()) {
      return new Arm(config);
    } else {
      return new SimulatedSingleJointArm(config);
    }
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
}
