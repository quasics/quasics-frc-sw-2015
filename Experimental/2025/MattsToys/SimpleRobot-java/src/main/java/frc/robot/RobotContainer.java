// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.LogitechDualshock;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmWaveCommand;
import frc.robot.commands.DriveTeamShootingSupport;
import frc.robot.commands.MoveArmToAngle;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.RainbowLighting;
import frc.robot.commands.SimpleElevatorMover;
import frc.robot.subsystems.abstracts.AbstractElevator;
import frc.robot.subsystems.interfaces.ICandle;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.ILighting;
import frc.robot.subsystems.interfaces.ISingleJointArm;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.subsystems.live.Arm;
import frc.robot.subsystems.live.Candle;
import frc.robot.subsystems.live.Drivebase;
import frc.robot.subsystems.live.Elevator;
import frc.robot.subsystems.live.Lighting;
import frc.robot.subsystems.live.Vision;
import frc.robot.subsystems.simulations.SimCandle;
import frc.robot.subsystems.simulations.SimDrivebase;
import frc.robot.subsystems.simulations.SimVisionWrapper;
import frc.robot.subsystems.simulations.SimulatedElevator;
import frc.robot.subsystems.simulations.SimulatedSingleJointArm;
import frc.robot.subsystems.simulations.SimulationUxSupport;
import frc.robot.utils.DeadbandEnforcer;
import frc.robot.utils.RobotConfigs;
import frc.robot.utils.RobotConfigs.RobotConfig;
import frc.robot.utils.SysIdGenerator;
import java.util.function.Supplier;

/**
 * RobotContainer for a demo (mostly simulation-oriented) robot.
 */
public class RobotContainer {
  public static final boolean CANDLE_SHOWS_SHOOTING_READY = false;

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
  final private IDrivebase m_drivebase = allocateDrivebase(m_robotConfig);
  final private AbstractElevator m_elevator = allocateElevator(m_robotConfig);
  final private ISingleJointArm m_arm = allocateArm(m_robotConfig);
  final private ILighting m_lighting = allocateLighting(m_robotConfig);
  @SuppressWarnings("unused") // Vision interacts via BulletinBoard
  final private IVision m_vision = allocateVision(m_robotConfig);
  final private ICandle m_candle = allocateCandle(m_robotConfig, m_lighting);

  // Controllers
  //
  // Note that we can also consider using CommandJoystick class instead of
  // Joystick. This would allow explicitly bind specific channels for X/Y (e.g.,
  // possibly simplifying live vs simulation handling by not requiring custom
  // value inversion), as well as directly providing "trigger factories" for
  // commands.
  //
  // Note also that live joysticks generally follow a different
  // orientation/coordinate system than the one used for the robot. (See
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
  // for details.)
  private final Joystick m_driveController = new Joystick(Constants.DriveTeam.DRIVER_JOYSTICK_ID);

  static final private boolean CHOREO_SHOULD_HANDLE_PATH_FLIPPING = false;

  /** Factory object for Choreo trajectories. */
  private final AutoFactory m_autoFactory =
      new AutoFactory(m_drivebase::getPose, m_drivebase::resetPose, m_drivebase::followTrajectory,
          CHOREO_SHOULD_HANDLE_PATH_FLIPPING, // If alliance flipping should be enabled
          m_drivebase.asSubsystem());

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
    configureArcadeDrive();
    configureDashboard();
    configureBindings();
    configurePeriodicOperations(robot);

    m_lighting.asSubsystem().setDefaultCommand(new RainbowLighting(m_lighting));

    if (CANDLE_SHOWS_SHOOTING_READY) {
      m_candle.asSubsystem().setDefaultCommand(new DriveTeamShootingSupport(m_candle));
    }
  }

  private void configurePeriodicOperations(TimedRobot robot) {
    // Once per cycle (under simulation), update the battery voltage based on the
    // current draw.
    if (Robot.isSimulation()) {
      robot.addPeriodic(() -> {
        SimulationUxSupport.instance.updateBatteryVoltageFromDraws();
      }, COMMAND_CYCLE_PERIOD);
    }
  }

  /**
   * Adds "SysID"-related commands to the dashboard, to support robot
   * characterization.
   */
  private void addSysIdControlsToDashboard() {
    SmartDashboard.putData("SysID: Quasistatic(fwd)",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, SysIdGenerator.Mode.Linear, Direction.kForward));
    SmartDashboard.putData("SysID: Quasistatic(rev)",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, SysIdGenerator.Mode.Linear, Direction.kReverse));
    SmartDashboard.putData("SysID: Dynamic(fwd)",
        SysIdGenerator.sysIdDynamic(m_drivebase, SysIdGenerator.Mode.Linear, Direction.kForward));
    SmartDashboard.putData("SysID: Dynamic(rev)",
        SysIdGenerator.sysIdDynamic(m_drivebase, SysIdGenerator.Mode.Linear, Direction.kReverse));

    // SysId commands for rotational actions (used to calculate kA-angular), for use
    // in estimating the moment of inertia (MOI).
    // See: https://choreo.autos/usage/estimating-moi/
    SmartDashboard.putData("SysID(rot): Quasistatic(fwd)",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, SysIdGenerator.Mode.Rotating, Direction.kForward));
    SmartDashboard.putData("SysID(rot): Quasistatic(rev)",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, SysIdGenerator.Mode.Rotating, Direction.kReverse));
    SmartDashboard.putData("SysID(rot): Dynamic(fwd)",
        SysIdGenerator.sysIdDynamic(m_drivebase, SysIdGenerator.Mode.Rotating, Direction.kForward));
    SmartDashboard.putData("SysID(rot): Dynamic(rev)",
        SysIdGenerator.sysIdDynamic(m_drivebase, SysIdGenerator.Mode.Rotating, Direction.kReverse));
  }

  /**
   * Configures the buttons on the dashboard.
   */
  private void configureDashboard() {
    addSysIdControlsToDashboard();

    SmartDashboard.putData("Wave arm", new ArmWaveCommand(m_arm));
    SmartDashboard.putData("Arm out", new MoveArmToAngle(m_arm, m_arm.getArmOutAngle()));
    SmartDashboard.putData("Arm up", new MoveArmToAngle(m_arm, m_arm.getArmUpAngle()));
    SmartDashboard.putData("Raise elevator (wait)",
        new MoveElevatorToPosition(m_elevator, AbstractElevator.TargetPosition.Top, true));
    SmartDashboard.putData("Raise elevator (nowait)",
        new MoveElevatorToPosition(m_elevator, AbstractElevator.TargetPosition.Top, false));

    SmartDashboard.putData(
        "Elevator up", new SimpleElevatorMover(m_elevator, SimpleElevatorMover.Direction.UP));
    SmartDashboard.putData(
        "Elevator down", new SimpleElevatorMover(m_elevator, SimpleElevatorMover.Direction.DOWN));

    // Trajectory commands
    SmartDashboard.putData("Demo path", generateCommandForChoreoTrajectory("Demo path"));
  }

  /** Sets "arcade drive" as the default operation for the drivebase. */
  private void configureArcadeDrive() {
    final DeadbandEnforcer deadbandEnforcer =
        new DeadbandEnforcer(Constants.DriveTeam.DRIVER_DEADBAND);
    Supplier<Double> forwardSupplier;
    Supplier<Double> rotationSupplier;

    if (Robot.isReal()) {
      // Configure the real robot.
      //
      // Note that we're inverting the values because Xbox controllers return
      // negative values when we push forward.
      forwardSupplier = ()
          ->
          - deadbandEnforcer.limit(m_driveController.getRawAxis(LogitechDualshock.LeftYAxis));
      rotationSupplier = ()
          ->
          - deadbandEnforcer.limit(m_driveController.getRawAxis(LogitechDualshock.RightXAxis));
    } else {
      // Configure the simulated robot
      //
      // Note that we're assuming a keyboard-based controller is actually being
      // used in the simulation environment (for now), and thus we want to use
      // axis 0&1 (from the "Keyboard 0" configuration).
      forwardSupplier = () -> deadbandEnforcer.limit(m_driveController.getRawAxis(0));
      rotationSupplier = () -> - deadbandEnforcer.limit(m_driveController.getRawAxis(1));
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
   * Generates a Choreo command for the specified trajectory, including initial
   * reset of odometry and explicit "stop" at the end.
   *
   * @param trajectoryName name of the trajectory being loaded
   * @return a command for the trajectory, or a no-op if it couldn't be found (in
   *         which case, a message is printed to stderr)
   *
   * @see #generateCommandForChoreoTrajectory(String, boolean)
   */
  protected Command generateCommandForChoreoTrajectory(String trajectoryName) {
    return generateCommandForChoreoTrajectory(trajectoryName, true, true);
  }

  /**
   * Generates a Choreo command for the specified trajectory, including an
   * explicit "stop" at the end.
   *
   * @param trajectoryName name of the trajectory being loaded
   * @param resetOdometry  if true, reset the robot's pose before running the
   *                       trajectory, based on the starting point in the
   *                       trajectory's data
   * @return a command for the trajectory, or a no-op if it couldn't be found (in
   *         which case, a message is printed to stderr)
   *
   * @see <a href="https://choreo.autos/choreolib/getting-started/">Choreo
   *      'Getting Started'</a>
   * @see <a href="https://choreo.autos/choreolib/auto-factory/">AutoFactory</a>
   */
  protected Command generateCommandForChoreoTrajectory(
      String trajectoryName, boolean resetOdometry) {
    return generateCommandForChoreoTrajectory(trajectoryName, resetOdometry, true);
  }

  /**
   * Generates a Choreo command for the specified trajectory.
   *
   * @param trajectoryName name of the trajectory being loaded
   * @param resetOdometry  if true, reset the robot's pose before running the
   *                       trajectory, based on the starting point in the
   *                       trajectory's data
   * @param stopAtEnd      if true, explicitly stop the robot at the end of the
   *                       trajectory
   * @return a command for the trajectory, or a no-op if it couldn't be found (in
   *         which case, a message is printed to stderr)
   *
   * @see <a href="https://choreo.autos/choreolib/getting-started/">Choreo
   *      'Getting Started'</a>
   * @see <a href="https://choreo.autos/choreolib/auto-factory/">AutoFactory</a>
   */
  protected Command generateCommandForChoreoTrajectory(
      String trajectoryName, boolean resetOdometry, boolean stopAtEnd) {
    // Per https://choreo.autos/choreolib/auto-factory/
    final Command startCommand =
        (resetOdometry ? m_autoFactory.resetOdometry(trajectoryName) : Commands.none());

    // Don't let the drive base continue moving after the trajectory is done (e.g.,
    // due to PID settings being left in place).
    final Command endCommand =
        (stopAtEnd ? new InstantCommand(() -> { m_drivebase.stop(); }, m_drivebase.asSubsystem())
                   : Commands.none());

    // Generate the actual trajectory-following command.
    try {
      return Commands.sequence(
          // Perform any "on start" actions
          startCommand,
          // Then do the thing
          m_autoFactory.trajectoryCmd(trajectoryName),
          // And finally, any "on end" actions
          endCommand);
    } catch (Exception e) {
      System.err.println("ERROR: Failed to load Choreo trajectory '" + trajectoryName + "'");
      e.printStackTrace();
      return Commands.none();
    }
  }

  /**
   * Generates a command for the specified PathPlanner trajectory.
   *
   * NOTE: THIS IS NOT YET FULLY IMPLEMENTED, AS CONFIGURATION OF THE AutoBuilder
   * FACTORY IS NOT YET BEING DONE.
   *
   * @param trajectoryName name of the trajectory being loaded
   * @return a command for the trajectory, or a no-op if it couldn't be found
   *
   * @see <a href="https://pathplanner.dev/pplib-getting-started.html">PathPlanner
   *      'Getting Started'</a>
   */
  protected static Command generateCommandForPathPlannerTrajectory(String trajectoryName) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(trajectoryName);

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      //
      // Note: the AutoBuilder would need to be configured first, based on auto files
      // created in the PathPlanner GUI app. (This would normally be done in the
      // constructor for the RobotContainer, prior to this function being invoked.)
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      // DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      System.err.println("Failed to load PathPlanner trajectory: " + trajectoryName);
      e.printStackTrace();
      return Commands.none();
    }
  }

  /**
   * Generates a command (based on a Choreo trajectory) to be used in Auto mode.
   *
   * Note: this function assumes that our position on the field is directly
   * mapping to our driver station location.
   *
   * @return a Choreo-based trajectory command, based on the robot's
   *         alliance/position
   */
  private Command generateChoreoAutoCommand() {
    var allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) {
      System.out.println("WARNING: Can't get alliance!");
      return Commands.none();
    }

    var positionOpt = DriverStation.getLocation();
    if (positionOpt.isEmpty()) {
      System.out.println("WARNING: Can't get position!");
      return Commands.none();
    }
    System.out.println("INFO: OK, we're " + allianceOpt.get() + "-" + positionOpt.getAsInt());

    // Simple matrix of choices: we know how to get to precisely 1 algae from each
    // of the red/blue starting points, and we'll assume that our position on the
    // field is directly mapping to our driver station location.

    switch (allianceOpt.get()) {
      case Blue:
        switch (positionOpt.getAsInt()) {
          case 1:
            return generateCommandForChoreoTrajectory("BStart-outside-to-south-algae");
          case 2:
            return generateCommandForChoreoTrajectory("BStart-center-to-center-algae");
          case 3:
            return generateCommandForChoreoTrajectory("BStart-inside-to-north-algae");
        }
        break;

      case Red:
        switch (positionOpt.getAsInt()) {
          case 1:
            return generateCommandForChoreoTrajectory("RStart-outside-to-south-algae");
          case 2:
            return generateCommandForChoreoTrajectory("RStart-center-to-center-algae");
          case 3:
            // "North" end, so the cage to field center
            return generateCommandForChoreoTrajectory("RStart-inside-to-north-algae");
        }
    }

    System.out.println("WARNING: Couldn't identify Choreo path");
    return Commands.none();
  }

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
      case eChoreo -> generateChoreoAutoCommand();
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
  private static AbstractElevator allocateElevator(RobotConfigs.RobotConfig config) {
    if (!config.hasElevator()) {
      return new AbstractElevator.NullElevator();
    }

    if (Robot.isReal()) {
      return new Elevator(config);
    } else {
      return new SimulatedElevator(config);
    }
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

    if (Robot.isReal()) {
      return new Vision(config);
    } else {
      return new SimVisionWrapper(config, new Vision(config));
    }
  }

  /**
   * Allocates a drive base subsystem object.
   *
   * @param config the target robot's configuration
   * @return a drive base subsystem for this robot (may be trivial)
   */
  private static IDrivebase allocateDrivebase(RobotConfigs.RobotConfig config) {
    if (!config.hasDrive()) {
      return new IDrivebase.NullDrivebase();
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
