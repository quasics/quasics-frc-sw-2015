// Copyright (c) 2024, FIRST, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DriveForDistance;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.ExtendClimbers;
import frc.robot.commands.RetractClimbers;
import frc.robot.commands.SetClimberSafetyMode;
import frc.robot.commands.RainbowLighting;
import frc.robot.commands.SpinInPlace;
import frc.robot.subsystems.AbstractDrivebase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.LightingInterface;
import frc.robot.subsystems.RealDrivebase;
import frc.robot.subsystems.RomiDrivebase;
import frc.robot.subsystems.SimulationDrivebase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.XrpDrivebase;
import frc.robot.utils.RobotSettings;
import frc.robot.utils.TrajectoryCommandGenerator;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.Supplier;

/**
 * TODO: Add support for identifying initial field location (at start of match).
 */
public class RobotContainer {
  final static boolean ENABLE_VISION_SUBSYSTEM = true;

  static final double MAX_AUTO_VELOCITY_MPS = 3;
  static final double MAX_AUTO_ACCELERATION_MPSS = 1;
  static final TrajectoryConfig AUTO_SPEED_PROFILE = new TrajectoryConfig(MAX_AUTO_VELOCITY_MPS,
      MAX_AUTO_ACCELERATION_MPSS);

  /** Options for the behavior when running under the simulator. */
  private enum SimulationMode {
    eSimulation, eXrp, eRomi
  }

  /** Defines options for auto mode. */
  private enum AutoMode {
    eDoNothing, eSpin, eFollowTrajectory
  }

  /** Default configuration when running in "isReal" mode. */
  private static final RobotSettings.Robot SETTINGS_FOR_REAL_MODE = RobotSettings.Robot.Margaret;

  private Joystick m_driveController = new Joystick(Constants.DriveTeam.DRIVER_JOYSTICK_ID);
  private final LightingInterface m_lighting = new Lighting(getRobotSettings());
  private final AbstractDrivebase m_drivebase;
  private final TrajectoryCommandGenerator m_trajectoryCommandGenerator;
  private final VisionSubsystem m_vision;
  private final Climber m_climber;

  Supplier<Double> m_arcadeDriveForwardStick;
  Supplier<Double> m_arcadeDriveRotationStick;

  /** Currently-configured option when running under the simulator. */
  private static final SimulationMode m_simulationMode = SimulationMode.eSimulation;

  /** The currently-configured autonomous mode. */
  private AutoMode mode = AutoMode.eDoNothing;

  /** Defines options for building sample trajectory commands. */
  private enum AutoModeTrajectorySelection {
    eControlSystemExampleFunctionalCommand,
    eControlSystemExampleRamseteCommand,
    eTrajectoryCommandGeneratorExample
  }

  private static final AutoModeTrajectorySelection m_autoModeTrajectorySelection = AutoModeTrajectorySelection.eControlSystemExampleRamseteCommand;

  public static RobotSettings.Robot getRobotSettings() {
    if (RobotBase.isReal()) {
      return SETTINGS_FOR_REAL_MODE;
    }

    switch (m_simulationMode) {
      case eRomi:
        return RobotSettings.Robot.Romi;
      case eXrp:
        return RobotSettings.Robot.Xrp;

      case eSimulation:
      default:
        return RobotSettings.Robot.Simulator;
    }
  }

  /*
   * Constructor.
   *
   * @todo Add sample solution for selecting which (real) drive base we're
   * actually working on.
   */
  public RobotContainer() {
    ////////////////////////////////////////////////////////
    // Subsystem setup
    m_drivebase = setupDriveBase();
    m_vision = maybeSetupVisionSubsystem();
    m_trajectoryCommandGenerator = new TrajectoryCommandGenerator(m_drivebase);
    if (getRobotSettings().hasClimber) {
      m_climber = new Climber();
    } else {
      m_climber = null;
    }

    ////////////////////////////////////////////////////////
    // Finish intialization
    resetPositionFromAllianceSelection();

    ////////////////////////////////////////////////////////
    // Set up button bindings and smart dashboard buttons
    configureBindings();

    // Tags 9&10 are on the Blue Source wall
    SmartDashboard.putData("Target 10",
        new DriveToAprilTag(getRobotSettings(), m_vision, m_drivebase, 10,
            Constants.AprilTags.SOURCE_TAG_BOTTOM_HEIGHT, Meters.of(.5)));

    maybeAddClimberCommandsToDashboard();

    ////////////////////////////////////////////////////////
    // Report other information for dev support
    if (RobotBase.isSimulation()) {
      System.err.println("Writing logs to: " + DataLogManager.getLogDir());
    } else {
      System.err.println("Logs should be written to: ~lvuser/logs directory");
    }
  }

  private AbstractDrivebase setupDriveBase() {
    AbstractDrivebase drivebase = null;
    if (Robot.isReal()) {
      drivebase = new RealDrivebase(getRobotSettings());

      // Note that we're inverting the values because Xbox controllers return
      // negative values when we push forward.
      m_arcadeDriveForwardStick = () -> -m_driveController.getRawAxis(Constants.LogitechGamePad.LeftYAxis);
      m_arcadeDriveRotationStick = () -> -m_driveController.getRawAxis(Constants.LogitechGamePad.RightXAxis);
    } else {
      // Note that we're assuming a keyboard-based controller is actually being
      // used in the simulation environment (for now), and thus we want to use
      // axis 1&2.
      m_arcadeDriveForwardStick = () -> -m_driveController.getRawAxis(0);
      m_arcadeDriveRotationStick = () -> -m_driveController.getRawAxis(1);
      System.out.println("Simulator configured for " + m_simulationMode + " mode");
      switch (m_simulationMode) {
        case eRomi:
          drivebase = new RomiDrivebase();
          break;
        case eXrp:
          drivebase = new XrpDrivebase();
          break;
        case eSimulation:
          drivebase = new SimulationDrivebase(RobotSettings.Robot.Simulator);
          break;
        default:
          System.err.println("**** WARNING: Unrecognized simulation mode (" + m_simulationMode
              + "), falling back on pure simulator");
          drivebase = new SimulationDrivebase(RobotSettings.Robot.Simulator);
          break;
      }
    }
    return drivebase;
  }

  private VisionSubsystem maybeSetupVisionSubsystem() {
    VisionSubsystem vision = null;
    if (ENABLE_VISION_SUBSYSTEM) {
      try {
        vision = new VisionSubsystem(getRobotSettings());
      } catch (Exception e) {
        System.err.println("*** Failed to set up vision subsystem!");
        e.printStackTrace();
      }
    } else {
      System.err.println(">>> Note: Vision subsystem is DISABLED.");
    }

    return vision;
  }

  private void configureBindings() {
    m_drivebase.setDefaultCommand(
        new ArcadeDrive(m_drivebase, m_arcadeDriveForwardStick, m_arcadeDriveRotationStick));
    m_lighting.setDefaultCommand(new RainbowLighting(m_lighting));

    // Bind full set of SysId routine tests to buttons on the SmartDashboard; a
    // complete routine should run each of these once.
    SmartDashboard.putData(
        "Quasistatic Fwd", m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData(
        "Quasistatic Rev", m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData(
        "Dynamic Fwd", m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData(
        "Dynamic Rev", m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    if (ENABLE_VISION_SUBSYSTEM) {
      SmartDashboard.putData("Reset position",
          new InstantCommand(() -> resetPositionFromAllianceSelection(), m_drivebase, m_vision));
    }

    SmartDashboard.putData(
        "Drive 1m @ 1mps", new DriveForDistance(m_drivebase, Meters.of(1), MetersPerSecond.of(1)));
  }

  Pose2d computeInitialPoseForDriversStation(Alliance alliance, int driversStation) {
    // Assume we're always facing *out* from the driver's station (for now),
    // and are right in front of it.
    Measure<Angle> initialAngle;
    Measure<Distance> xPos = Meters.of(0), yPos = Meters.of(0);
    Measure<Distance> halfLength = m_drivebase.getLengthIncludingBumpers().divide(2);
    if (alliance == Alliance.Blue) {
      initialAngle = Degrees.of(0);
      xPos = Meters.of(1.088).plus(halfLength);
      if (driversStation == 1) {
        yPos = Meters.of(2.265);
      } else if (driversStation == 2) {
        yPos = Meters.of(3.981);
      } else {
        yPos = Meters.of(7.255);
      }
    } else {
      initialAngle = Degrees.of(180);
      xPos = Meters.of(16.542).minus(halfLength);
      if (driversStation == 1) {
        yPos = Meters.of(2.265);
      } else if (driversStation == 2) {
        yPos = Meters.of(3.981);
      } else {
        yPos = Meters.of(7.255);
      }
    }
    return new Pose2d(new Translation2d(xPos, yPos), new Rotation2d(initialAngle));
  }

  void resetPositionFromAllianceSelection() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      alliance = Optional.of(Alliance.Blue);
    }
    var location = DriverStation.getLocation();
    if (location.isEmpty()) {
      location = OptionalInt.of(1);
    }

    m_drivebase.resetOdometry(
        computeInitialPoseForDriversStation(alliance.get(), location.getAsInt()));
    if (m_vision != null) {
      var pose = m_drivebase.getPose();
      m_vision.updateReferencePose(pose);
      m_vision.updateLastPose(pose);
      m_vision.resetSimPose(pose);
    }
  }

  public Command getAutonomousCommand() {
    switch (mode) {
      case eDoNothing:
        return new PrintCommand("*** Auto mode: doing nothing, as instructed....");
      case eSpin:
        return new SpinInPlace(m_drivebase);
      case eFollowTrajectory:
        return generateAutoModeSCurveCommand();
      default:
        return new PrintCommand("*****\n***** Error: Unhandled AutoMode setting!\n*****");
    }
  }

  private Command generateFunctionalCommandForControlSystemSampleTrajectory() {
    final Trajectory t = TrajectoryGenerator.generateTrajectory(new Pose2d(2, 2, new Rotation2d()),
        List.of(), new Pose2d(6, 4, new Rotation2d()), new TrajectoryConfig(2, 2));
    final RamseteController ramsete = new RamseteController();
    final Timer timer = new Timer();
    FunctionalCommand cmd = new FunctionalCommand(
        // init()
        () -> {
          m_drivebase.resetOdometry(t.getInitialPose());
          timer.restart();
        },
        // execute
        () -> {
          double elapsed = timer.get();
          Trajectory.State reference = t.sample(elapsed);
          ChassisSpeeds speeds = ramsete.calculate(m_drivebase.getPose(), reference);
          m_drivebase.arcadeDrive(MetersPerSecond.of(speeds.vxMetersPerSecond),
              RadiansPerSecond.of(speeds.omegaRadiansPerSecond));
        },
        // end
        (Boolean interrupted) -> {
          m_drivebase.stop();
        },
        // isFinished
        () -> {
          return false;
        },
        // requiremnets
        m_drivebase);
    return cmd;
  }

  private Command generateRamseteCommandForControlSystemSampleTrajectory() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    final DifferentialDriveVoltageConstraint voltageConstraints = new DifferentialDriveVoltageConstraint(
        m_drivebase.getMotorFeedforward(),
        m_drivebase.getKinematics(),
        /* maxVoltage= */ 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(MAX_AUTO_VELOCITY_MPS, MAX_AUTO_ACCELERATION_MPSS)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(m_drivebase.getKinematics())
    // // Apply the voltage constraint
    // // NOTE: THE FAILURE SEEMS TO BE COMING FROM IN HERE!!!!
    // .addConstraint(voltageConstraints)
    // End of constraints
    ;

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_drivebase::getPose,
        new RamseteController(2, 0.7), m_drivebase.getMotorFeedforward(),
        m_drivebase.getKinematics(), m_drivebase::getWheelSpeeds,
        new PIDController(m_drivebase.getKP(), 0, 0), new PIDController(m_drivebase.getKP(), 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivebase::setMotorVoltages, m_drivebase);

    // Reset odometry to the starting pose of the trajectory.
    m_drivebase.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivebase.stop());
  }

  private Command generateAutoModeSCurveCommand() {
    switch (m_autoModeTrajectorySelection) {
      case eControlSystemExampleFunctionalCommand:
        return generateFunctionalCommandForControlSystemSampleTrajectory();
      case eControlSystemExampleRamseteCommand:
        return generateRamseteCommandForControlSystemSampleTrajectory();
      case eTrajectoryCommandGeneratorExample:
        return m_trajectoryCommandGenerator.generateCommand(AUTO_SPEED_PROFILE,
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)), false);
      default:
        return new PrintCommand("Unexpected value for m_autoModeTrajectorySelection!");
    }
  }

  private void maybeAddClimberCommandsToDashboard() {
    if (m_climber == null) {
      return;
    }

    SmartDashboard.putData("Extend climbers", new ExtendClimbers(m_climber));
    SmartDashboard.putData("Retract climbers", new RetractClimbers(m_climber));
    SmartDashboard.putData("Safe climbers", new SetClimberSafetyMode(m_climber, true));
    SmartDashboard.putData("Unsafe climbers", new SetClimberSafetyMode(m_climber, false));
  }
}
