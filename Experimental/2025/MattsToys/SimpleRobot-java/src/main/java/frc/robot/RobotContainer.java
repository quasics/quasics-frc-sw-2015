// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.LogitechDualshock;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmWaveCommand;
import frc.robot.commands.MoveArmToAngle;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.RainbowLighting;
import frc.robot.subsystems.AbstractElevator;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.ILighting;
import frc.robot.subsystems.interfaces.ISingleJointArm;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.subsystems.live.Drivebase;
import frc.robot.subsystems.simulations.SimDrivebase;
import frc.robot.subsystems.simulations.SimulatedElevator;
import frc.robot.subsystems.simulations.SimulatedSingleJointArm;
import frc.robot.subsystems.simulations.SimulatedVision;
import frc.robot.utils.DeadbandEnforcer;
import frc.robot.utils.RobotConfigs;
import frc.robot.utils.RobotConfigs.RobotConfig;
import frc.robot.utils.SysIdGenerator;
import java.util.function.Supplier;

public class RobotContainer {
  static final boolean CHOREO_SHOULD_HANDLE_PATH_FLIPPING = false;

  enum AutoModeOperation {
    eDoNothing,
    eChoreo,
    eMoveAndRaise
  }

  static final AutoModeOperation AUTO_MODE_OPTION = AutoModeOperation.eChoreo;

  /** Indicates the robot we are going to target. */
  final RobotConfigs.Robot DEPLOYED_ON = RobotConfigs.Robot.Simulation;

  /** Configuration data for the targeted robot. */
  final RobotConfig m_robotConfig = RobotConfigs.getConfig(DEPLOYED_ON);

  // Subsystems
  final IVision m_vision = allocateVision(m_robotConfig);
  private final IDrivebase m_drivebase = allocateDrivebase(m_robotConfig);
  final AbstractElevator m_elevator = allocateElevator(m_robotConfig);
  final ISingleJointArm m_arm = new SimulatedSingleJointArm();
  final ILighting m_lighting = allocateLighting(m_robotConfig);

  // Controllers
  //
  // TODO: Consider using CommandJoystick class instead of Joystick. (Would let me
  // explicitly bind specific channels for X/Y, possibly simplifying live vs
  // simulation handling, as well as directly providing "trigger factories" for
  // commands.)
  //
  // Note that live joysticks generally follow a different orientation/coordinate
  // system than the one used for the robot.
  //
  // @see
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
  private final Joystick m_driveController = new Joystick(Constants.DriveTeam.DRIVER_JOYSTICK_ID);

  /** Factory object for Choreo trajectories. */
  private final AutoFactory m_autoFactory = new AutoFactory(
      m_drivebase::getPose,
      m_drivebase::resetPose,
      m_drivebase::followTrajectory,
      CHOREO_SHOULD_HANDLE_PATH_FLIPPING, // If alliance flipping should be enabled
      m_drivebase.asSubsystem());

  public RobotContainer() {
    configureArcadeDrive();
    configureSmartDashboard();
    configureBindings();

    m_lighting.setDefaultCommand(new RainbowLighting(m_lighting));
  }

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

  private void configureSmartDashboard() {
    addSysIdControlsToDashboard();

    SmartDashboard.putData("Wave arm", new ArmWaveCommand(m_arm));
    SmartDashboard.putData("Arm out", new MoveArmToAngle(m_arm, ISingleJointArm.ARM_OUT_ANGLE));
    SmartDashboard.putData("Arm up", new MoveArmToAngle(m_arm, ISingleJointArm.ARM_UP_ANGLE));
    SmartDashboard.putData("Raise elevator (wait)",
        new MoveElevatorToPosition(m_elevator, AbstractElevator.TargetPosition.Top, true));
    SmartDashboard.putData("Raise elevator (nowait)",
        new MoveElevatorToPosition(m_elevator, AbstractElevator.TargetPosition.Top, false));

    // Trajectory commands
    SmartDashboard.putData("Demo path", generateCommandForChoreoTrajectory("Demo path"));
  }

  private void configureArcadeDrive() {
    final DeadbandEnforcer deadbandEnforcer = new DeadbandEnforcer(Constants.DriveTeam.DRIVER_DEADBAND);
    Supplier<Double> forwardSupplier;
    Supplier<Double> rotationSupplier;

    if (Robot.isReal()) {
      // Configure the real robot.
      //
      // Note that we're inverting the values because Xbox controllers return
      // negative values when we push forward.
      forwardSupplier = () -> -deadbandEnforcer.limit(m_driveController.getRawAxis(LogitechDualshock.LeftYAxis));
      rotationSupplier = () -> -deadbandEnforcer.limit(m_driveController.getRawAxis(LogitechDualshock.RightXAxis));
    } else {
      // Configure the simulated robot
      //
      // Note that we're assuming a keyboard-based controller is actually being
      // used in the simulation environment (for now), and thus we want to use
      // axis 0&1 (from the "Keyboard 0" configuration).
      forwardSupplier = () -> deadbandEnforcer.limit(m_driveController.getRawAxis(0));
      rotationSupplier = () -> -deadbandEnforcer.limit(m_driveController.getRawAxis(1));
    }

    m_drivebase.asSubsystem().setDefaultCommand(
        new ArcadeDrive(m_drivebase, forwardSupplier, rotationSupplier));
  }

  private void configureBindings() {
    // TODO: Configure any bindings as required.
  }

  ////////////////////////////////////////////////////////////////////////////////
  //
  // Factory methods for commands to be executed by the rbot.
  //
  ////////////////////////////////////////////////////////////////////////////////

  private Command generateCommandForChoreoTrajectory(String trajectoryName) {
    return generateCommandForChoreoTrajectory(trajectoryName, true);
  }

  /**
   * @see <a href="https://choreo.autos/choreolib/getting-started/">Choreo
   *      'Getting Started'</a>
   * @see <a href="https://choreo.autos/choreolib/auto-factory/">AutoFactory</a>
   */
  private Command generateCommandForChoreoTrajectory(String trajectoryName, boolean resetOdometry) {
    return Commands.sequence(
        // Per https://choreo.autos/choreolib/auto-factory/
        (resetOdometry ? m_autoFactory.resetOdometry(trajectoryName) : Commands.none()),
        // Then do the thing
        m_autoFactory.trajectoryCmd(trajectoryName));
  }

  /**
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

  private Command generateChoreoAutoCommand() {
    var allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) {
      return Commands.none();
    }

    // Simple matrix of choices: we know how to get to precisely 1 algae from each
    // of the red starting points (and nowhere from blue), and we'll assume that
    // our position on the field is directly mapping to our driver station location.

    switch (allianceOpt.get()) {
      case Blue:
        System.out.println("WARNING: Don't know how to handle Blue with Choreo paths.");
        return Commands.none();

      case Red:
        var positionOpt = DriverStation.getLocation();
        if (positionOpt.isEmpty()) {
          System.out.println("WARNING: Can't get position!");
          return Commands.none();
        }

        System.out.println("INFO: OK, we're Red-" + positionOpt.getAsInt());
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

  public Command getAutonomousCommand() {
    return switch (AUTO_MODE_OPTION) {
      case eDoNothing -> Commands.print("No autonomous command configured");
      case eChoreo -> generateChoreoAutoCommand();
      case eMoveAndRaise -> new ParallelCommandGroup(
          new frc.robot.commands.DriveForDistance(m_drivebase, .50, Units.Meters.of(3)),
          new frc.robot.commands.MoveElevatorToExtreme(m_elevator, true));
    };
  }

  ////////////////////////////////////////////////////////////////////////////////
  //
  // Factory methods for our subsystems
  //
  ////////////////////////////////////////////////////////////////////////////////

  private static AbstractElevator allocateElevator(RobotConfigs.RobotConfig config) {
    if (!config.hasElevator()) {
      return new AbstractElevator.NullElevator();
    }

    if (Robot.isReal()) {
      // TODO: Implement "real elevator" support.
      return new AbstractElevator.NullElevator();
    } else {
      return new SimulatedElevator(config);
    }
  }

  private static IVision allocateVision(RobotConfigs.RobotConfig config) {
    if (!config.hasCamera()) {
      return new IVision.NullVision();
    }

    if (Robot.isReal()) {
      return new Vision(config);
    } else {
      return new SimulatedVision(config);
    }
  }

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

  private static ILighting allocateLighting(RobotConfigs.RobotConfig config) {
    if (!config.hasLighting()) {
      return new ILighting.NullLighting();
    }

    return new Lighting(config);
  }
}
