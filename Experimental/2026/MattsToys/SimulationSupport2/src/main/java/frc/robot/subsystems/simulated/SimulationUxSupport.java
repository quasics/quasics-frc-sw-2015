// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulated;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.subsystems.interfaces.IElevator;
import frc.robot.subsystems.interfaces.ISingleJointArm;

/**
 * Support for simulation user experience (UX).
 *
 * <ul>
 * <li>To show the elevator's visualization, select Network Tables ->
 * SmartDashboard -> Elevator Sim
 *
 * <li>To show the field visualization, select Network Tables -> SmartDashboard
 * -> Field
 * </ul>
 */
public class SimulationUxSupport {
  /** Name used to publish the field simulation UX to SmartDashboard. */
  private static final String FIELD_KEY = "Field";

  private static final String CLIMBER_KEY = IClimber.SUBSYSTEM_NAME + " Sim";

  /** Name used to publish the elevator simulation UX to SmartDashboard. */
  private static final String ELEVATOR_KEY = IElevator.SUBSYSTEM_NAME
      + " Sim";

  /** Color used to mark a fixed target position for the elevator's reach. */
  private static final Color8Bit FIXED_POSITION_COLOR = new Color8Bit(0, 0, 255);

  /**
   * Color used to mark the elevator's floor ("zero point", which may be
   * different from the lower bound).
   */
  private static final Color8Bit LIMIT_COLOR = new Color8Bit(255, 0, 0);

  /** Color used to render the elevator when running under manual control. */
  private static final Color8Bit NO_SETPOINT = new Color8Bit("#FFA500");

  /**
   * Color used to render the elevator when we've reached the target position.
   */
  private static final Color8Bit AT_SETPOINT = new Color8Bit("#00FF00");

  /**
   * Color used to render the elevator when we're driving towards the target
   * position.
   */
  private static final Color8Bit NOT_AT_SETPOINT = new Color8Bit("#FF0000");

  /** Color used to render the elevator when it's in the "idle" state. */
  private static final Color8Bit IDLE = new Color8Bit("#808080");

  /** Singleton instance of this class. */
  public static final SimulationUxSupport instance = new SimulationUxSupport();

  /** Length to use for the arm in the simulation UX. */
  private static final double ARM_LENGTH = 1;

  /** Root of the simulation for elevator and arm (shown in one window). */
  private final Mechanism2d m_elevatorAndArmRootMech2d;

  /** Root of the simulation for climber. */
  private final Mechanism2d m_climberRootMech2d;

  /**
   * Mechanism2d visualization of the elevator (for rendering in SmartDashboard,
   * or the simulator).
   */
  private final MechanismLigament2d m_elevatorMech2d;

  /**
   * Mechanism2d visualization of the climber (for rendering in SmartDashboard,
   * or the simulator).
   */
  private final MechanismLigament2d m_climberMech2d;

  /**
   * Mechanism2d visualization of the single-joint arm (for rendering in
   * SmartDashboard, or the simulator).
   */
  private final MechanismLigament2d m_armMech2d;

  /**
   * Field UX for showing simulated driving.
   *
   * Note that this could be moved into the SimulationUxSupport class.
   */
  final Field2d m_fieldSim = new Field2d();

  /**
   * Constructor.
   *
   * Note that we are "lazy", and won't publish the Mechanism2d or Field2d
   * unless they are actually being used/updated by the various subsystems.
   * (This helps to avoid cluttering up the SmartDashboard, and also helps to
   * highlight when we've failed to allocate a subsystem.)
   */
  private SimulationUxSupport() {
    // Simulation rendering setup.
    m_elevatorAndArmRootMech2d = new Mechanism2d(9,
        (SimElevator.getDefinedHeightForPosition(IElevator.ElevatorPosition.TOP)
            * 1.15) // Leave a little room at the top
    );
    m_elevatorMech2d = m_elevatorAndArmRootMech2d.getRoot("Root", 5, 0)
        .append(new MechanismLigament2d(IElevator.SUBSYSTEM_NAME, 0, 90));
    m_armMech2d = m_elevatorMech2d.append(new MechanismLigament2d(ISingleJointArm.SUBSYSTEM_NAME, ARM_LENGTH, 0));

    // Set up climber simulation rendering
    m_climberRootMech2d = new Mechanism2d(9,
        ((SimClimber.MAX_HEIGHT - SimClimber.MIN_HEIGHT) * 1.15) // Leave a little room at the top
    );
    m_climberMech2d = m_climberRootMech2d.getRoot("Root", 5, 0)
        .append(new MechanismLigament2d(IClimber.SUBSYSTEM_NAME, 0, 90));

    // Render climber boundary markers
    addClimberMarkers();

    // Render elevator boundary markers
    addElevatorMarkers();

    // Render arm boundary markers
    addArmMarker("ArmMin", SimArm.ARM_MIN, LIMIT_COLOR);
    addArmMarker("ArmMax", SimArm.ARM_MAX, LIMIT_COLOR);
  }

  /**
   * Renders climber "marker lines" for comparison basis (e.g., is it at position
   * X, etc.).
   */
  private void addClimberMarkers() {
    addClimberLevel("Retracted", SimClimber.getHeightForPosition(IClimber.Position.Retracted), FIXED_POSITION_COLOR);
    addClimberLevel("Extended", SimClimber.getHeightForPosition(IClimber.Position.Extended), FIXED_POSITION_COLOR);
    addClimberLevel("PulledUp", SimClimber.getHeightForPosition(IClimber.Position.PulledUp), FIXED_POSITION_COLOR);
  }

  /**
   * Renders elevator "marker lines" for comparison basis (e.g., is it at position
   * X, etc.).
   */
  private void addElevatorMarkers() {
    addElevatorLevel("Floor", 0, LIMIT_COLOR);
    addElevatorLevel("Bottom",
        SimElevator.getDefinedHeightForPosition(
            IElevator.ElevatorPosition.BOTTOM),
        FIXED_POSITION_COLOR);
    addElevatorLevel("Low",
        SimElevator.getDefinedHeightForPosition(IElevator.ElevatorPosition.LOW),
        FIXED_POSITION_COLOR);
    addElevatorLevel("Medium",
        SimElevator.getDefinedHeightForPosition(
            IElevator.ElevatorPosition.MEDIUM),
        FIXED_POSITION_COLOR);
    addElevatorLevel("High",
        SimElevator.getDefinedHeightForPosition(
            IElevator.ElevatorPosition.HIGH),
        FIXED_POSITION_COLOR);
    addElevatorLevel("Top",
        SimElevator.getDefinedHeightForPosition(IElevator.ElevatorPosition.TOP),
        FIXED_POSITION_COLOR);
  }

  /**
   * Adds a marker line for a given level of the climber.
   *
   * @param name   name for the marker
   * @param height height of the marker
   * @param color  color to use for the marker
   */
  private void addClimberLevel(String name, double height, Color8Bit color) {
    m_climberRootMech2d.getRoot(name + "Root", 0, height - SimClimber.MIN_HEIGHT)
        .append(new MechanismLigament2d(name, 10, 0, 1, color));
  }

  /**
   * Adds a marker line for a given level of the elevator.
   *
   * @param name   name for the marker
   * @param height height of the marker
   * @param color  color to use for the marker
   */
  private void addElevatorLevel(String name, double height, Color8Bit color) {
    m_elevatorAndArmRootMech2d.getRoot(name + "Root", 0, height)
        .append(new MechanismLigament2d(name, 10, 0, 1, color));
  }

  /**
   * Adds a marker line for a given angle of the arm.
   *
   * @param name  name for the marker
   * @param angle angle of the marker
   * @param color color to use for the marker
   */
  private void addArmMarker(String name, Angle angle, Color8Bit color) {
    m_elevatorMech2d.append(
        new MechanismLigament2d(name, ARM_LENGTH, angle.in(Degrees), 1, color));
  }

  /**
   * Enumeration of the status of a device (elevator or arm).
   */
  public enum DeviceStatus {
    /** Device is being controlled manually (i.e., not PID). */
    Manual,
    /** Device is at setpoint/target position (PID control). */
    AtSetpoint,
    /** Device is not at setpoint/target position (PID control). */
    NotAtSetpoint,
    /** Device is idle. */
    Idle
  }

  /**
   * Sets the color of a mechanism based on its status.
   *
   * @param mech   mechanism to set the color for
   * @param status current status of the device
   */
  private static void setMechanismColor(
      MechanismLigament2d mech, DeviceStatus status) {
    switch (status) {
      case Manual:
        mech.setColor(NO_SETPOINT);
        break;
      case Idle:
        mech.setColor(IDLE);
        break;
      case AtSetpoint:
        mech.setColor(AT_SETPOINT);
        break;
      case NotAtSetpoint:
        mech.setColor(NOT_AT_SETPOINT);
        break;
    }
  }

  /**
   * Lazily publishes a mechanism to SmartDashboard if it hasn't already been
   * published.
   *
   * @param key      key to use for SmartDashboard
   * @param sendable item to publish
   */
  private static void lazyPublishToSmartDashboard(String key, Sendable item) {
    if (!SmartDashboard.containsKey(key)) {
      SmartDashboard.putData(key, item);
    }
  }

  /**
   * Updates the rendering of the climber's position in the simulation UX.
   *
   * @param currentHeight current height of the climber
   * @param targetHeight  target height of the climber
   * @param status        current status of the climber
   */
  public void updateClimber(double currentHeight, double targetHeight, DeviceStatus status) {
    m_climberMech2d.setLength(currentHeight);
    setMechanismColor(m_climberMech2d, status);
    lazyPublishToSmartDashboard(CLIMBER_KEY, m_climberRootMech2d);
  }

  /**
   * Updates the rendering of the elevator's position in the simulation UX.
   *
   * @param currentHeight current height of the elevator
   * @param targetHeight  target height of the elevator
   * @param status        current status of the elevator
   */
  public void updateElevator(
      double currentHeight, double targetHeight, DeviceStatus status) {
    m_elevatorMech2d.setLength(currentHeight);
    setMechanismColor(m_elevatorMech2d, status);
    lazyPublishToSmartDashboard(ELEVATOR_KEY, m_elevatorAndArmRootMech2d);
  }

  /**
   * Updates the rendering of the arm's position and status.
   *
   * @param currentAngle current angle of the arm
   * @param status       current status of the arm
   */
  public void updateArm(Angle currentAngle, DeviceStatus status) {
    m_armMech2d.setAngle(currentAngle.in(Degrees));
    setMechanismColor(m_armMech2d, status);
    lazyPublishToSmartDashboard(ELEVATOR_KEY, m_elevatorAndArmRootMech2d);
  }

  /**
   * Updates the robot's actual pose on the field (based on pure simulation
   * data).
   *
   * @param robotPose current pose of the robot
   */
  public void updateFieldRobotPose(Pose2d robotPose) {
    m_fieldSim.setRobotPose(robotPose);
    lazyPublishToSmartDashboard(FIELD_KEY, m_fieldSim);
  }

  /**
   * Updates the robot's estimated pose on the field (based on some approach for
   * doing so).
   *
   * @param label     label associated with the estimated pose (e.g., "Odometry")
   * @param robotPose current estimated pose of the robot
   */
  public void updateEstimatedRobotPose(String label, Pose2d robotPose) {
    m_fieldSim.getObject(label).setPose(robotPose);
    lazyPublishToSmartDashboard(FIELD_KEY, m_fieldSim);
  }
}