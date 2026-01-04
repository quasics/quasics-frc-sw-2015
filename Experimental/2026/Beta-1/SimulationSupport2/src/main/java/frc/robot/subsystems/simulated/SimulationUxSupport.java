// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulated;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.interfaces.IElevator;

/**
 * Support for simulation user experience (UX).
 *
 * <ul>
 * <li>To show the elevator's visualization, select Network Tables -> SmartDashboard -> Elevator Sim
 *
 * <li>To show the field visualization, select Network Tables -> SmartDashboard -> Field
 * </ul>
 */
public class SimulationUxSupport {
  /** Name used to publish the field simulation UX to SmartDashboard. */
  private static final String FIELD_KEY = "Field";

  /** Name used to publish the elevator simulation UX to SmartDashboard. */
  private static final String ELEVATOR_KEY = IElevator.SUBSYSTEM_NAME + " Sim";

  /** Color used to mark a fixed target position for the elevator's reach. */
  private static final Color8Bit FIXED_POSITION_COLOR = new Color8Bit(0, 0, 255);

  /**
   * Color used to mark the elevator's floor ("zero point", which may be different from the lower
   * bound).
   */
  private static final Color8Bit LIMIT_COLOR = new Color8Bit(255, 0, 0);

  /** Color used to render the elevator when running under manual control. */
  private static final Color8Bit NO_SETPOINT = new Color8Bit("#FFA500");

  /** Color used to render the elevator when we've reached the target position. */
  private static final Color8Bit AT_SETPOINT = new Color8Bit("#00FF00");

  /** Color used to render the elevator when we're driving towards the target position. */
  private static final Color8Bit NOT_AT_SETPOINT = new Color8Bit("#FF0000");

  /** Color used to render the elevator when it's in the "idle" state. */
  private static final Color8Bit IDLE = new Color8Bit("#808080");

  /** Singleton instance of this class. */
  public static final SimulationUxSupport instance = new SimulationUxSupport();

  /** Length to use for the arm in the simulation UX. */
  private static final double ARM_LENGTH = 1;

  /** Root of the simulation. */
  private final Mechanism2d m_rootMech2d;

  /**
   * Mechanism2d visualization of the elevator (for rendering in SmartDashboard, or the simulator).
   */
  private final MechanismLigament2d m_elevatorMech2d;

  /**
   * Mechanism2d visualization of the single-joint arm (for rendering in SmartDashboard,
   * or the simulator).
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
   * Note that we are "lazy", and won't publish the Mechanism2d or Field2d unless they are actually
   * being used/updated by the various subsystems.  (This helps to avoid cluttering up the
   * SmartDashboard, and also helps to highlight when we've failed to allocate a subsystem.)
   */
  private SimulationUxSupport() {
    // Simulation rendering setup.
    m_rootMech2d = new Mechanism2d(9,
        (SimElevator.getDefinedHeightForPosition(IElevator.ElevatorPosition.TOP)
            * 1.15) // Leave a little room at the top
    );
    m_elevatorMech2d =
        m_rootMech2d.getRoot("Root", 5, 0).append(new MechanismLigament2d("Elevator", 0, 90));
    m_armMech2d = m_elevatorMech2d.append(new MechanismLigament2d("Arm", ARM_LENGTH, 0));

    //
    // Elevator boundary markers. (These aren't saved because we'll never need to
    // redraw them.)
    addElevatorLevel("Floor", 0, LIMIT_COLOR);
    addElevatorLevel("Bottom",
        SimElevator.getDefinedHeightForPosition(IElevator.ElevatorPosition.BOTTOM),
        FIXED_POSITION_COLOR);
    addElevatorLevel("Low", SimElevator.getDefinedHeightForPosition(IElevator.ElevatorPosition.LOW),
        FIXED_POSITION_COLOR);
    addElevatorLevel("Medium",
        SimElevator.getDefinedHeightForPosition(IElevator.ElevatorPosition.MEDIUM),
        FIXED_POSITION_COLOR);
    addElevatorLevel("High",
        SimElevator.getDefinedHeightForPosition(IElevator.ElevatorPosition.HIGH),
        FIXED_POSITION_COLOR);
    addElevatorLevel("Top", SimElevator.getDefinedHeightForPosition(IElevator.ElevatorPosition.TOP),
        FIXED_POSITION_COLOR);

    //
    // Arm boundary markers
    //
    addArmMarker("ArmMin", SimArm.ARM_MIN, LIMIT_COLOR);
    addArmMarker("ArmMax", SimArm.ARM_MAX, LIMIT_COLOR);
  }

  /**
   * Adds a marker line for a given level of the elevator.
   *
   * @param name name for the marker
   * @param height height of the marker
   * @param color color to use for the marker
   */
  private void addElevatorLevel(String name, double height, Color8Bit color) {
    m_rootMech2d.getRoot(name + "Root", 0, height)
        .append(new MechanismLigament2d(name, 10, 0, 1, color));
  }

  /**
   * Adds a marker line for a given angle of the arm.
   *
   * @param name name for the marker
   * @param angle angle of the marker
   * @param color color to use for the marker
   */
  private void addArmMarker(String name, Angle angle, Color8Bit color) {
    m_elevatorMech2d.append(new MechanismLigament2d(name, ARM_LENGTH, angle.in(Degrees), 1, color));
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
   * Updates the rendering of the elevator's position in the simulation UX.
   *
   * @param currentHeight current height of the elevator
   * @param targetHeight target height of the elevator
   * @param status current status of the elevator
   */
  public void updateElevator(double currentHeight, double targetHeight, DeviceStatus status) {
    m_elevatorMech2d.setLength(currentHeight);

    // Update color based on state.
    switch (status) {
      case Manual:
        m_elevatorMech2d.setColor(NO_SETPOINT);
        break;
      case Idle:
        m_elevatorMech2d.setColor(IDLE);
        break;
      case AtSetpoint:
        m_elevatorMech2d.setColor(AT_SETPOINT);
        break;
      case NotAtSetpoint:
        m_elevatorMech2d.setColor(NOT_AT_SETPOINT);
        break;
    }

    if (!SmartDashboard.containsKey(ELEVATOR_KEY)) {
      // Publish the simulation of the elevator to SmartDashboard.
      SmartDashboard.putData(ELEVATOR_KEY, m_rootMech2d);
    }
  }

  /**
   * Updates the rendering of the arm's position and status.
   *
   * @param currentAngle current angle of the arm
   * @param status current status of the arm
   */
  public void updateArm(Angle currentAngle, DeviceStatus status) {
    m_armMech2d.setAngle(currentAngle.in(Degrees));

    if (status == DeviceStatus.Idle) {
      m_armMech2d.setColor(IDLE);
    } else if (status == DeviceStatus.AtSetpoint) {
      m_armMech2d.setColor(AT_SETPOINT);
    } else if (status == DeviceStatus.NotAtSetpoint) {
      m_armMech2d.setColor(NOT_AT_SETPOINT);
    }

    if (!SmartDashboard.containsKey(ELEVATOR_KEY)) {
      // Publish the simulation of the elevator to SmartDashboard.
      SmartDashboard.putData(ELEVATOR_KEY, m_rootMech2d);
    }
  }

  /**
   * Updates the robot's actual pose on the field (based on pure simulation data).
   *
   * @param robotPose current pose of the robot
   */
  public void updateFieldRobotPose(Pose2d robotPose) {
    m_fieldSim.setRobotPose(robotPose);

    if (!SmartDashboard.containsKey(FIELD_KEY)) {
      // Publish the simulated field to the smart dashboard
      SmartDashboard.putData(FIELD_KEY, m_fieldSim);
    }
  }

  /**
   * Updates the robot's estimated pose on the field (based on some approach for doing so).
   *
   * @param label label associated with the estimated pose (e.g., "Odometry")
   * @param robotPose current estimated pose of the robot
   */
  public void updateEstimatedRobotPose(String label, Pose2d robotPose) {
    m_fieldSim.getObject(label).setPose(robotPose);

    if (!SmartDashboard.containsKey(FIELD_KEY)) {
      // Publish the simulated field to the smart dashboard
      SmartDashboard.putData(FIELD_KEY, m_fieldSim);
    }
  }
}