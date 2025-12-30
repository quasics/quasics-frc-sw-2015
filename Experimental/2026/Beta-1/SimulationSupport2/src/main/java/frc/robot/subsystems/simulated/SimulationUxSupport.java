// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulated;

import edu.wpi.first.math.geometry.Pose2d;
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

  /** Color used to mark the upper boundary for the elevator's reach. */
  private static final Color8Bit UPPER_BOUND_COLOR = new Color8Bit(255, 0, 0);

  /** Color used to mark the lower boundary for the elevator's reach. */
  private static final Color8Bit LOWER_BOUND_COLOR = new Color8Bit(0, 0, 255);

  /** Color used to render the elevator when running under manual control. */
  private final static Color8Bit NO_SETPOINT = new Color8Bit("#FFA500");

  /** Color used to render the elevator when we've reached the target position. */
  private final static Color8Bit AT_SETPOINT = new Color8Bit("#00FF00");

  /** Color used to render the elevator when we're driving towards the target position. */
  private final static Color8Bit NOT_AT_SETPOINT = new Color8Bit("#FF0000");

  /** Color used to render the elevator when it's in the "idle" state. */
  private final static Color8Bit IDLE = new Color8Bit("#808080");

  /** Singleton instance of this class. */
  public static final SimulationUxSupport instance = new SimulationUxSupport();

  /**
   * Mechanism2d visualization of the elevator (for rendering in SmartDashboard,
   * or the simulator).
   */
  private final MechanismLigament2d m_elevatorMech2d;

  private final Mechanism2d rootMech2d;

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
    rootMech2d = new Mechanism2d(9,
        (SimElevator.getDefinedHeightForPosition(IElevator.ElevatorPosition.TOP)
            * 1.15) // Leave a little room at the top
    );

    m_elevatorMech2d =
        rootMech2d.getRoot("Root", 5, 0).append(new MechanismLigament2d("LeftClimber", 0, 90));

    //
    // Elevator boundary markers. (These aren't saved because we'll never need to
    // redraw them.)

    // Rendering "lower bound" for elevator
    var lowerBoundRoot = rootMech2d.getRoot(
        "FloorRoot", 0, SimElevator.getDefinedHeightForPosition(IElevator.ElevatorPosition.BOTTOM));
    lowerBoundRoot.append(new MechanismLigament2d("Floor", 10, 0, 3, LOWER_BOUND_COLOR));

    // Rendering "upper bound" for elevator
    var topBoundRoot = rootMech2d.getRoot(
        "TopRoot", 0, SimElevator.getDefinedHeightForPosition(IElevator.ElevatorPosition.HIGH));
    topBoundRoot.append(new MechanismLigament2d("Top", 10, 0, 3, UPPER_BOUND_COLOR));
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
    if (status == DeviceStatus.Manual) {
      m_elevatorMech2d.setColor(NO_SETPOINT);
    } else if (status == DeviceStatus.Idle) {
      // Idle state
      m_elevatorMech2d.setColor(IDLE);
    } else {
      // Automatic position control
      if (Math.abs(currentHeight - targetHeight) < 0.01) {
        m_elevatorMech2d.setColor(AT_SETPOINT);
      } else {
        m_elevatorMech2d.setColor(NOT_AT_SETPOINT);
      }
    }

    if (!SmartDashboard.containsKey(ELEVATOR_KEY)) {
      // Publish the simulation of the elevator to SmartDashboard.
      SmartDashboard.putData(ELEVATOR_KEY, rootMech2d);
    }
  }

  public void updateFieldRobotPose(Pose2d robotPose) {
    m_fieldSim.setRobotPose(robotPose);

    if (!SmartDashboard.containsKey(FIELD_KEY)) {
      // Publish the simulated field to the smart dashboard
      SmartDashboard.putData(FIELD_KEY, m_fieldSim);
    }
  }
}