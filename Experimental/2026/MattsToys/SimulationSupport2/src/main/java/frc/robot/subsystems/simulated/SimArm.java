// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulated;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ISingleJointArm;
import java.io.IOException;

/**
 * Simulated arm subsystem.
 *
 * Notes:
 * <ul>
 * <li>Arm angles are measured from the vertical.
 * <li>This is a *pure* simulation (all counting, no motor controllers, etc.).
 * </ul>
 */
public class SimArm extends SubsystemBase implements ISingleJointArm {
  /** Whether to use PID control or simple proportional control. */
  static final boolean USE_PID = true;
  /** Speed to use for non-PID control. */
  static final double NON_PID_SPEED = 0.05;
  /** Tolerance for considering the arm "at" the setpoint. */
  static final double SETPOINT_TOLERANCE = 0.01;

  /** Minimum limit on arm movement. */
  final public static Angle ARM_MAX = Degrees.of(100);
  /** Minimum limit on arm movement. */
  final public static Angle ARM_MIN = Degrees.of(-10);
  /** Arm position when fully extended from the robot. */
  final public static Angle ARM_OUT = Degrees.of(90);
  /** Arm position when fully raised. */
  final public static Angle ARM_UP = Degrees.of(0);

  /** Current control state for the arm. */
  private State m_state = State.IDLE;

  /** Target angle for the arm, if set. */
  private Angle m_targetAngle;

  /** Current angle of the arm. */
  private Angle m_currentAngle;

  /** PID controller for automatic positioning. */
  private PIDController m_pidController = new PIDController(1, 0, 0);

  /** Constructor. */
  public SimArm() {
    setName(SUBSYSTEM_NAME);
    m_currentAngle = Degrees.of(45);
  }

  /** Update the simulated display elements. */
  private void updateSimulatedDisplay() {
    SimulationUxSupport.DeviceStatus status = switch (m_state) {
      case IDLE -> SimulationUxSupport.DeviceStatus.Idle;
      case MOVING_TO_POSITION ->
        (Math.abs(m_currentAngle.minus(m_targetAngle).in(Degrees)) < SETPOINT_TOLERANCE)
            ? SimulationUxSupport.DeviceStatus.AtSetpoint
            : SimulationUxSupport.DeviceStatus.NotAtSetpoint;
    };

    // Update any simulated display elements here, if needed
    SimulationUxSupport.instance.updateArm(m_currentAngle, status);
  }

  //
  // SubsystemBase methods
  //

  @Override
  public void simulationPeriodic() {
    if (m_state == State.IDLE) {
      // Nothing to do.
    } else if (m_state == State.MOVING_TO_POSITION) {
      if (USE_PID) {
        // Simple PID control to move to target angle
        double output =
            m_pidController.calculate(m_currentAngle.in(Degrees), m_targetAngle.in(Degrees));
        m_currentAngle =
            m_currentAngle.plus(Degrees.of(output * 0.02)); // Simulate movement over 20ms
      } else {
        final double errorDegrees = m_targetAngle.minus(m_currentAngle).in(Degrees);
        final double sign = Math.signum(errorDegrees);
        double delta = NON_PID_SPEED * sign;
        if (Math.abs(errorDegrees) < Math.abs(delta)) {
          delta = errorDegrees;
        }
        m_currentAngle = m_currentAngle.plus(Degrees.of(delta)); // Simulate movement over 20ms
      }
    }

    updateSimulatedDisplay();
  }

  //
  // ISingleArm methods
  //

  @Override
  public Angle getCurrentAngle() {
    return m_currentAngle;
  }

  @Override
  public Angle getArmMinAngle() {
    return Degrees.of(-5);
  }

  @Override
  public Angle getArmMaxAngle() {
    return Degrees.of(110);
  }

  @Override
  public Angle getArmOutAngle() {
    return ARM_OUT;
  }

  @Override
  public Angle getArmUpAngle() {
    return ARM_UP;
  }

  @Override
  public void stop() {
    m_state = State.IDLE;
    m_targetAngle = null;
  }

  @Override
  public void setTargetPosition(Angle targetPosition) {
    if (targetPosition == null) {
      stop();
      return;
    }
    m_state = State.MOVING_TO_POSITION;
    m_targetAngle = targetPosition;
  }

  @Override
  public boolean atTargetPosition() {
    // Consider the arm at the target if IDLE or within tolerance
    return m_state == State.IDLE
        || Math.abs(m_currentAngle.minus(m_targetAngle).in(Degrees)) < SETPOINT_TOLERANCE;
  }

  @Override
  public State getState() {
    return m_state;
  }

  @Override
  public void close() throws IOException {
    // No-op: this class has no resources to release/close.
  }
}
