// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.abstracts;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.interfaces.IElevator;

/**
 * Provides pretty basic control for a single-mechanism "elevator" that can be
 * raised/lowered, including a "safety mode" that is intended to prevent it from
 * being wound too far though this assumes that we'd always be starting up the
 * code with the elevator fully lowered, so that "0 in code" is "0 in reality").
 *
 * Note that this is an example of using an "abstract" class as the base for one
 * or more derived types, where the base (a) implements the general
 * functionality and (b) defines the hardware-specific pieces that the derived
 * classes then fill in.
 *
 * Possible improvements would include:
 * <ul>
 * <li>Limit switches for safety controls at top and bottom</li>
 * <li>Limit switches for external detection at L1, L2</li>
 * <li>Assuming that there were (good) hard stops on the elevator, monitoring
 * the velocity of the motor to detect when we run up against them</li>
 * </ul>
 *
 * TODO: Consider adjusting min/max safe height to reflect 2025 hardware.
 */
public abstract class AbstractElevator extends SubsystemBase implements IElevator {
  /** Current operational mode. */
  protected Mode m_mode = Mode.Stopped;

  /** Current target position for PID-based control (if any). */
  protected TargetPosition m_target = TargetPosition.DontCare;

  /** If true, safety constraints are enabled during manual control. */
  private boolean m_safetyOn = true;

  /** Creates a new abstract elevator. */
  public AbstractElevator() {
    setName(NAME);
  }

  /**
   * Turns "safe mode" on/off.
   *
   * @param tf enables "safe mode" if true
   */
  public void enableSafeMode(boolean tf) {
    m_safetyOn = tf;
  }

  /**
   * Indicates if "safe mode" is enabled.
   *
   * @return true iff "safe mode" is enabled
   */
  public boolean isSafeModeEnabled() {
    return m_safetyOn;
  }

  /**
   * Returns the elevator's current operating mode.
   *
   * @return current operating mode
   */
  public Mode getMode() {
    return m_mode;
  }

  /** Stops the elevator (including clearing any target position). */
  public void stop() {
    m_mode = Mode.Stopped;
    m_target = TargetPosition.DontCare;
    stop_impl();
  }

  /**
   * Starts extending the elevator. Will not do anything if "safe mode" is
   * enabled, and elevator is already beyond MAX_SAFE_HEIGHT.
   *
   * @return true iff we started moving the elevator (i.e., not in safe mode, or
   *         below MAX_SAFE_HEIGHT)
   */
  public boolean extend() {
    if (m_safetyOn && getHeight_impl().gte(MAX_SAFE_HEIGHT)) {
      stop();
      return false;
    }

    extend_impl();
    m_mode = Mode.Extending;
    return true;
  }

  /**
   * Starts retracting the elevator. Will not do anything if "safe mode" is
   * enabled, and elevator is already beyond MIN_SAFE_HEIGHT.
   *
   * @return true iff we started moving the elevator (i.e., not in safe mode, or
   *         above MIN_SAFE_HEIGHT)
   */
  public boolean retract() {
    if (m_safetyOn && getHeight_impl().lte(MIN_SAFE_HEIGHT)) {
      stop();
      return false;
    }

    retract_impl();
    m_mode = Mode.Retracting;
    return true;
  }

  @Override
  public void periodic() {
    if (m_mode == Mode.Targeted && m_target != TargetPosition.DontCare) {
      // OK, we've got a target to move towards.
      updateMotor_impl();
    } else {
      // We're under manual control.
      if (m_safetyOn) {
        // Check to see if we've exceeded our safety limits.
        final Distance currentHeight = getHeight_impl();
        if ((m_mode == Mode.Extending && currentHeight.gte(MAX_SAFE_HEIGHT))
            || (m_mode == Mode.Retracting && currentHeight.lte(MIN_SAFE_HEIGHT))) {
          stop();
        }
      }
    }
  }

  /**
   * Establishes a target position that we want to move towards (e.g., via PID).
   *
   * @param targetPosition the desired elevator position
   */
  public void setTargetPosition(TargetPosition targetPosition) {
    // OK: stop any manual control that's in progresss.
    stop();

    m_mode = Mode.Targeted;
    m_target = targetPosition;
  }

  public Distance getHeight() {
    return getHeight_impl();
  }

  public abstract LinearVelocity getVelocity();

  public abstract Voltage getVoltage();

  public void setMotorVoltage(Voltage volts) {
    m_mode = Mode.Profiling;
    setMotorVoltage_impl(volts);
  }

  protected abstract void setMotorVoltage_impl(Voltage volts);

  /**
   * Determines if we are at our target position.
   *
   * @return true iff the elevator is currently at the target position (or if
   *         target is "don't care")
   */
  public abstract boolean atTargetPosition();

  /**
   * Handles motor adjustments (e.g., via PID) when a target position has been
   * set.
   *
   * @see #periodic()
   * @see #setTargetPosition(TargetPosition)
   */
  protected abstract void updateMotor_impl();

  /** Resets the underlying encoder on the elevator. */
  protected abstract void resetEncoder_impl();

  /**
   * Determines the current height.
   *
   * @return the current elevator height (in meters)
   */
  protected abstract Distance getHeight_impl();

  /** Stops the actual motor on the elevator. */
  protected abstract void stop_impl();

  /** Starts (actually) extending the elevator. */
  protected abstract void extend_impl();

  /** Starts (actually) retracting the elevator. */
  protected abstract void retract_impl();

  /**
   * Calculates the voltage needed for the elevator (for PID control).
   *
   * @param setpoint    the target position of the elevator
   * @param encoder     the encoder providing the current height
   * @param pid         PID controller for the elevator
   * @param feedForward feed-forward for the elevator
   * @return the voltage to be applied to continue moving toward the setpoing
   */
  protected Voltage calculateMotorVoltage(Distance setpoint, TrivialEncoder encoder,
      PIDController pid, ElevatorFeedforward feedForward) {
    final boolean noisy = false;

    final double velocity = encoder.getVelocity().in(MetersPerSecond);
    final double pidOutput = pid.calculate(encoder.getPosition().in(Meters), setpoint.in(Meters));
    final double feedForwardOutput = feedForward.calculate(velocity);

    final double output = MathUtil.clamp(pidOutput + feedForwardOutput, -12.0, +12.0);

    if (noisy) {
      System.out.printf("PID -> pos: %.02f, set: %.02f, vel: %.02f, pidOut: %.02f, ff: %.02f, "
              + "output: %.02f, atSetpoint: %b%n",
          encoder.getPosition(), setpoint.in(Meters), velocity, pidOutput, feedForwardOutput,
          output, pid.atSetpoint());
    }

    return Volts.of(output);
  }
}
