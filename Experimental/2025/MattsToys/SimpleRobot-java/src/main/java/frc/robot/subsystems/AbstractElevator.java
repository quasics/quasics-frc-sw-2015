// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
 */
public abstract class AbstractElevator extends SubsystemBase {
  public static final String NAME = "Elevator";
  public static final double MAX_SAFE_HEIGHT = 2.5;
  public static final double MIN_SAFE_HEIGHT = 0.0;

  /** Supported target positions for the elevator. */
  public enum TargetPosition {
    /** No target set (manual control). */
    DontCare,
    Bottom,
    Top,
    L1,
    L2
  }

  /** Operational modes for the elevator. */
  public enum Mode {
    /** Stopped (under manual control). */
    Stopped,
    /** Extending (under manual control). */
    Extending,
    /** Retracting (under manual control). */
    Retracting,
    /** Moving to (or at) a specified target position. */
    Targeted
  }

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

  public void enableSafeMode(boolean tf) {
    m_safetyOn = tf;
  }

  public boolean isSafeModeEnabled() {
    return m_safetyOn;
  }

  public Mode getMode() {
    return m_mode;
  }

  public void stop() {
    stop_impl();
    m_mode = Mode.Stopped;
  }

  public boolean extend() {
    if (m_safetyOn && getHeight_impl() >= MAX_SAFE_HEIGHT) {
      stop();
      return false;
    }

    extend_impl();
    m_mode = Mode.Extending;
    return true;
  }

  public boolean retract() {
    if (m_safetyOn && getHeight_impl() <= 0) {
      stop();
      return false;
    }

    retract_impl();
    m_mode = Mode.Retracting;
    return true;
  }

  public abstract boolean atTargetPosition();

  @Override
  public void periodic() {
    if (m_mode == Mode.Targeted && m_target != TargetPosition.DontCare) {
      // OK, we've got a target to move towards.
      updateMotor_impl();
    } else {
      // We're under manual control.
      if (m_safetyOn) {
        // Check to see if we've exceeded our safety limits.
        final double currentHeight = getHeight_impl();
        if ((m_mode == Mode.Extending && currentHeight >= MAX_SAFE_HEIGHT)
            || (m_mode == Mode.Retracting && currentHeight <= 0)) {
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

  /** Returns the current elevator height (in meters). */
  protected abstract double getHeight_impl();

  /** Stops the actual motor on the elevator. */
  protected abstract void stop_impl();

  /** Starts (actually) extending the elevator. */
  protected abstract void extend_impl();

  /** Starts (actually) retracting the elevator. */
  protected abstract void retract_impl();
}
