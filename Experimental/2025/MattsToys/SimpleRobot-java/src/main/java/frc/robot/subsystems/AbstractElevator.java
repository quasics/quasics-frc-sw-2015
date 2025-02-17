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
 * <li>Assuming that there were (good) hard stops on the elevator, monitoring
 * the
 * velocity of the motor to detect when we run up against them</li>
 * <li>Supporting "known positions" for the elevator, to allow client code to
 * drive to a predefined height</li>
 * </ul>
 */
public abstract class AbstractElevator extends SubsystemBase {
  public static final String NAME = "Elevator";
  public static final double MAX_SAFE_HEIGHT = 2.5;
  public static final double MIN_SAFE_HEIGHT = 0.0;

  public enum TargetPosition {
    kDontCare,
    kBottom,
    kTop,
    kL1,
    kL2
  }

  public enum Mode {
    Stopped, Extending, Retracting
  }

  protected Mode m_mode = Mode.Stopped;

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
    if (m_safetyOn && getRevolutions_impl() >= MAX_SAFE_HEIGHT) {
      stop();
      return false;
    }

    extend_impl();
    m_mode = Mode.Extending;
    return true;
  }

  public boolean retract() {
    if (m_safetyOn && getRevolutions_impl() <= 0) {
      stop();
      return false;
    }

    retract_impl();
    m_mode = Mode.Retracting;
    return true;
  }

  @Override
  public void periodic() {
    if (m_safetyOn) {
      final double revolutions = getRevolutions_impl();
      if ((m_mode == Mode.Extending && revolutions >= MAX_SAFE_HEIGHT)
          || (m_mode == Mode.Retracting && revolutions <= 0)) {
        stop();
      }
    }
  }

  protected abstract void resetEncoder_impl();

  protected abstract double getRevolutions_impl();

  protected abstract void stop_impl();

  protected abstract void extend_impl();

  protected abstract void retract_impl();

  public abstract void setTargetPosition(TargetPosition targetPosition);
}
