// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

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
public interface IElevator extends ISubsystem {
  /** Subsystem name. */
  public static final String NAME = "Elevator";

  /** Minimum safe height. */
  public static final Distance MAX_SAFE_HEIGHT = Meters.of(2.5);
  /** Maximum safe height. */
  public static final Distance MIN_SAFE_HEIGHT = Meters.of(0.0);

  /** Supported target positions for the elevator. */
  public enum TargetPosition {
    /** No target set (manual control). */
    DontCare,
    /** Target MIN_SAFE_HEIGHT. */
    Bottom,
    /** Target MAX_SAFE_HEIGHT. */
    Top,
    /** Target L1 height. */
    L1,
    /** Target L2 height. */
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
    Targeted,
    Profiling
  }

  /**
   * Turns "safe mode" on/off.
   *
   * @param tf enables "safe mode" if true
   */
  void enableSafeMode(boolean tf);

  /**
   * Indicates if "safe mode" is enabled.
   *
   * @return true iff "safe mode" is enabled
   */
  boolean isSafeModeEnabled();

  /**
   * Returns the elevator's current operating mode.
   *
   * @return current operating mode
   */
  Mode getMode();

  /** Stops the elevator (including clearing any target position). */
  public void stop();

  /**
   * Starts extending the elevator. Will not do anything if "safe mode" is
   * enabled, and elevator is already beyond MAX_SAFE_HEIGHT.
   *
   * @return true iff we started moving the elevator (i.e., not in safe mode, or
   *         below MAX_SAFE_HEIGHT)
   */
  public boolean extend();

  /**
   * Starts retracting the elevator. Will not do anything if "safe mode" is
   * enabled, and elevator is already beyond MIN_SAFE_HEIGHT.
   *
   * @return true iff we started moving the elevator (i.e., not in safe mode, or
   *         above MIN_SAFE_HEIGHT)
   */
  public boolean retract();

  /**
   * Establishes a target position that we want to move towards (e.g., via PID).
   *
   * @param targetPosition the desired elevator position
   */
  public void setTargetPosition(TargetPosition targetPosition);

  public Distance getHeight();

  public LinearVelocity getVelocity();

  public Voltage getVoltage();

  public void setMotorVoltage(Voltage volts);

  /**
   * Determines if we are at our target position.
   *
   * @return true iff the elevator is currently at the target position (or if
   *         target is "don't care")
   */
  public boolean atTargetPosition();

  /**
   * Trivial implementation of an AbstractElevator. (Does nothing, but does it
   * well.... :-)
   */
  static public class NullElevator implements IElevator {
    /** Constructor. */
    public NullElevator() {
      System.out.println("INFO: Allocating NullElevator");
    }

    @Override
    public boolean atTargetPosition() {
      return true;
    }

    @Override
    public LinearVelocity getVelocity() {
      return MetersPerSecond.of(0);
    }

    @Override
    public Voltage getVoltage() {
      return Volts.of(0);
    }

    @Override
    public void enableSafeMode(boolean tf) {
      // No-op
    }

    @Override
    public boolean isSafeModeEnabled() {
      return true;
    }

    @Override
    public Mode getMode() {
      return Mode.Stopped;
    }

    @Override
    public void stop() {
      // No-op
    }

    @Override
    public boolean extend() {
      return false;
    }

    @Override
    public boolean retract() {
      return false;
    }

    @Override
    public void setTargetPosition(TargetPosition targetPosition) {
      // No-op
    }

    @Override
    public Distance getHeight() {
      return Meters.of(0);
    }

    @Override
    public void setMotorVoltage(Voltage volts) {
      // No-op
    }
  }
}
