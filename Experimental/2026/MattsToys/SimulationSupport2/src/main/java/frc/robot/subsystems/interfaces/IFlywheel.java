// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

/**
 * An interface for a flywheel subsystem. This is meant to be a simple
 * abstraction that can be implemented by different types of flywheel
 * subsystems (e.g., SparkMax, TalonFX, etc.) without exposing the underlying
 * hardware details.
 */
public interface IFlywheel {
  /** Sets the target RPM for the flywheel. */
  public void setRPM(double targetRPM);

  /** Returns the current RPM of the flywheel. */
  public double getCurrentRPM();

  /** Returns the setpoint RPM of the flywheel. */
  public double getSetpointRPM();

  /** Stops the flywheel by setting the target RPM to 0. */
  default void stop() {
    setRPM(0);
  }

  /**
   * A null implementation of IFlywheel that does nothing. This can be used for
   * testing or as a placeholder (e.g., when the flywheel subsystem is not
   * supported on a particular robot configuration).
   */
  public class NullFlywheel implements IFlywheel {
    @Override
    public void setRPM(double targetRPM) {
      // Do nothing
    }

    @Override
    public double getCurrentRPM() {
      return 0;
    }

    @Override
    public double getSetpointRPM() {
      return 0;
    }
  }
}
