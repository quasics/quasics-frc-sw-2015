// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

/**
 * A simple record to hold the configuration for a flywheel subsystem. This can
 * be used to easily create different types of flywheel subsystems based on the
 * configuration provided (e.g., SparkMax, TalonFX, etc.).
 * 
 * Notes:
 * <ul>
 * <li>
 * The PID/FF constants should be determined experimentally using SysId or
 * another approach. (See discussion at the provided link for more information.)
 * 
 * <li>
 * You'll often only need kP to be set in the PID configuration, due to a
 * flywheel generally being pretty stable. If we see a persistant small gap
 * between the target RPM and what we're actually getting out of the subsystem,
 * then you can try adding a *tiny* bit of kI.)
 * </ul>
 * 
 * @see https://github.com/quasics/quasics-frc-sw-2015/wiki/Profiling-a-flywheel-to-get-PID-and-FF-constants
 * 
 */
public record FlywheelConfig(
    FlywheelType type, int motorID, boolean inverted,
    SimpleFeedForwardConfig feedForward, PIDConfig pidConfig) {
  public enum FlywheelType {
    SparkMax,
    TalonFX,
    Null
  }
}
