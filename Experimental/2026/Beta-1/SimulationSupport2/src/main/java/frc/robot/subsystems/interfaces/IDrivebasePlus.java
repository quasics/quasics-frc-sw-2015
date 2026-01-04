// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Interface for a drivebase subsystem with additional functionality. */
public interface IDrivebasePlus extends IDrivebase {
  //
  // Methods to return constants for the drivebase.
  //

  /** Returns the static gain for the drivebase (generally computed using the SysID tool). */
  double getKs();

  /** Returns the velocity gain for the drivebase (generally computed using the SysID tool). */
  double getKv();

  /** Returns the acceleration gain for the drivebase (generally computed using the SysID tool). */
  double getKa();

  /**
   * Returns the kP value for the drivebase to convert velocity errors (in m/s) to voltages
   * (generally computed using the SysID tool).
   */
  double getKp();

  //
  // Methods to support more advanced control of the drivebase (e.g., profiling, trajectory
  // following, etc.).
  //

  /** Returns the (odometry-based) estimate of the robot's position. */
  public Pose2d getEstimatedPose();

  /**
   * Directly sets the voltages for the drivebase. (Note: operates directly; no PID.)
   *
   * This is useful for motion profiling and other advanced control techniques.
   *
   * @param leftVoltage  voltage for the left side
   * @param rightVoltage voltage for the right side
   */
  public void tankDriveVolts(Voltage leftVoltage, Voltage rightVoltage);

  /**
   * Sets the speeds for the drivebase using chassis speeds. (Note: operates
   * directly; no PID.)
   *
   * @param speeds chassis speeds to set
   */
  void setSpeeds(ChassisSpeeds speeds);

  /**
   * Sets the speeds for the drivebase using PID control.
   *
   * @param speeds chassis speeds to set
   */
  void driveTankWithPID(ChassisSpeeds speeds);

  /** Returns the current position reported by the left encoder. */
  Distance getLeftPosition();

  /** Returns the current speed reported by the left encoder. */
  LinearVelocity getLeftVelocity();

  /**
   * Returns the current voltage reported by the left controller. (Useful for profiling the robot.)
   */
  Voltage getLeftVoltage();

  /** Returns the current position reported by the right encoder. */
  Distance getRightPosition();

  /** Returns the current speed reported by the right encoder. */
  LinearVelocity getRightVelocity();

  /**
   * Returns the current voltage reported by the right controller. (Useful for profiling the robot.)
   */
  Voltage getRightVoltage();
}
