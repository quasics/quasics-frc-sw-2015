// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Defines a pretty simple interface to a drive base, providing only simple
 * driving support (i.e., no sensors, PID controls, etc.).
 */
public interface ITrivialDrivebase extends ISubsystem {
  /** Name for the subsystem (and base for BulletinBoard keys). */
  final String SUBSYSTEM_NAME = "Drivebase";

  /** Maximum linear velocity that we'll allow/assume in our code. */
  final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.5);

  /** Maximum rotational velocity for arcade drive. */
  final AngularVelocity MAX_ROTATION = DegreesPerSecond.of(180);

  /** Zero linear velocity. (A potentially useful constant.) */
  final LinearVelocity ZERO_MPS = MetersPerSecond.of(0.0);

  /** Zero rotational velocity. (A potentially useful constant.) */
  final AngularVelocity ZERO_TURNING = RadiansPerSecond.of(0.0);

  /////////////////////////////////////////////////////////////////////////////////
  //
  // "Purely abstract methods", outlining pretty basic functionality for a drive
  // base.
  //
  /////////////////////////////////////////////////////////////////////////////////

  /**
   * Drive the robot using tank drive.
   *
   * @param leftPercentage  The percentage of maximum speed for the left side
   *                        (positive is forward).
   * @param rightPercentage The percentage of maximum speed for the right side
   *                        (positive is forward).
   */
  void tankDrive(double leftPercentage, double rightPercentage);

  /**
   * Drivates the robot using arcade drive.
   * 
   * @param speedPercentage    The percentage of maximum speed forward/backward
   *                           (positive is forward).
   * @param rotationPercentage The percentage of maximum rotation speed.
   */
  void arcadeDrive(double speedPercentage, double rotationPercentage);

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Convenience methods
  //
  /////////////////////////////////////////////////////////////////////////////////

  /** Stops the robot. */
  default void stop() {
    tankDrive(0, 0);
  }

  /**
   * Drive straight forward/backward. (Effectively, tank drive with a single
   * speed for both sides.)
   *
   * @param percentage The percentage of maximum speed to drive at (positive is
   *                   forward).
   *
   * @see #tankDrive(double, double)
   */
  default void tankDrive(double percentage) {
    tankDrive(percentage, percentage);
  }

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Trivial implementation
  //
  /////////////////////////////////////////////////////////////////////////////////

  /** Null implementation of the ITrivialDrivebase interface. */
  public static class NullDrivebase implements ITrivialDrivebase {
    /** Constructor. */
    public NullDrivebase() {
      System.out.println("INFO: Allocating null drivebase");
    }

    @Override
    public void tankDrive(double leftPercentage, double rightPercentage) {
      // No-op
    }

    @Override
    public void arcadeDrive(double speedPercentage, double rotationPercentage) {
      // No-op
    }
  }
}
