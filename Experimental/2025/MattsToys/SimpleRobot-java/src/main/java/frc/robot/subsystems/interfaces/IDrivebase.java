// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Basic interface for drive base functionality.
 */
public interface IDrivebase extends ISubsystem {
  /** Name for the subsystem (and base for BulletinBoard keys). */
  final String SUBSYSTEM_NAME = "Drivebase";

  /** Key used to post Pose information to BulletinBoard. */
  final String POSE_KEY = SUBSYSTEM_NAME + ".Pose";

  /** Maximum linear velocity that we'll allow/assume in our code. */
  final LinearVelocity MAX_SPEED = MetersPerSecond.of(1.25);

  /** Maximum rotational velocity for arcade drive. */
  final AngularVelocity MAX_ROTATION = RadiansPerSecond.of(.25 * Math.PI);

  /** Zero velocity. (A potentially useful constant.) */
  final LinearVelocity ZERO_MPS = MetersPerSecond.of(0.0);

  /**
   * Drive the robot using tank drive (as a percentage of MAX_SPEED).
   *
   * @param leftPercentage  The percentage of MAX_SPEED for the left side.
   * @param rightPercentage The percentage of MAX_SPEED for the right side.
   */
  default void tankDrive(double leftPercentage, double rightPercentage) {
    setSpeeds(new DifferentialDriveWheelSpeeds(
        MAX_SPEED.times(leftPercentage), MAX_SPEED.times(rightPercentage)));
  }

  /**
   * Drive the robot using arcade drive.
   *
   * @param speed    The linear velocity to drive at.
   * @param rotation The angular velocity to rotate at.
   */
  void arcadeDrive(LinearVelocity speed, AngularVelocity rotation);

  /**
   * Set the wheel speeds (positive values are forward).
   *
   * @param wheelSpeeds The wheel speeds to set.
   */
  void setSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds);

  /**
   * Utility method: straight forward/backward. (Effectively, tank drive with a
   * single speed for both sides.)
   *
   * @param percentage The percentage of MAX_SPEED to drive at.
   */
  default void setSpeed(double percentage) {
    tankDrive(percentage, percentage);
  }

  /** Utility method: stops the robot. */
  default void stop() {
    tankDrive(0, 0);
  }

  /** @return The reading from the left encoder (in meters) */
  Distance getLeftPositionMeters();

  /** @return The reading from the right encoder (in meters) */
  Distance getRightPositionMeters();

  /** @return heading of the robot (as an Angle) */
  Angle getHeading();

  /** @return heading of the robot (as a Pose2d) */
  Pose2d getPose();
}
