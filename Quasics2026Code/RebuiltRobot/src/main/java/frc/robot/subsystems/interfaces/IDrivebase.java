// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.Supplier;

public interface IDrivebase {
  /**
   * Drives the robot using arcade controls. (That is, one parameter controls
   * forward/backward speed, and the other controls turning speed.)
   * 
   * The implementation is expected to combine these in a way that allows the
   * robot to do both at the same time, and to turn in place when the forward
   * speed is zero.
   * 
   * @param forwardspeed forward/backward speed (positive is forward, negative
   *                     is backward)
   * @param turnspeed    turning speed (positive is clockwise, negative is
   *                     counterclockwise)
   */
  void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed);

  /**
   * Drives the robot using "tank drive" style controls. (That is, one parameter
   * controls the speed of the left side of the drivebase, and the other controls
   * the speed of the right side.)
   * 
   * @param leftSpeed  the speed for the left side of the drivebase (positive is
   *                   forward, negative is backward)
   * @param rightSpeed the speed for the right side of the drivebase (positive is
   *                   forward, negative is backward)
   */
  void setSpeeds(LinearVelocity leftSpeed, LinearVelocity rightSpeed);

  /**
   * Drives the robot using "tank drive" style controls. (That is, one parameter
   * controls the speed of the left side of the drivebase, and the other controls
   * the speed of the right side.)
   * 
   * The parameters are expected to be in the range [-1, 1], where 1 represents
   * full forward speed, and -1 represents full backward speed. (The
   * implementation should ensure that values outside this range are handled
   * appropriately, e.g., by clamping them to the range.)
   * 
   * @param leftPercent  the percentage of full speed for the left side of the
   *                     drivebase
   * @param rightPercent the percentage of full speed for the right side of the
   *                     drivebase
   */
  void setPercent(double leftPercent, double rightPercent);

  // Used to set voltage directly to the motors (for characterization,
  // trajectory following, etc.)
  void setVoltages(Voltage leftVoltage, Voltage rightVoltage);

  double mpsToPercent(LinearVelocity speed);

  Pose2d getOdometryPose();

  Pose2d getEstimatedPose();

  void resetOdometry(Pose2d pose);

  default void stop() {
    setSpeeds(MetersPerSecond.of(0), MetersPerSecond.of(0));
  }

  void setReferencePositionSupplier(Supplier<Pose2d> supplier);
}
