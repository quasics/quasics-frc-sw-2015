// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/** Add your docs here. */
public interface DriveBaseInterface {
  public void tankDrive(double leftSpeed, double rightSpeed);

  public void arcadeDrive(double forwardSpeed, double turnSpeed);

  public void stop();

  /** Returns a Gyro to be used in looking at the robot's heading. */
  public Gyro getYawGyro();

  /** Returns the current yaw angle (in degrees). */
  public double getYawDegrees();

  /**
   * @return the current reading for the left encoder (in meters)
   */
  public double getLeftEncoderPositionMeters();

  /**
   * @return the current reading for the right encoder (in meters)
   */
  public double getRightEncoderPositionMeters();

  /**
   * @return the current speed for the left wheels (in meters/sec)
   */
  public double getLeftSpeed();

  /**
   * @return the current speed for the right wheels (in meters/sec)
   */
  public double getRightSpeed();

  /** Resets both the left and right encoders to 0. */
  public void resetEncoders();
}
