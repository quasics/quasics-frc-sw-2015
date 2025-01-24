// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Abstract class for a drivebase. This class provides a common interface for
 * hardware-specific functionality, and builds on that to provide higher-level
 * functionality related to driving/navigation.
 */
public abstract class AbstractDrivebase extends SubsystemBase {

  /** Odometry for the robot, purely calculated from encoders/gyro. */
  final private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0,
      new Pose2d());

  /** Creates a new AbstractDrivebase. */
  public AbstractDrivebase() {
    setName("Drivebase");
  }

  /////////////////////////////////////////////////////////////////////////////
  // Abstract (hardware-specific) methods that must be implemented by
  // subclasses.

  /** Sets the speeds for the left- and right-side motors. */
  public abstract void tankDrive(double leftPercentage, double rightPercentage);

  /**
   * @return the distance that the wheels on the left have traveled since startup.
   */
  public abstract double getLeftDistanceMeters();

  /**
   * @return the distance that the wheels on the right have traveled since
   *         startup.
   */
  public abstract double getRightDistanceMeters();

  /**
   * @return the robot's current heading in degrees, relative to the direction on
   *         startup.
   */
  public abstract double getHeadingInDegrees();

  /////////////////////////////////////////////////////////////////////////////
  // Driving methods that can be implemented using subclasses' methods.

  /** Stops the drive base (setting motor speeds to 0). */
  public void stop() {
    tankDrive(0, 0);
  }

  /** @returns the robot's "pose" (for use with other WPILib functionality). */
  public Pose2d getPose2d() {
    return m_odometry.getPoseMeters();
  }

  /////////////////////////////////////////////////////////////////////////////
  // Other functions

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // Update our odometry
    final Rotation2d rotation = Rotation2d.fromDegrees(getHeadingInDegrees());
    final double leftDistanceMeters = getLeftDistanceMeters();
    final double rightDistanceMeters = getRightDistanceMeters();
    m_odometry.update(rotation, leftDistanceMeters, rightDistanceMeters);
  }
}
