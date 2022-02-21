// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotSettings;

public abstract class AbstractDriveBase extends SubsystemBase {
  interface TrivialEncoder {
    double getPosition();

    double getVelocity();

    void reset();
  }

  /** Configured tank width for the robot. */
  final private double m_tankWidth;

  /**
   * Tracks odometry for the robot. (Updated in periodic().)
   */
  final private DifferentialDriveOdometry m_odometry;

  /** Creates a new AbstractDriveBase. */
  protected AbstractDriveBase(RobotSettings robotSettings) {
    setName("DriveBase");

    m_tankWidth = robotSettings.trackWidthMeters;

    ////////////////////////////////////////
    // Odometry setup.

    // Notes:
    // 1) This *must* be done after the encoders have been reset to 0, or at least
    // before the first time it's updated in periodic().
    // 2) We're building a "robot-oriented" view of the field (as in, our starting
    // position will be treated as the origin), rather than a "field-oriented" view.
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(0), new Pose2d());
  }

  /**
   * Stops the drive base.
   */
  public void stop() {
    tankDrive(0, 0);
  }

  /**
   * Sets the power for the left and right side of the drive base.
   *
   * @param leftPercent  % power to apply to left side (-1.0 to +1.0)
   * @param rightPercent % power to apply to right side (-1.0 to +1.0)
   */
  public void tankDrive(double leftPercent, double rightPercent) {
    var boundedLeft = Math.max(-1.0, Math.min(1.0, leftPercent));
    var boundedRight = Math.max(-1.0, Math.min(1.0, rightPercent));
    doTankDrive(boundedLeft, boundedRight);
  }

  /**
   * Arcade drive support. The calculated values will be squared to decrease
   * sensitivity at low speeds.
   *
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward
   *                  is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
   *                  Clockwise is positive.
   */
  public void arcadeDrive(double xSpeed, double zRotation) {
    doArcadeDrive(xSpeed, zRotation, true);
  }

  /**
   * Arcade drive support.
   *
   * @param xSpeed       The robot's speed along the X axis [-1.0..1.0]. Forward
   *                     is positive.
   * @param zRotation    The robot's rotation rate around the Z axis [-1.0..1.0].
   *                     Clockwise is positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    doArcadeDrive(xSpeed, zRotation, squareInputs);
  }

  /**
   * Sets the power for the left and right side of the drive base.
   *
   * @param leftPercent  % power to apply to left side (-1.0 to +1.0)
   * @param rightPercent % power to apply to right side (-1.0 to +1.0)
   */
  protected abstract void doTankDrive(double leftPercent, double rightPercent);

  /**
   * Arcade drive support.
   *
   * @param xSpeed       The robot's speed along the X axis [-1.0..1.0]. Forward
   *                     is positive.
   * @param zRotation    The robot's rotation rate around the Z axis [-1.0..1.0].
   *                     Clockwise is positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  protected abstract void doArcadeDrive(double xSpeed, double zRotation, boolean squareInputs);

  protected abstract TrivialEncoder getLeftEncoder();

  protected abstract TrivialEncoder getRightEncoder();

  /**
   * @return the current reading for the left encoder (in meters)
   */
  public double getLeftEncoderPosition() {
    return getLeftEncoder().getPosition();
  }

  /**
   * @return the current reading for the right encoder (in meters)
   */
  public double getRightEncoderPosition() {
    return getRightEncoder().getPosition();
  }

  /**
   * @return the current speed for the left wheels (in meters/sec)
   */
  public double getLeftSpeed() {
    return getLeftEncoder().getVelocity();
  }

  /**
   * @return the current speed for the right wheels (in meters/sec)
   */
  public double getRightSpeed() {
    return getRightEncoder().getVelocity();
  }

  /**
   * Resets both the left and right encoders to 0.
   */
  public void resetEncoders() {
    getRightEncoder().reset();
    getLeftEncoder().reset();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    updateOdometry();
  }

  public abstract Gyro getZAxisGyro();

  Rotation2d getGyroAngle() {
    return getZAxisGyro().getRotation2d();
  }

  //////////////////////////////////////////////////////////////////
  // Trajectory-following support.

  public Pose2d GetPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }

  private void updateOdometry() {
    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    var gyroAngle = Rotation2d.fromDegrees(-getZAxisGyro().getAngle());

    // Update the pose
    m_odometry.update(gyroAngle, getLeftEncoderPosition(), getRightEncoderPosition());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getGyroAngle());
  }

  public double getTrackWidth() {
    return m_tankWidth;
  }

  public abstract void tankDriveVolts(double leftVolts, double rightVolts);
}
