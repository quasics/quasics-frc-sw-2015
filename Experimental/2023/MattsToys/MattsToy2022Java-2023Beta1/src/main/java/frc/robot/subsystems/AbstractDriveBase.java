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

/**
 * An abstract base class for drive base subsystems to be used with either the
 * Romi units or "full-sized" FRC robots.
 */
public abstract class AbstractDriveBase extends SubsystemBase {
  /**
   * Wrapper for the core functionality that this class needs from the encoders.
   * 
   * This is required, as the RelativeEncoder class provided by the Spark Max
   * controllers isn't *actually* a version of the WPILib Encoder type.
   */
  interface TrivialEncoder {
    /** Returns the distance recorded by the encoder (in meters). */
    double getPosition();

    /** Returns the current speed reported by the encoder (in meters/sec). */
    double getVelocity();

    /** Resets the encoder's distance. */
    void reset();
  }

  /** Configured tank width for the robot. */
  final private double m_tankWidth;

  /**
   * Tracks odometry for the robot. (Updated in periodic().)
   */
  final private DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new AbstractDriveBase.
   * 
   * @param robotSettings the settings to be used in configuring the drive base
   *                      (used to differentiate between the various FRC bots, as
   *                      well as with the ROMi units)
   */
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

  /** Returns an encoder wrapper for the wheels on the robot's left side. */
  protected abstract TrivialEncoder getLeftEncoder();

  /** Returns an encoder wrapper for the wheels on the robot's right side. */
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

  /** Returns a Gyro to be used in looking at the robot's heading. */
  public abstract Gyro getZAxisGyro();

  /** Returns the robot's current heading as reported by the gyro. */
  Rotation2d getGyroAngle() {
    return getZAxisGyro().getRotation2d();
  }

  //////////////////////////////////////////////////////////////////
  // Trajectory-following support.

  /**
   * Returns the position of the robot on the field.
   * 
   * Note that this class reports position using a robot-based perspective (i.e.,
   * relative to the robot's starting position/direction), rather than a
   * field-oriented perspective.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * @return the current wheel speeds for the robot.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }

  /**
   * Updates the current odometry data for the robot.
   * 
   * @see #periodic()
   */
  private void updateOdometry() {
    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    var gyroAngle = Rotation2d.fromDegrees(-getZAxisGyro().getAngle());

    // Update the pose
    m_odometry.update(gyroAngle, getLeftEncoderPosition(), getRightEncoderPosition());
  }

  /** Resets the robot's odometry data. */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getGyroAngle());
  }

  /** Returns the width between the robot's tracks. */
  public double getTrackWidth() {
    return m_tankWidth;
  }

  /**
   * Sets the robot's wheel speeds using the specified left/right voltage.
   * 
   * This is intended for use during path following/PID-controleld operations.
   */
  public abstract void tankDriveVolts(double leftVolts, double rightVolts);
}
