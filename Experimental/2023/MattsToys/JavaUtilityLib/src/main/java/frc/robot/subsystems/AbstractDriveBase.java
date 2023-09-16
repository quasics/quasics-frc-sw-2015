// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

public abstract class AbstractDriveBase extends SubsystemBase
    implements DriveBaseWith3AxisGyroInterface {
  /** The differential drive object used for basic maneuvering. */
  protected DifferentialDrive m_diffDrive = null;

  private MotorController m_leftMotor;
  private MotorController m_rightMotor;

  /** Tracks odometry for the robot. (Updated in periodic().) */
  private DifferentialDriveOdometry m_odometry;

  protected final RobotSettings m_robotSettings;

  /** Creates a new AbstractDriveBase. */
  public AbstractDriveBase(RobotSettings robotSettings) {
    if (robotSettings == null) {
      throw new IllegalArgumentException("Null robot settings");
    }

    m_robotSettings = robotSettings;
    System.err.println("Configuring drive base using settings:\n" + robotSettings);
  }

  // Set up the differential drive controller
  protected final void configureDifferentialDrive(MotorController left, MotorController right) {
    m_leftMotor = left;
    m_rightMotor = right;
    m_diffDrive = new DifferentialDrive(left, right);

    ////////////////////////////////////////
    // Odometry setup.

    resetEncoders();

    // Notes:
    // 1) This *must* be done after the encoders have been reset to 0, or at least
    // before the first time it's updated in periodic().
    // 2) We're building a "robot-oriented" view of the field (as in, our starting
    // position will be treated as the origin), rather than a "field-oriented" view.
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0, new Pose2d());
  }

  /**
   * This should call configureDifferentialDrive() to make sure that the base class has the
   * left/right motor controllers to work with. It's abstract to *force* the subclasses to do this
   * (rather than risk forgetting).
   *
   * <p>However, this method will not be automatically invoked by this library: the clients should
   * make sure that it's invoked (either by their constructors, or by the RobotContainer during
   * initialization).
   */
  public abstract void finalizeSetup();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();

    SmartDashboard.putNumber("Left distance", getLeftDistanceMeters());
    SmartDashboard.putNumber("Right distance", getRightDistanceMeters());
    SmartDashboard.putNumber("Yaw", getGyroAngle().getDegrees());
  }

  //////////////////////////////////////////////////////////////////
  // Basic maneuvering support.

  @Override
  public final void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    if (m_diffDrive != null) {
      m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    }
  }

  @Override
  public final void tankDrive(double leftSpeed, double rightSpeed) {
    if (m_diffDrive != null) {
      m_diffDrive.tankDrive(leftSpeed, rightSpeed);
    }
  }

  @Override
  public final void stop() {
    m_diffDrive.stopMotor();
  }

  //////////////////////////////////////////////////////////////////
  // Positioning/pose information support.

  public abstract double getYawDegrees();

  /** Returns a Gyro to be used in looking at the robot's heading/yaw (rotation on Z-axis). */
  public abstract Gyro getYawGyro();

  /**
   * Returns a Gyro to be used in looking at the robot's pitch (rotation on Y-axis).
   *
   * <p>Note that this may be a NullGyro (always returning 0) if the robot's hardware doesn't
   * support this.
   */
  public abstract Gyro getPitchGyro();

  /**
   * Returns a Gyro to be used in looking at the robot's roll (rotation on X-axis).
   *
   * <p>Note that this may be a NullGyro (always returning 0) if the robot's hardware doesn't
   * support this.
   */
  public abstract Gyro getRollGyro();

  /** Returns the robot's current heading as reported by the gyro. */
  public Rotation2d getGyroAngle() {
    return getYawGyro().getRotation2d();
  }

  /** Returns an encoder wrapper for the wheels on the robot's left side. */
  protected abstract TrivialEncoder getLeftEncoder();

  /** Returns an encoder wrapper for the wheels on the robot's right side. */
  protected abstract TrivialEncoder getRightEncoder();

  /**
   * @return the current reading for the left encoder (in meters)
   */
  @Override
  public final double getLeftEncoderPositionMeters() {
    return getLeftEncoder().getPosition();
  }

  /**
   * @return the current reading for the right encoder (in meters)
   */
  @Override
  public final double getRightEncoderPositionMeters() {
    return getRightEncoder().getPosition();
  }

  /**
   * @return the current speed for the left wheels (in meters/sec)
   */
  public final double getLeftSpeed() {
    return getLeftEncoder().getVelocity();
  }

  /**
   * @return the current speed for the right wheels (in meters/sec)
   */
  public final double getRightSpeed() {
    return getRightEncoder().getVelocity();
  }

  /** Resets both the left and right encoders to 0. */
  public final void resetEncoders() {
    getRightEncoder().reset();
    getLeftEncoder().reset();
  }
  //////////////////////////////////////////////////////////////////
  // Trajectory-following support.

  /**
   * Returns the position of the robot on the field.
   *
   * <p>Note that this class reports position using a robot-based perspective (i.e., relative to the
   * robot's starting position/direction), rather than a field-oriented perspective.
   */
  public final Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * @return the current wheel speeds for the robot.
   */
  public final DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }

  /**
   * Updates the current odometry data for the robot.
   *
   * @see #periodic()
   */
  private final void updateOdometry() {
    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    var gyroAngle = Rotation2d.fromDegrees(-getYawGyro().getAngle());

    // Update the pose
    m_odometry.update(gyroAngle, getLeftEncoderPositionMeters(), getRightEncoderPositionMeters());
  }

  /** Resets the robot's odometry data. */
  public final void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(new Rotation2d(0), 0, 0, new Pose2d());
  }

  /** Returns the width between the robot's tracks. */
  public double getTrackWidth() {
    return m_robotSettings.trackWidthMeters;
  }

  /**
   * Sets the robot's wheel speeds using the specified left/right voltage.
   *
   * <p>This is intended for use during path following/PID-controleld operations.
   */
  public final void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
    m_diffDrive.feed();
  }

  //////////////////////////////////////////////////////////////////
  // TODO(mjh): Thin out these methods, which are mostly redundant.
  // At the least, I should write utilities to convert meters ->
  // inches, etc., and just use those in conjunction with the "in
  // meters" functions.

  public final double getLeftDistanceMeters() {
    return getLeftEncoderPositionMeters();
  }

  public final double getRightDistanceMeters() {
    return getRightEncoderPositionMeters();
  }

  public final double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  public final double getLeftDistanceMillimeters() {
    return getLeftEncoderPositionMeters() * 1000;
  }

  public final double getRightDistanceMillimeters() {
    return getRightEncoderPositionMeters() * 1000;
  }

  public final double getAverageDistanceMillimeters() {
    return (getLeftDistanceMillimeters() + getRightDistanceMillimeters()) / 2.0;
  }

  // "Inch-equivalent" methods

  public final double getLeftDistanceInch() {
    return Units.metersToInches(getLeftDistanceMeters());
  }

  public final double getRightDistanceInch() {
    return Units.metersToInches(getRightDistanceMeters());
  }

  public final double getAverageDistanceInch() {
    return Units.metersToInches(getAverageDistanceMeters());
  }

  //////////////////////////////////////////////////////////////////
  // Functions used to support alt. forms of the "TurnDegrees" command.
  //
  // Note that they are *only* used for that command, which was...
  // um... "borrowed" from the Romi sample code. If that command is
  // rewritten to use the gyro for control, then this can be eliminated.

  public abstract double getWheelPlacementDiameterMillimeters();

  public final double getWheelPlacementDiameterInch() {
    return Units.metersToInches(getWheelPlacementDiameterMillimeters() * 1000);
  }
}
