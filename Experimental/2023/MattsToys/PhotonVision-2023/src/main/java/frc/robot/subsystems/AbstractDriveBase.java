// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An abstract base class for drive base subsystems to be used with either the
 * Romi units or "full-sized" FRC robots.
 */
public abstract class AbstractDriveBase extends SubsystemBase {
  /**
   * Wrapper for the core functionality that this class needs from the encoders.
   * 
   * This is required, as the RelativeEncoder class provided by the Spark Max
   * controllers isn't *actually* a version of the WPILib Encoder type. So we'll
   * define an interface that will expose the basic functionality needed, and then
   * just wrap any encoder inside an instance of this type.
   */
  interface BasicEncoderInterface {
    /** Returns the distance recorded by the encoder (in meters). */
    double getPosition();

    /** Returns the current speed reported by the encoder (in meters/sec). */
    double getVelocity();

    /** Resets the encoder's distance. */
    void reset();
  }

  /**
   * Creates a new AbstractDriveBase.
   * 
   * @param robotSettings the settings to be used in configuring the drive base
   *                      (used to differentiate between the various FRC bots, as
   *                      well as with the ROMi units)
   */
  protected AbstractDriveBase() {
    setName("DriveBase");
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
  protected abstract BasicEncoderInterface getLeftEncoder();

  /** Returns an encoder wrapper for the wheels on the robot's right side. */
  protected abstract BasicEncoderInterface getRightEncoder();

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
}
