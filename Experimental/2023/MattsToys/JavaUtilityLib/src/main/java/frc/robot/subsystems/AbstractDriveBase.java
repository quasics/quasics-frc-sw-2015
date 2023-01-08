// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public abstract class AbstractDriveBase extends SubsystemBase {
  /** Creates a new AbstractDriveBase. */
  public AbstractDriveBase() {
  }

  protected DifferentialDrive m_diffDrive = null;

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    if (m_diffDrive != null) {
      m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    }
  }

  public void tabjDrive(double leftSpeed, double rightSpeed) {
    if (m_diffDrive != null) {
      m_diffDrive.arcadeDrive(leftSpeed, rightSpeed);
    }
  }

  // Set up the differential drive controller
  protected final void configureDifferentialDrive(MotorController left, MotorController right) {
    m_diffDrive = new DifferentialDrive(left, right);
  }

  /**
   * This should call configureDifferentialDrive() to make sure that the base
   * class has the left/right motor controllers to work with. It's abstract to
   * *force* the subclasses to do this (rather than risk forgetting).
   * 
   * However, this method will not be automatically invoked by this library: the
   * clients should make sure that it's invoked (either by their constructors, or
   * by the RobotContainer during initialization).
   */
  public abstract void finalizeSetup();

  public abstract double getWheelPlacementDiameterMillimeters();

  public abstract void resetEncoders();

  public abstract double getLeftDistanceMillimeters();

  public abstract double getRightDistanceMillimeters();

  public final double getAverageDistanceMillimeters() {
    return (getLeftDistanceMillimeters() + getRightDistanceMillimeters()) / 2.0;
  }

  // "Inch-equivalent" methods

  public static final double MILLIMETERS_PER_INCH = 25.4;

  public final double getLeftDistanceInch() {
    return getLeftDistanceMillimeters() / MILLIMETERS_PER_INCH;
  }

  public final double getRightDistanceInch() {
    return getRightDistanceMillimeters() / MILLIMETERS_PER_INCH;
  }

  public final double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  public final double getWheelPlacementDiameterInch() {
    return getWheelPlacementDiameterMillimeters() / MILLIMETERS_PER_INCH;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
