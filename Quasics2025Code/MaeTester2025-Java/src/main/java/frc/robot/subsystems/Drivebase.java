// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.CANBusIds.SparkMax.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {

  static final double ANDYMARK_6IN_PLACTION_DIAMETER_METERS =
      Units.inchesToMeters(6.0);
  static final double WHEEL_CIRCUMFERENCE_METERS =
      Math.PI * ANDYMARK_6IN_PLACTION_DIAMETER_METERS;
  static final double DRIVEBASE_GEAR_RATIO = 10.71;
  static final double TICKS_PER_REV_FOR_NEO_MOTOR = 42;

  // TODO: We *really* need to replace the use of a motor controller group with
  // CAN-based "leader/follower" configuration.
  private final MotorControllerGroup m_leftMotors;
  private final MotorControllerGroup m_rightMotors;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final DifferentialDrive m_drive;

  /** Creates a new Drivebase. */
  public Drivebase() {
    SparkMax leftFront = new SparkMax(LEFT_FRONT, MotorType.kBrushless),
                leftRear = new SparkMax(LEFT_REAR, MotorType.kBrushless),
                rightFront = new SparkMax(RIGHT_FRONT, MotorType.kBrushless),
                rightRear = new SparkMax(RIGHT_REAR, MotorType.kBrushless);
    rightFront.setInverted(true);
    rightRear.setInverted(true);
    leftFront.setInverted(false);
    leftRear.setInverted(false);

    m_leftEncoder = leftFront.getEncoder();
    m_rightEncoder = rightFront.getEncoder();

    m_leftMotors = new MotorControllerGroup(leftFront, leftRear);
    m_rightMotors = new MotorControllerGroup(rightFront, rightRear);
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    configureEncoders();
  }

  private void configureEncoders() {
    // Default for the encoders is to report distance and velocity in
    // revolutions and rev/minute; we want that in meters and meters/sec.
    final double distanceScalingFactorForGearing =
        WHEEL_CIRCUMFERENCE_METERS / DRIVEBASE_GEAR_RATIO;
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;

    m_leftEncoder.setPositionConversionFactor(distanceScalingFactorForGearing);
    m_rightEncoder.setPositionConversionFactor(distanceScalingFactorForGearing);

    m_leftEncoder.setVelocityConversionFactor(velocityScalingFactor);
    m_rightEncoder.setVelocityConversionFactor(velocityScalingFactor);

    resetEncoders();
  }

  public void stop() { m_drive.tankDrive(0, 0); }

  public void setMotorSpeed(double leftPercent, double rightPercent) {
    m_drive.tankDrive(leftPercent, rightPercent);
  }

  public void arcadeDrive(double forwardPercent, double turnPercent) {
    m_drive.arcadeDrive(forwardPercent, turnPercent);
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getLeftEncoderDistanceMeters() {
    return m_leftEncoder.getPosition();
  }
  public double getRightEncoderDistanceMeters() {
    return m_rightEncoder.getPosition();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(),
                                            m_rightEncoder.getVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
  }
}
