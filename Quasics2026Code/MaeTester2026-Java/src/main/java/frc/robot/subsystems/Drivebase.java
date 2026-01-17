// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.CANBusIds.SparkMax.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
  static final double ANDYMARK_6IN_PLACTION_DIAMETER_METERS = Units.inchesToMeters(6.0);
  static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * ANDYMARK_6IN_PLACTION_DIAMETER_METERS;
  static final double DRIVEBASE_GEAR_RATIO = 10.71;
  static final double TICKS_PER_REV_FOR_NEO_MOTOR = 42;

  // TODO: We *really* need to replace the use of a motor controller group with
  // CAN-based "leader/follower" configuration.
  final SparkMax m_leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
  final SparkMax m_leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
  final SparkMax m_rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
  final SparkMax m_rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

  final SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
  final SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
  final SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
  final SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final DifferentialDrive m_drive;

  /** Creates a new Drivebase. */
  public Drivebase() {
    rightLeaderConfig.inverted(true);
    rightFollowerConfig.inverted(true);

    leftFollowerConfig.follow(LEFT_LEADER_ID);
    rightFollowerConfig.follow(RIGHT_LEADER_ID);

    m_leftFollower.configure(
        leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollower.configure(
        rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Left Leader is effectively a no-op, no config changes, but for our sanity configure it along with the rest. 
    m_leftLeader.configure(
        leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(
        rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_leftEncoder = m_leftLeader.getEncoder();
    m_rightEncoder = m_rightLeader.getEncoder();

    m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    configureEncoders();
  }

  private void configureEncoders() {
    resetEncoders();
  }

  public void stop() {
    m_drive.tankDrive(0, 0);
  }

  public void setMotorSpeed(double leftPercent, double rightPercent) {
    m_drive.tankDrive(leftPercent, rightPercent);
  }

  public void arcadeDrive(double forwardPercent, double turnPercent) {
    m_drive.arcadeDrive(forwardPercent, turnPercent);
  }

  private double getPositionConversionFactor() {
    final double distanceScalingFactorForGearing =
        WHEEL_CIRCUMFERENCE_METERS / DRIVEBASE_GEAR_RATIO;
    return distanceScalingFactorForGearing;
  }

  private double getVelocityConversionFactor() {
    final double distanceScalingFactorForGearing =
        WHEEL_CIRCUMFERENCE_METERS / DRIVEBASE_GEAR_RATIO;
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;
    return velocityScalingFactor;
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getLeftEncoderDistanceMeters() {
    return m_leftEncoder.getPosition() * getPositionConversionFactor();
  }
  public double getRightEncoderDistanceMeters() {
    return m_rightEncoder.getPosition() * getPositionConversionFactor();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_leftEncoder.getVelocity() * getVelocityConversionFactor(),
        m_rightEncoder.getVelocity() * getVelocityConversionFactor());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLeader.setVoltage(leftVolts);
    m_rightLeader.setVoltage(rightVolts);
  }
}
