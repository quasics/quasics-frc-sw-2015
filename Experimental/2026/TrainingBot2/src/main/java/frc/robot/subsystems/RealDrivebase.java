// Copyright (c) Matthew Healy, Quasics Robotics, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.CanBusIds.PIGEON2_CAN_ID;
import static frc.robot.Constants.CanBusIds.SparkMaxIds.LEFT_LEADER_ID;
import static frc.robot.Constants.CanBusIds.SparkMaxIds.RIGHT_LEADER_ID;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;

/**
 * Defines an implementation of the drivebase subsystem that uses real hardware,
 * based on Quasics' standard drivebase configuration.
 */
public class RealDrivebase extends AbstractDrivebase {
  ////////////////////////////////////////////////////////////////////////////////
  // Common physical characteristics for Quasics' robots (and directly derived
  // values).
  static final Distance ANDYMARK_6IN_PLACTION_DIAMETER = Inches.of(6.0);
  static final Distance WHEEL_CIRCUMFERENCE = ANDYMARK_6IN_PLACTION_DIAMETER.times(Math.PI);
  static final double GEAR_RATIO = 8.45;
  static final double DISTANCE_SCALING_FACTOR_FOR_GEARING = WHEEL_CIRCUMFERENCE.div(GEAR_RATIO).in(Meters);
  static final double VELOCITY_SCALING_FACTOR = DISTANCE_SCALING_FACTOR_FOR_GEARING / 60;

  ////////////////////////////////////////////////////////////////////////////////
  // Hardware control/sensing.
  //

  // Motors (we only need to control the leaders; the followers will... well,
  // follow them).
  final SparkMax m_leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
  final SparkMax m_rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

  /** The gyro/ALU that we're using for direction identification. */
  private final Pigeon2 m_rawGyro = new Pigeon2(PIGEON2_CAN_ID);

  /**
   * Constructor.
   */
  public RealDrivebase() {
    super();

    ////////////////////////////////////////
    // Configure the encoders.
    System.out.println("Wheel circumference (m): " + WHEEL_CIRCUMFERENCE.in(Meters));

    // Conversion factor from units in rotations (or RPM) to meters (or m/s).
    final double distanceScalingFactorForGearing = WHEEL_CIRCUMFERENCE.div(GEAR_RATIO).in(Meters);
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;
    System.out.println("Using gear ratio: " + GEAR_RATIO);
    System.out.println("Adjustment for gearing (m/rotation): " + distanceScalingFactorForGearing);
    System.out.println("Velocity adj.: " + velocityScalingFactor);

    final SparkMaxConfig leftConfig = new SparkMaxConfig();
    final SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig.encoder.positionConversionFactor(distanceScalingFactorForGearing);
    leftConfig.encoder.velocityConversionFactor(velocityScalingFactor);
    rightConfig.encoder.positionConversionFactor(distanceScalingFactorForGearing);
    rightConfig.encoder.velocityConversionFactor(velocityScalingFactor);

    rightConfig.inverted(true);

    m_leftLeader.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void tankDrive(double leftPercentage, double rightPercentage) {
    m_leftLeader.set(leftPercentage);
    m_rightLeader.set(rightPercentage);
  }

  @Override
  public double getLeftDistanceMeters() {
    return m_leftEncoder.getPosition();
  }

  @Override
  public double getRightDistanceMeters() {
    return m_rightEncoder.getPosition();
  }

  @Override
  public double getHeadingInDegrees() {
    // Negate since getAngle() historically provided clockwise-positive, and Pigeon
    // is counter-clockwise-positive.
    return -m_rawGyro.getRotation2d().getDegrees();
  }
}
