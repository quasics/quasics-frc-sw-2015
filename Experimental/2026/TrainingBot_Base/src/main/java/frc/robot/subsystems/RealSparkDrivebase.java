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
 * based on Quasics' standard "Spark Max" drivebase configuration.
 */
public class RealSparkDrivebase extends AbstractDrivebase {
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

  // Encoders
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  /** The gyro/ALU that we're using for direction identification. */
  private final Pigeon2 m_rawGyro = new Pigeon2(PIGEON2_CAN_ID);

  /**
   * Constructor.
   */
  public RealSparkDrivebase() {
    super(new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless), new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless));

    // The motor controller variables are defined in the base class, but we need to
    // use them here to finish configuring them, and to get the encoders. Note that
    // we know that they'll be of type SparkMax, so we can use them directly as
    // such.
    SparkMax leftLeader = (SparkMax) m_leftController;
    SparkMax rightLeader = (SparkMax) m_rightController;

    m_leftEncoder = leftLeader.getEncoder();
    m_rightEncoder = rightLeader.getEncoder();

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

    leftConfig.follow(0);
    leftConfig.encoder.positionConversionFactor(distanceScalingFactorForGearing);
    leftConfig.encoder.velocityConversionFactor(velocityScalingFactor);
    leftConfig.inverted(false);

    rightConfig.follow(0);
    rightConfig.encoder.positionConversionFactor(distanceScalingFactorForGearing);
    rightConfig.encoder.velocityConversionFactor(velocityScalingFactor);
    rightConfig.inverted(true);

    leftLeader.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
