// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.OffsetGyro;
import frc.robot.sensors.SparkMaxEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;

/**
 * Drive base subsystem for actual FRC hardware, using the motor configuration
 * that Quasics has employed for the last few years (Spark MAX motors at known
 * CAN addresses).
 */
public class RealDrivebase extends AbstractDrivebase {
  /**
   * Enum class used to represent the physical characteristics (e.g., PID/motor
   * gain values, track
   * width, etc.) that are specific to a given robot.
   *
   * For example, Sally is just a drive base and is thus much lighter than the
   * robots we generally
   * put on the field, which means that the kS/kV values for her tend to be
   * smaller.
   */
  public enum RobotCharacteristics {
    // Characteristics from 2023 "ChargedUp" constants for Sally
    Sally(
        /* Track Width (m) */
        0.381 * 2, // TODO: Confirm this
        /* Gear ratio */
        8.45,
        /* PID */
        0.29613, 0.0, 0.0,
        /* Gains */
        0.19529, 2.2329, 0.0),
    // Characteristics from 2023 "ChargedUp" code constants for Gladys
    Gladys(
        /* Track Width (m) */
        0.5588 /* 22in */, // TODO: Confirm this
        /* Gear ratio */
        8.45,
        /* PID */
        0.29613, 0, 0,
        /* Gains */
        0.25829, 4.5623, 1.608),
    // Characteristics from 2023 "ChargedUp" code constants for Mae
    Mae(
        /* Track Width (m) */
        0.5588 /* 22in */, // TODO: Confirm this
        /* Gear ratio */
        8.45,
        /* PID */
        0.001379, 0, 0, // TODO: Confirm this, since it seems *really* low
        /*
         * Gains
         *
         * TODO: Confirm these, since they're very different from 2022 values. (Though
         * we also
         * changed the hardware significantly post-season.)
         */
        0.13895, 1.3143, 0.1935);

    RobotCharacteristics(double trackWidthMeters, double gearRatio, double kP, double kI, double kD,
        double kS, double kV, double kA) {
      this.kTrackWidthMeters = trackWidthMeters;
      this.gearRatio = gearRatio;
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
    }

    final double kTrackWidthMeters;
    final double gearRatio;
    final double kP;
    final double kI;
    final double kD;
    final double kS;
    final double kV;
    final double kA;
  }

  // Motor IDs are based on those Quasics has used over the last couple of years.
  static final int LEFT_FRONT_CAN_ID = 1;
  static final int LEFT_REAR_CAN_ID = 2;
  static final int RIGHT_FRONT_CAN_ID = 3;
  static final int RIGHT_REAR_CAN_ID = 4;

  static final int PIGEON2_CAN_ID = 1;

  // Common physical characteristics for Quasics' robots (and directly derived
  // values).
  static final double ANDYMARK_6IN_PLACTION_DIAMETER_METERS = Units.inchesToMeters(6.0);
  static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * ANDYMARK_6IN_PLACTION_DIAMETER_METERS;

  // Hardware control/sensing.
  //
  private final Pigeon2 m_gyro = new Pigeon2(PIGEON2_CAN_ID);
  private final IGyro m_iGyro = IGyro.wrapGyro(m_gyro);
  private final IGyro m_offsetGyro = new OffsetGyro(m_iGyro);

  final CANSparkMax m_leftRear = new CANSparkMax(LEFT_REAR_CAN_ID, MotorType.kBrushless);
  final CANSparkMax m_rightRear = new CANSparkMax(RIGHT_REAR_CAN_ID, MotorType.kBrushless);
  final CANSparkMax m_leftFront = new CANSparkMax(LEFT_FRONT_CAN_ID, MotorType.kBrushless);
  final CANSparkMax m_rightFront = new CANSparkMax(RIGHT_FRONT_CAN_ID, MotorType.kBrushless);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftRear, m_leftFront);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightRear, m_rightFront);

  private final RelativeEncoder m_leftEncoder = m_leftRear.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightRear.getEncoder();

  private final TrivialEncoder m_leftTrivialEncoder = new SparkMaxEncoderWrapper(m_leftEncoder);
  private final TrivialEncoder m_rightTrivialEncoder = new SparkMaxEncoderWrapper(m_rightEncoder);

  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_offsetGyro.getRotation2d(),
      m_leftTrivialEncoder.getPosition(), m_rightTrivialEncoder.getPosition());

  /**
   * Preferred constructor.
   *
   * @param robot specifies the robot-specific characteristics of the actual
   *              device to be driven
   */
  public RealDrivebase(RobotCharacteristics robot) {
    this(robot.name(), robot.kTrackWidthMeters, robot.gearRatio, robot.kP, robot.kI, robot.kD,
        robot.kS, robot.kV, robot.kA);
  }

  /**
   * Detailed constructor.
   */
  private RealDrivebase(String name, double trackWidthMeters, double gearRatio, double kP,
      double kI, double kD, double kS, double kV, double kA) {
    super(trackWidthMeters, kP, kI, kD, kS, kV, kA);

    super.setName(getClass().getSimpleName());

    ////////////////////////////////////////
    // Configure the motors.

    // * Motor inversions (if needed, and the motors aren't already
    // soft-configured).
    // m_rightGroup.setInverted(true);

    // Set initial coast/brake mode.
    enableCoastingMode(false);

    ////////////////////////////////////////
    // Configure the encoders.
    System.out.println("Configuring drivebase for " + name);
    System.out.println("Wheel circumference (m): " + WHEEL_CIRCUMFERENCE_METERS);

    // Conversion factor from units in rotations (or RPM) to meters (or m/s).
    final double distanceScalingFactorForGearing = WHEEL_CIRCUMFERENCE_METERS / gearRatio;
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;
    System.out.println("Using gear ratio: " + gearRatio);
    System.out.println("Adjustment for gearing (m/rotation): " + distanceScalingFactorForGearing);
    System.out.println("Velocity adj.: " + velocityScalingFactor);

    m_leftEncoder.setPositionConversionFactor(distanceScalingFactorForGearing);
    m_rightEncoder.setPositionConversionFactor(distanceScalingFactorForGearing);

    m_leftEncoder.setVelocityConversionFactor(velocityScalingFactor);
    m_rightEncoder.setVelocityConversionFactor(velocityScalingFactor);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Tell the motors to coast (or brake) if they're not being told how fast to
   * go (e.g., when the robot is disabled, or not being driven in auto mode).
   *
   * @param tf iff true, configure for coast mode; otherwise, for braking
   */
  void enableCoastingMode(boolean tf) {
    final var mode = (tf ? IdleMode.kCoast : IdleMode.kBrake);
    m_leftFront.setIdleMode(mode);
    m_leftRear.setIdleMode(mode);
    m_rightFront.setIdleMode(mode);
    m_rightRear.setIdleMode(mode);
  }

  protected DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  protected TrivialEncoder getLeftEncoder() {
    return m_leftTrivialEncoder;
  }

  protected TrivialEncoder getRightEncoder() {
    return m_rightTrivialEncoder;
  }

  protected IGyro getGyro() {
    return m_offsetGyro;
  }

  protected double getLeftSpeedPercentage() {
    return m_leftRear.get();
  }

  protected double getRightSpeedPercentage() {
    return m_rightRear.get();
  }

  @Override
  protected void setMotorVoltagesImpl(double leftVoltage, double rightVoltage) {
    m_leftGroup.setVoltage(leftVoltage);
    m_rightGroup.setVoltage(rightVoltage);

    logValue("Left volts", leftVoltage);
    logValue("Right volts", rightVoltage);
  }
}
