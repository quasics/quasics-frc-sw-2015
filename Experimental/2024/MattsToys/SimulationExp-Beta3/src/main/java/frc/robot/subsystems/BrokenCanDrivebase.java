// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.SparkMaxEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;

public class BrokenCanDrivebase extends AbstractDrivebase {
  // Sample PID values from 2023 "ChargedUp" constants for Sally
  private static final double kP = 0.29613;
  private static final double kI = 0;
  private static final double kD = 0;

  // Motor gains must be determined for your own robot.
  private static final double kS = 0.19529; // From 2023 "ChargedUp" constants for Sally
  private static final double kV = 2.2329; // From 2023 "ChargedUp" constants for Sally

  // Motor IDs are based on those Quasics has used over the last couple of years.
  static final int LEFT_FRONT_CAN_ID = 1;
  static final int LEFT_REAR_CAN_ID = 2;
  static final int RIGHT_FRONT_CAN_ID = 3;
  static final int RIGHT_REAR_CAN_ID = 4;

  // The robot's physical characteristics (and directly derived values).
  static final double ANDYMARK_6IN_PLACTION_DIAMETER_METERS = Units.inchesToMeters(6.0);
  static final double GLADYS_GEAR_RATIO = 8.45;
  static final double TRACK_WIDTH_METERS = 0.381 * 2;
  static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * ANDYMARK_6IN_PLACTION_DIAMETER_METERS;
  static final double DISTANCE_SCALING_FACTOR_FOR_GEARING = WHEEL_CIRCUMFERENCE_METERS / GLADYS_GEAR_RATIO;
  static final double VELOCITY_SCALING_FACTOR = DISTANCE_SCALING_FACTOR_FOR_GEARING / 60;

  // Hardware control/sensing.
  private final AnalogGyro m_gyro = new AnalogGyro(0);
  private final IGyro m_wrappedGyro = IGyro.wrapGyro(m_gyro);

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

  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_wrappedGyro.getRotation2d(),
      m_leftTrivialEncoder.getPosition(), m_rightTrivialEncoder.getPosition());

  /** Creates a new Drivebase. */
  public BrokenCanDrivebase() {
    super(TRACK_WIDTH_METERS, kP, kI, kD, kS, kV);

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

    System.out.println("Wheel circumference (m): " + WHEEL_CIRCUMFERENCE_METERS);

    // Conversion factor from units in rotations (or RPM) to meters (or m/s).
    System.out.println("Using gear ratio: " + GLADYS_GEAR_RATIO);
    System.out.println(
        "Adjustment for gearing (m/rotation): " + DISTANCE_SCALING_FACTOR_FOR_GEARING);
    System.out.println("Velocity adj.: " + VELOCITY_SCALING_FACTOR);

    m_leftEncoder.setPositionConversionFactor(DISTANCE_SCALING_FACTOR_FOR_GEARING);
    m_rightEncoder.setPositionConversionFactor(DISTANCE_SCALING_FACTOR_FOR_GEARING);

    m_leftEncoder.setVelocityConversionFactor(VELOCITY_SCALING_FACTOR);
    m_rightEncoder.setVelocityConversionFactor(VELOCITY_SCALING_FACTOR);

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
    return m_wrappedGyro;
  }

  @Override
  protected void setMotorVoltagesImpl(double leftVoltage, double rightVoltage) {
    m_leftGroup.setVoltage(leftVoltage);
    m_rightGroup.setVoltage(rightVoltage);

    logValue("Left volts", leftVoltage);
    logValue("Right volts", rightVoltage);
  }
}
