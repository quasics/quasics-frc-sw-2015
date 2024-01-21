// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.OffsetGyro;
import frc.robot.sensors.SparkMaxEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

/**
 * Drive base subsystem for actual FRC hardware, using the motor configuration
 * that Quasics has employed for the last few years (Spark MAX motors at known
 * CAN addresses).
 */
public class RealDrivebase extends AbstractDrivebase {
  // Motor IDs are based on those Quasics has used over the last couple of
  // years.
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

  // Motors
  final CANSparkMax m_leftRear = new CANSparkMax(LEFT_REAR_CAN_ID, MotorType.kBrushless);
  final CANSparkMax m_rightRear = new CANSparkMax(RIGHT_REAR_CAN_ID, MotorType.kBrushless);
  final CANSparkMax m_leftFront = new CANSparkMax(LEFT_FRONT_CAN_ID, MotorType.kBrushless);
  final CANSparkMax m_rightFront = new CANSparkMax(RIGHT_FRONT_CAN_ID, MotorType.kBrushless);

  // Leaders (only valid if kMotorConfigModel is not NoLeader)
  final CANSparkMax m_leftLeader;
  final CANSparkMax m_rightLeader;

  private final RelativeEncoder m_leftEncoder = m_leftRear.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightRear.getEncoder();

  private final TrivialEncoder m_leftTrivialEncoder = new SparkMaxEncoderWrapper(m_leftEncoder);
  private final TrivialEncoder m_rightTrivialEncoder = new SparkMaxEncoderWrapper(m_rightEncoder);

  /**
   * Preferred constructor.
   *
   * @param robot specifies the robot-specific characteristics of the actual
   *              device to be driven
   */
  public RealDrivebase(RobotSettings.Robot robot) {
    this(robot, robot.trackWidthMeters, robot.gearRatio,
        robot.kP, robot.kI, robot.kD,
        robot.kS, robot.kV, robot.kA);
  }

  /**
   * Detailed constructor.
   */
  private RealDrivebase(
      RobotSettings.Robot robot, double trackWidthMeters, double gearRatio,
      double kP, double kI, double kD,
      double kS, double kV, double kA) {
    super(trackWidthMeters, kP, kI, kD, kS, kV, kA);

    super.setName(getClass().getSimpleName());

    switch (robot.motorConfigModel) {
      case RearMotorsLeading:
        m_leftLeader = m_leftRear;
        m_rightLeader = m_rightRear;
        break;
      case FrontMotorsLeading:
        m_leftLeader = m_leftFront;
        m_rightLeader = m_rightFront;
        break;
      case NoLeader:
      default:
        m_leftLeader = null;
        m_rightLeader = null;
        break;
    }

    ////////////////////////////////////////
    // Configure the motors.
    System.err.println("*** Note: Configuring motors as " + robot.motorConfigModel);

    // * Motor inversions (if needed, and the motors aren't already
    // soft-configured).
    // m_rightGroup.setInverted(true);

    // Set initial coast/brake mode.
    enableCoastingMode(false);

    ////////////////////////////////////////
    // Configure the encoders.
    System.out.println("Configuring drivebase for " + robot.name());
    System.out.println("Wheel circumference (m): " +
        WHEEL_CIRCUMFERENCE_METERS);

    // Conversion factor from units in rotations (or RPM) to meters (or m/s).
    final double distanceScalingFactorForGearing = WHEEL_CIRCUMFERENCE_METERS / gearRatio;
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;
    System.out.println("Using gear ratio: " + gearRatio);
    System.out.println("Adjustment for gearing (m/rotation): " +
        distanceScalingFactorForGearing);
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
    if (m_leftLeader != null && m_rightLeader != null) {
      m_leftLeader.setIdleMode(mode);
      m_rightLeader.setIdleMode(mode);
    } else {
      m_leftFront.setIdleMode(mode);
      m_leftRear.setIdleMode(mode);
      m_rightFront.setIdleMode(mode);
      m_rightRear.setIdleMode(mode);
    }
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
    return (m_leftLeader != null ? m_leftLeader : m_leftRear).get();
  }

  protected double getRightSpeedPercentage() {
    return (m_rightLeader != null ? m_rightLeader : m_rightRear).get();
  }

  @Override
  protected void setMotorVoltagesImpl(double leftVoltage, double rightVoltage) {
    // TODO: Remove one pair, once CAN motors are configured for lead/follow on
    // drive bases.
    if (m_leftLeader != null && m_rightLeader != null) {
      m_leftLeader.setVoltage(leftVoltage);
      m_rightLeader.setVoltage(rightVoltage);
    } else {
      m_leftFront.setVoltage(leftVoltage);
      m_rightFront.setVoltage(rightVoltage);
      m_leftRear.setVoltage(leftVoltage);
      m_rightRear.setVoltage(rightVoltage);
    }

    logValue("Left volts", leftVoltage);
    logValue("Right volts", rightVoltage);
  }
}
