// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;

public class XrpDrivebase extends AbstractDrivebase {
  private static final double kGearRatio = (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterMeters = 0.060;

  // Sample PID constants.
  private static final double kP = 8.5;
  private static final double kI = 0;
  private static final double kD = 0;

  // Motor gains are for example purposes only, and must be determined for your
  // own robot.
  private static final double kS = 1;
  private static final double kV = 3;

  private static final double kTrackWidthMeters = 0.155;

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final TrivialEncoder m_leftTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder);
  private final TrivialEncoder m_rightTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder);

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();
  private final IGyro m_wrappedGyro = IGyro.wrapYawGyro(m_gyro);

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new XrpDrivebase. */
  public XrpDrivebase() {
    super(kTrackWidthMeters, kP, kI, kD, kS, kV);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);

    m_odometry = new DifferentialDriveOdometry(m_wrappedGyro.getRotation2d(), m_leftTrivialEncoder.getPosition(),
        m_rightTrivialEncoder.getPosition());
  }

  @Override
  protected DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  @Override
  protected TrivialEncoder getLeftEncoder() {
    return m_leftTrivialEncoder;
  }

  @Override
  protected TrivialEncoder getRightEncoder() {
    return m_rightTrivialEncoder;
  }

  @Override
  protected IGyro getGyro() {
    return m_wrappedGyro;
  }

  @Override
  protected void setMotorVoltagesImpl(double leftVoltage, double rightVoltage) {
    m_leftMotor.setVoltage(leftVoltage);
    m_rightMotor.setVoltage(rightVoltage);
  }
}
