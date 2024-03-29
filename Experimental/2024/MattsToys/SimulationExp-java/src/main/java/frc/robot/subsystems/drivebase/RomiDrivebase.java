// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

public class RomiDrivebase extends AbstractDrivebase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeters = 0.070; // 2.75591 in

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the wrapper types used by the base class.
  private final TrivialEncoder m_leftTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder);
  private final TrivialEncoder m_rightTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder);
  private final IGyro m_wrappedGyro = IGyro.wrapYawGyro(m_gyro);

  /** Creates a new RomiDrivebase. */
  public RomiDrivebase() {
    super(RobotSettings.Robot.Romi);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) /
        kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) /
        kCountsPerRevolution);
  }

  @Override
  protected TrivialEncoder getLeftEncoder_HAL() {
    return m_leftTrivialEncoder;
  }

  @Override
  protected TrivialEncoder getRightEncoder_HAL() {
    return m_rightTrivialEncoder;
  }

  @Override
  protected IGyro getGyro_HAL() {
    return m_wrappedGyro;
  }

  /**
   * If true, log voltage/speed computation data to stdout.
   *
   * @see #setMotorVoltagesImpl
   */
  final static boolean LOG_MOTOR_SETTINGS = false;

  @Override
  protected void setMotorVoltages_HAL(double leftVoltage, double rightVoltage) {
    final double leftSpeed = convertVoltageToPercentSpeed(leftVoltage);
    final double rightSpeed = convertVoltageToPercentSpeed(rightVoltage);
    if (LOG_MOTOR_SETTINGS) {
      System.out.println("> XrpDrive - voltages: " + leftVoltage + " / " +
          rightVoltage + "\tspeeds: " + leftSpeed + " / " +
          rightSpeed);
    }
    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
    m_leftMotor.setVoltage(leftVoltage);
    m_rightMotor.setVoltage(rightVoltage);

    logValue("Left volts", leftVoltage);
    logValue("Right volts", rightVoltage);
  }

  protected double getLeftSpeedPercentage_HAL() {
    return m_leftMotor.get();
  }

  protected double getRightSpeedPercentage_HAL() {
    return m_rightMotor.get();
  }

  protected void tankDrivePercent_HAL(double leftPercent, double rightPercent) {
    m_leftMotor.set(leftPercent);
    m_rightMotor.set(rightPercent);

    logValue("Left percent", leftPercent);
    logValue("Right percent", rightPercent);
  }

  protected double getLeftVoltage_HAL() {
    return convertPercentSpeedToVoltage(m_leftMotor.get());
  }

  protected double getRightVoltage_HAL() {
    return convertPercentSpeedToVoltage(m_rightMotor.get());
  }
}
