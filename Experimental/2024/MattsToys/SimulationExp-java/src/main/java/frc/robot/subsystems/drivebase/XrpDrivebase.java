// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

/**
 * Implementing a version of the AbstractDrivebase functionality that works with
 * an XRP device, allowing initial prototyping/development of code.
 * 
 * Possible future enhancements:
 * <ul>
 * <li>
 * Wire in the ultrasonic rangefinder (analog input 2), ranging from 0V (20mm)
 * to 5V (4000mm).
 * </li>
 * <li>
 * Wire in the line following (reflectance) Sensor (analog input 0 for left, 1
 * for right), with returned values ranging from 0V (pure white) to 5V (pure
 * black).
 * </li>
 * </ul>
 *
 * @see
 *      https://docs.wpilib.org/en/latest/docs/xrp-robot/getting-to-know-xrp.html
 */
public class XrpDrivebase extends AbstractDrivebase {
  private static final double kGearRatio = (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterMeters = 0.060;

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the wrapper types used by the base class.
  private final TrivialEncoder m_leftTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder);
  private final TrivialEncoder m_rightTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder);
  private final IGyro m_wrappedGyro = IGyro.wrapYawGyro(m_gyro);

  /** Creates a new XrpDrivebase. */
  public XrpDrivebase() {
    super(RobotSettings.Robot.Xrp);

    // Per docs: "The right motor will spin in a backward direction when
    // positive output is applied. Thus the corresponding motor controller needs
    // to be inverted in robot code."
    m_rightMotor.setInverted(true);
    m_leftMotor.setInverted(false);

    // Use meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) /
        kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) /
        kCountsPerRevolution);
  }

  /**
   * Example of working with XRP-specific functionality.
   *
   * @return acceleration along the X-axis in g-forces.
   */
  public double getAccelerationX() {
    return m_accelerometer.getX();
  }

  // ---------------------------------------------------------------------------
  // Implementations of abstract functions from the base class.
  // ---------------------------------------------------------------------------

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
    // When simulating the behavior on the XRP, setting the voltage for the
    // motors appears not to translate into actual motor control: we need to
    // call "set()" on them in order to make things happen. But the
    // <code>AbstractDriveBase</code> class is assuming that it can look at the
    // voltages in order to compute PID components. So we need to
    // forward-calculate the expected speed, and apply both the voltage and
    // speed settings.
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
    m_leftMotor.setVoltage(leftPercent);
    m_rightMotor.setVoltage(rightPercent);

    logValue("Left volts", leftPercent);
    logValue("Right volts", rightPercent);
  }

  protected double getLeftVoltage_HAL() {
    return convertPercentSpeedToVoltage(m_leftMotor.get());
  }

  protected double getRightVoltage_HAL() {
    return convertPercentSpeedToVoltage(m_rightMotor.get());
  }
}
