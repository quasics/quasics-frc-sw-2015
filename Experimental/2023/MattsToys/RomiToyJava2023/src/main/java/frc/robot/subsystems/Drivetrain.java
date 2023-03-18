// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import frc.robot.sensors.RomiGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

/**
 * Extends the AbstractDriveBase class for use with the Romi hardware.
 */
public class Drivetrain extends AbstractDriveBase {
  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoderLive = new Encoder(4, 5);
  private final Encoder m_rightEncoderLive = new Encoder(6, 7);

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Set up the (real) RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the wrappers for access to individual-axis gyros.
  private final Gyro m_yawGyro = m_gyro.getYawGyro();
  private final Gyro m_pitchGyro = m_gyro.getPitchGyro();
  private final Gyro m_rollGyro = m_gyro.getRollGyro();

  // Set up the encoder wrappers used with the base class functionality.
  private final TrivialEncoder m_trivialLeftEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoderLive);
  private final TrivialEncoder m_trivialRightEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoderLive);

  /**
   * Constructor.
   * 
   * @param robotSettings the settings to be used with the current robot; these
   *                      will influence things like motor inversion, etc.
   */
  public Drivetrain(RobotSettings robotSettings) {
    super(robotSettings);

    setName("Drivetrain");

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor.setInverted(robotSettings.leftMotorsInverted);
    m_rightMotor.setInverted(robotSettings.rightMotorsInverted);

    // Use meters as unit for encoder distances
    final double wheelCircumferenceMeters = Math.PI * robotSettings.wheelDiameterMeters;
    final double wheelDistancePerPulse = wheelCircumferenceMeters / robotSettings.gearRatio;
    m_leftEncoderLive.setDistancePerPulse(wheelDistancePerPulse);
    m_rightEncoderLive.setDistancePerPulse(wheelDistancePerPulse);
    resetEncoders();

    // Set up our base class.
    configureDifferentialDrive(m_leftMotor, m_rightMotor);

    // It looks like the simulation on the Romi may not feed the drive safety checks
    // as quickly as needed, so we'll disable the checks here. (This is *not*
    // generally encouraged, but it both gets rid of repeated warnings in the
    // Terminal and also prevents the drive from "stuttering".)
    m_diffDrive.setSafetyEnabled(false);
  }

  @Override
  public void finalizeSetup() {
    configureDifferentialDrive(m_leftMotor, m_rightMotor);
  }

  @Override
  public double getWheelPlacementDiameterMillimeters() {
    /*
     * Quoting from sample Romi code provided by WPILib:
     * The standard Romi chassis found here,
     * https://www.pololu.com/category/203/romi-chassis-kits,
     * has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
     * or 5.551 inches. We then take into consideration the width of the tires.
     */
    return 70.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  @Override
  public final double getYawDegrees() {
    return m_gyro.getAngleZ();
  }

  /**
   * Current angle of the Romi around the X-axis (roll).
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis (pitch).
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis (yaw).
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
    m_yawGyro.reset();
    m_pitchGyro.reset();
    m_rollGyro.reset();
  }

  @Override
  protected TrivialEncoder getLeftEncoder() {
    return m_trivialLeftEncoder;
  }

  @Override
  protected TrivialEncoder getRightEncoder() {
    return m_trivialRightEncoder;
  }

  @Override
  public Gyro getYawGyro() {
    return m_yawGyro;
  }

  @Override
  public Gyro getPitchGyro() {
    return m_pitchGyro;
  }

  @Override
  public Gyro getRollGyro() {
    return m_rollGyro;
  }
}
