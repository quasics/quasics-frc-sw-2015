// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.sensors.RomiGyro;
import frc.robot.utils.RobotSettings;

public class Drivetrain extends AbstractDriveBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMillimeters = 70; // 2.75591 inches

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoderLive = new Encoder(4, 5);
  private final Encoder m_rightEncoderLive = new Encoder(6, 7);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
  
  class TrivialEncoderImpl implements TrivialEncoder {
    final Encoder encoder;

    TrivialEncoderImpl(Encoder encoder) {
      this.encoder = encoder;
    }

    @Override
    public double getPosition() {
      return encoder.getDistance();
    }

    @Override
    public double getVelocity() {
      return encoder.getRate();
    }

    @Override
    public void reset() {
      encoder.reset();
    }
  }

  private final TrivialEncoder m_trivialLeftEncoder = new TrivialEncoderImpl(m_leftEncoderLive);
  private final TrivialEncoder m_trivialRightEncoder = new TrivialEncoderImpl(m_rightEncoderLive);

  /** Creates a new Drivetrain. */
  public Drivetrain(RobotSettings robotSettings) {
    super(robotSettings);
  
    setName("Drivetrain");

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoderLive.setDistancePerPulse((Math.PI * kWheelDiameterMillimeters) / kCountsPerRevolution);
    m_rightEncoderLive.setDistancePerPulse((Math.PI * kWheelDiameterMillimeters) / kCountsPerRevolution);
    resetEncoders();

    // Set up our base class.
    configureDifferentialDrive(m_leftMotor, m_rightMotor);
  }

  public void finalizeSetup() {
    configureDifferentialDrive(m_leftMotor, m_rightMotor);
  }

  public double getWheelPlacementDiameterMillimeters() {
    /* 
       Quoting from sample Romi code provided by WPILib:
         The standard Romi chassis found here,
         https://www.pololu.com/category/203/romi-chassis-kits,
         has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
         or 5.551 inches. We then take into consideration the width of the tires.
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

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
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
  public Gyro getZAxisGyro() {
    return new Gyro() {
      @Override
      public void close() throws Exception {}

      @Override
      public void calibrate() {}

      @Override
      public void reset() {
        m_gyro.reset();
      }

      @Override
      public double getAngle() {
        return m_gyro.getAngleZ();
      }

      @Override
      public double getRate() {
        return m_gyro.getRateZ();
      }
    };
  }
}
