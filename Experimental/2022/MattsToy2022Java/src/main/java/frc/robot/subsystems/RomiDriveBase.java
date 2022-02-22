package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.RobotSettings;
import frc.robot.sensors.RomiGyro;
import frc.robot.utils.SimulatedGyro;

public class RomiDriveBase extends AbstractDriveBase {
  /** Implementation of TrivialEncoder for use with the Romis. */
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

  /** Encoder ticks per revolution of the wheels on a Romi. */
  private static final double kCountsPerRevolution = 1440.0;

  /** Diameter of the wheels on the Romi. */
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  /** Left motor on the Romi unit (tied to PWM 0). */
  private final Spark m_leftMotor = new Spark(0);
  /** Right motor on the Romi unit (tied to PWM 1). */
  private final Spark m_rightMotor = new Spark(1);

  /** Encoder on the left side of the Romi (tied to DIO pins 4/5). */
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  /** Encoder on the right side of the Romi (tied to DIO pins 6/7). */
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final TrivialEncoder m_leftEncoderWrapper = new TrivialEncoderImpl(m_leftEncoder);
  private final TrivialEncoder m_rightEncoderWrapper = new TrivialEncoderImpl(m_rightEncoder);

  /** The differential drive controller for the robot. */
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  /** Wrapper for the Gyro hardware on the Romi. */
  private final RomiGyro m_gyro = new RomiGyro();

  /**
   * Constructor.
   * 
   * @param settings settings to be used in configuring the Romi's drive base.
   */
  public RomiDriveBase(RobotSettings settings) {
    super(settings);

    if (settings.leftMotorsInverted) {
      m_leftMotor.setInverted(true);
    }
    if (settings.rightMotorsInverted) {
      m_rightMotor.setInverted(true);
    }

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);

    resetEncoders();
  }

  @Override
  protected void doTankDrive(double leftPercent, double rightPercent) {
    m_diffDrive.tankDrive(leftPercent, rightPercent);
  }

  @Override
  protected void doArcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    m_diffDrive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  @Override
  protected TrivialEncoder getLeftEncoder() {
    return m_leftEncoderWrapper;
  }

  @Override
  protected TrivialEncoder getRightEncoder() {
    return m_rightEncoderWrapper;
  }

  @Override
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(-rightVolts);
    m_diffDrive.feed();
  }

  /** Wrapper to provide access to the Romi's Z-axis gyro *as* a Gyro object. */
  private SimulatedGyro simpleGyro = new SimulatedGyro(
      /* close */ () -> {
      },
      /* calibrate */ () -> {
      },
      /* reset */ () -> {
        m_gyro.reset();
      },
      /* getAngle */ () -> {
        return m_gyro.getAngleZ();
      },
      /* getRate */ () -> {
        return m_gyro.getRateZ();
      });

  @Override
  public Gyro getZAxisGyro() {
    return simpleGyro;
  }
}
