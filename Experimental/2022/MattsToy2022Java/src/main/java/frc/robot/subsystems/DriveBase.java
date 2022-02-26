// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotSettings;
import frc.robot.utils.BooleanSetter;
import frc.robot.utils.DummyGyro;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveBase extends AbstractDriveBase {
  class TrivialEncoderImpl implements TrivialEncoder {
    final RelativeEncoder encoder;

    TrivialEncoderImpl(RelativeEncoder encoder) {
      this.encoder = encoder;
    }

    @Override
    public double getPosition() {
      return encoder.getPosition();
    }

    @Override
    public double getVelocity() {
      return encoder.getVelocity();
    }

    @Override
    public void reset() {
      encoder.setPosition(0);
    }
  }

  /**
   * Utility class to handle detecting/reporting on faults with a Pigeon2 IMU (if
   * used).
   */
  static class PigeonStatusChecker implements Runnable {
    private int lastMask = 0;
    private com.ctre.phoenix.sensors.Pigeon2_Faults m_faults = new com.ctre.phoenix.sensors.Pigeon2_Faults();

    private com.ctre.phoenix.sensors.WPI_Pigeon2 m_pigeon;

    PigeonStatusChecker(com.ctre.phoenix.sensors.WPI_Pigeon2 pigeon) {
      m_pigeon = pigeon;
    }

    private String getFaultMessage() {
      if (!m_faults.hasAnyFault()) {
        return "No faults";
      }
      String retval = "";
      retval += m_faults.APIError ? "APIError, " : "";
      retval += m_faults.AccelFault ? "AccelFault, " : "";
      retval += m_faults.BootIntoMotion ? "BootIntoMotion, " : "";
      retval += m_faults.GyroFault ? "GyroFault, " : "";
      retval += m_faults.HardwareFault ? "HardwareFault, " : "";
      retval += m_faults.MagnetometerFault ? "MagnetometerFault, " : "";
      retval += m_faults.ResetDuringEn ? "ResetDuringEn, " : "";
      retval += m_faults.SaturatedAccel ? "SaturatedAccel, " : "";
      retval += m_faults.SaturatedMag ? "SaturatedMag, " : "";
      retval += m_faults.SaturatedRotVelocity ? "SaturatedRotVelocity, " : "";
      return retval;
    }

    @Override
    public void run() {
      m_pigeon.getFaults(m_faults);
      int mask = m_faults.toBitfield();
      if (mask != lastMask) {
        System.err.println("Pigeon reports: " + getFaultMessage());
        lastMask = mask;
      }
    }
  }

  /** Encoder used to determine distance left wheels have travelled. */
  final private TrivialEncoder m_leftEncoder;
  /** Encoder used to determine distance right wheels have travelled. */
  final private TrivialEncoder m_rightEncoder;

  /** Motor group for the left side. */
  final private MotorControllerGroup m_leftMotors;
  /** Motor group for the right side. */
  final private MotorControllerGroup m_rightMotors;

  /** Differential drive normally used to actually run the motors. */
  final private DifferentialDrive m_drive;

  final private Gyro m_gyro;

  /**
   * Utility functor, used to conveniently update "idle mode" setting for all of
   * the motors.
   */
  final private BooleanSetter m_coastingEnabled;

  /**
   * Status checker used to monitor for faults reported by the Pigeon 2 IMU (iff
   * we're using one).
   *
   * @see #USE_PIGEON_IMU
   */
  final private PigeonStatusChecker m_pigeonChecker;

  /** Creates a new DriveBase. */
  public DriveBase(RobotSettings robotSettings) {
    super(robotSettings);

    super.setName("DriveBase");

    /////////////////////////////////
    // Set up the motors/groups.

    // Create the individual motors.
    final CANSparkMax leftRear = new CANSparkMax(Constants.MotorIds.SparkMax.LEFT_REAR_DRIVE_MOTOR_ID,
        MotorType.kBrushless);
    final CANSparkMax rightRear = new CANSparkMax(Constants.MotorIds.SparkMax.RIGHT_REAR_DRIVE_MOTOR_ID,
        MotorType.kBrushless);
    final CANSparkMax leftFront = new CANSparkMax(Constants.MotorIds.SparkMax.LEFT_FRONT_DRIVE_MOTOR_ID,
        MotorType.kBrushless);
    final CANSparkMax rightFront = new CANSparkMax(Constants.MotorIds.SparkMax.RIGHT_FRONT_DRIVE_MOTOR_ID,
        MotorType.kBrushless);

    // Configure which motors are inverted/not.
    leftRear.setInverted(robotSettings.leftMotorsInverted);
    leftFront.setInverted(robotSettings.leftMotorsInverted);
    rightRear.setInverted(robotSettings.rightMotorsInverted);
    rightFront.setInverted(robotSettings.rightMotorsInverted);

    // Make sure that we can set coast/brake mode for the motors later.
    m_coastingEnabled = (tf) -> {
      final var applyMode = tf ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake;

      leftRear.setIdleMode(applyMode);
      rightRear.setIdleMode(applyMode);
      leftFront.setIdleMode(applyMode);
      rightFront.setIdleMode(applyMode);
    };

    /////////////////////////////////
    // Set up the differential drive.
    RelativeEncoder nativeLeftEncoder = leftRear.getEncoder();
    RelativeEncoder nativeRightEncoder = rightRear.getEncoder();

    m_leftMotors = new MotorControllerGroup(leftFront, leftRear);
    m_rightMotors = new MotorControllerGroup(rightFront, rightRear);
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    ////////////////////////////////////////
    // Configure the encoders.

    final double wheelCircumferenceMeters = edu.wpi.first.math.util.Units
        .inchesToMeters(Constants.WHEEL_DIAMETER_INCHES);
    System.out.println("Wheel circumference (m): " + wheelCircumferenceMeters);

    // Conversion factor from units in rotations (or RPM) to meters (or m/s).
    final double adjustmentForGearing = wheelCircumferenceMeters / robotSettings.gearRatio;
    System.out.println("Using gear ratio: " + robotSettings.gearRatio);
    System.out.println("Adjustment for gearing (m/rotation): " + adjustmentForGearing);

    // Further conversion factor from m/min to m/s (used for velocity).
    final double velocityAdjustment = adjustmentForGearing / 60;
    System.out.println("Velocity adj.: " + velocityAdjustment);

    nativeLeftEncoder.setPositionConversionFactor(adjustmentForGearing);
    nativeRightEncoder.setPositionConversionFactor(adjustmentForGearing);

    nativeLeftEncoder.setVelocityConversionFactor(velocityAdjustment);
    nativeRightEncoder.setVelocityConversionFactor(velocityAdjustment);

    nativeLeftEncoder.setPosition(0);
    nativeRightEncoder.setPosition(0);

    // Hang onto encoders for future reference by superclass.
    m_leftEncoder = new TrivialEncoderImpl(nativeLeftEncoder);
    m_rightEncoder = new TrivialEncoderImpl(nativeRightEncoder);

    ////////////////////////////////////////
    // Configure the gyro.
    switch (robotSettings.installedGyroType) {
      case Pigeon2:
        com.ctre.phoenix.sensors.WPI_Pigeon2 pigeon = new com.ctre.phoenix.sensors.WPI_Pigeon2(
            robotSettings.pigeonCanId);
        m_gyro = pigeon;
        m_pigeonChecker = new PigeonStatusChecker((com.ctre.phoenix.sensors.WPI_Pigeon2) m_gyro);
        break;
      case ADXRS450:
        m_gyro = new edu.wpi.first.wpilibj.ADXRS450_Gyro(edu.wpi.first.wpilibj.SPI.Port.kOnboardCS0);
        m_pigeonChecker = null;
        break;
      case None:
      default:
        m_gyro = new DummyGyro();
        m_pigeonChecker = null;
        break;
    }
  }

  @Override
  protected void doTankDrive(double leftPercent, double rightPercent) {
    m_drive.tankDrive(leftPercent, rightPercent);
  }

  @Override
  protected void doArcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    m_drive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  @Override
  protected TrivialEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  @Override
  protected TrivialEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Enables/disabled "coast" mode on the motors (when stopped).
   *
   * @param tf if true, enable "coast" mode; otherwise, enable "brake" mode
   */
  public void setCoastingEnabled(boolean tf) {
    m_coastingEnabled.set(tf);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    super.periodic();

    // Update current info on faults.
    if (m_pigeonChecker != null) {
      m_pigeonChecker.run();
    }
  }

  @Override
  public Gyro getZAxisGyro() {
    return m_gyro;
  }

  //////////////////////////////////////////////////////////////////
  // Trajectory-following support.

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }
}
