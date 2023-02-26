// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;
import frc.robot.sensors.NullGyro;
import frc.robot.sensors.SparkMaxEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

/** Provides drive base support for a "big bot" built using SparkMax motors on a CAN bus. */
public class Drivebase extends AbstractDriveBase {
  /** Utility class to handle detecting/reporting on faults with a Pigeon2 IMU (if used). */
  static class PigeonStatusChecker implements Runnable {
    private int lastMask = 0;
    private com.ctre.phoenix.sensors.Pigeon2_Faults m_faults =
        new com.ctre.phoenix.sensors.Pigeon2_Faults();

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

  /** Motor group for the left side. */
  private final MotorController m_leftMotors;

  /** Motor group for the right side. */
  private final MotorController m_rightMotors;

  /** Encoder for the left side distance computations. */
  private final TrivialEncoder m_leftEncoder;

  /** Encoder for the right side distance computations. */
  private final TrivialEncoder m_rightEncoder;

  /**
   * Gyro installed on the robot, based on settings provided to constructor.
   *
   * <p>This will either be a Pigeon2, an ADXRS450, or a NullGyro (which always reports "no
   * movement").
   */
  private final Gyro m_gyro;

  /**
   * Status checker used to monitor for faults reported by the Pigeon 2 IMU (iff we're using one).
   *
   * @see #USE_PIGEON_IMU
   */
  private final PigeonStatusChecker m_pigeonChecker;

  /** Creates a new Drivebase. */
  public Drivebase(RobotSettings settings) {
    super(settings);

    super.setName("Drivebase");

    /////////////////////////////////
    // Set up the motors/groups.

    // Create the individual motors.
    final CANSparkMax leftRear = new CANSparkMax(settings.leftRearMotorId, MotorType.kBrushless);
    final CANSparkMax rightRear = new CANSparkMax(settings.rightRearMotorId, MotorType.kBrushless);
    final CANSparkMax leftFront = new CANSparkMax(settings.leftFrontMotorId, MotorType.kBrushless);
    final CANSparkMax rightFront =
        new CANSparkMax(settings.rightFrontMotorId, MotorType.kBrushless);

    // Configure which motors are inverted/not.
    leftRear.setInverted(settings.leftMotorsInverted);
    leftFront.setInverted(settings.leftMotorsInverted);
    rightRear.setInverted(settings.rightMotorsInverted);
    rightFront.setInverted(settings.rightMotorsInverted);

    /////////////////////////////////
    // Set up the differential drive.
    final RelativeEncoder leftLiveEncoder = leftRear.getEncoder();
    final RelativeEncoder rightLiveEncoder = rightRear.getEncoder();

    m_leftMotors = new MotorControllerGroup(leftFront, leftRear);
    m_rightMotors = new MotorControllerGroup(rightFront, rightRear);

    ////////////////////////////////////////
    // Configure the encoders.

    final double wheelCircumferenceMeters =
        edu.wpi.first.math.util.Units.inchesToMeters(Constants.WHEEL_DIAMETER_INCHES);
    System.out.println("Wheel circumference (m): " + wheelCircumferenceMeters);

    // Conversion factor from units in rotations (or RPM) to meters (or m/s).
    final double adjustmentForGearing = wheelCircumferenceMeters / settings.gearRatio;
    System.out.println("Using gear ratio: " + settings.gearRatio);
    System.out.println("Adjustment for gearing (m/rotation): " + adjustmentForGearing);

    // Further conversion factor from m/min to m/s (used for velocity).
    final double velocityAdjustment = adjustmentForGearing / 60;
    System.out.println("Velocity adj.: " + velocityAdjustment);

    leftLiveEncoder.setPositionConversionFactor(adjustmentForGearing);
    rightLiveEncoder.setPositionConversionFactor(adjustmentForGearing);

    leftLiveEncoder.setVelocityConversionFactor(velocityAdjustment);
    rightLiveEncoder.setVelocityConversionFactor(velocityAdjustment);

    leftLiveEncoder.setPosition(0);
    rightLiveEncoder.setPosition(0);

    m_leftEncoder = new SparkMaxEncoderWrapper(leftLiveEncoder);
    m_rightEncoder = new SparkMaxEncoderWrapper(rightLiveEncoder);

    ////////////////////////////////////////
    // Configure the gyro.
    switch (settings.installedGyroType) {
      case Pigeon2:
        com.ctre.phoenix.sensors.WPI_Pigeon2 pigeon =
            new com.ctre.phoenix.sensors.WPI_Pigeon2(settings.pigeonCanId);
        m_gyro = pigeon;
        m_pigeonChecker = new PigeonStatusChecker((com.ctre.phoenix.sensors.WPI_Pigeon2) m_gyro);
        break;
      case ADXRS450:
        m_gyro =
            new edu.wpi.first.wpilibj.ADXRS450_Gyro(edu.wpi.first.wpilibj.SPI.Port.kOnboardCS0);
        m_pigeonChecker = null;
        break;
      case None:
      default:
        m_gyro = new NullGyro();
        m_pigeonChecker = null;
        break;
    }
  }

  // Give the base class what it needs to set up the differential drive, and do
  // our own h/w-specific setup.
  @Override
  public void finalizeSetup() {
    super.configureDifferentialDrive(m_leftMotors, m_rightMotors);

    // "Drive safety" settings
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#motor-safety
    m_diffDrive.setExpiration(0.1); // Set watchdog timeout to 0.1sec (should be default)
    m_diffDrive.setSafetyEnabled(true); // Turn on "drive safety" checks (should be default)
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
  protected TrivialEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  @Override
  protected TrivialEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  @Override
  public Gyro getZAxisGyro() {
    return m_gyro;
  }

  @Override
  public double getWheelPlacementDiameterMillimeters() {
    // TODO Auto-generated method stub
    return 0;
  }
}
