// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotSettings;
import frc.robot.utils.BooleanSetter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
  /**
   * Controls whether a Pigeon 2 IMU (from CTRE) will be used for the gyro, or if
   * we will just use a standard Analog Devices gyro.
   */
  static final private boolean USE_PIGEON_IMU = false;

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
  final private RelativeEncoder m_leftEncoder;
  /** Encoder used to determine distance right wheels have travelled. */
  final private RelativeEncoder m_rightEncoder;

  /** Motor group for the left side. */
  final private MotorControllerGroup m_leftMotors;
  /** Motor group for the right side. */
  final private MotorControllerGroup m_rightMotors;

  /** Differential drive normally used to actually run the motors. */
  final private DifferentialDrive m_drive;

  /**
   * Utility functor, used to conveniently update "idle mode" setting for all of
   * the motors.
   */
  final private BooleanSetter m_coastingEnabled;

  /**
   * Tracks odometry for the robot. (Updated in periodic().)
   */
  final private DifferentialDriveOdometry m_odometry;

  /** Configured tank width for the robot. */
  final private double m_tankWidth;

  /** Gyro used to determine current robot angle. */
  final private Gyro m_gyro;

  /**
   * Status checker used to monitor for faults reported by the Pigeon 2 IMU (iff
   * we're using one).
   *
   * @see #USE_PIGEON_IMU
   */
  final private PigeonStatusChecker m_pigeonChecker;

  /** Creates a new DriveBase. */
  public DriveBase(RobotSettings robotSettings) {
    super.setName("DriveBase");

    m_tankWidth = robotSettings.trackWidthMeters;

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

    // Hang onto encoders for future reference.
    m_leftEncoder = leftRear.getEncoder();
    m_rightEncoder = rightRear.getEncoder();

    /////////////////////////////////
    // Set up the differential drive.

    m_leftMotors = new MotorControllerGroup(leftFront, leftRear);
    m_rightMotors = new MotorControllerGroup(rightFront, rightRear);
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    ////////////////////////////////////////
    // Configure the encoders.

    final double wheelCircumferenceMeters = edu.wpi.first.math.util.Units
        .inchesToMeters(Constants.WHEEL_DIAMETER_INCHES);
    System.out.println("Wheel circumference (m): " + wheelCircumferenceMeters);

    // Conversion factor from units in rotations (or RPM) to meters (or m/s).
    final double adjustmentForGearing = wheelCircumferenceMeters / Constants.DRIVE_BASE_GEAR_RATIO;
    System.out.println("Adjustment for gearing (m/rotation): " + adjustmentForGearing);

    // Further conversion factor from m/min to m/s (used for velocity).
    final double velocityAdjustment = adjustmentForGearing / 60;
    System.out.println("Velocity adj.: " + velocityAdjustment);

    m_leftEncoder.setPositionConversionFactor(adjustmentForGearing);
    m_rightEncoder.setPositionConversionFactor(adjustmentForGearing);

    m_leftEncoder.setVelocityConversionFactor(velocityAdjustment);
    m_rightEncoder.setVelocityConversionFactor(velocityAdjustment);

    resetEncoders();

    ////////////////////////////////////////
    // Allocate/configure the gyro.

    if (USE_PIGEON_IMU) {
      com.ctre.phoenix.sensors.WPI_Pigeon2 pigeon = new com.ctre.phoenix.sensors.WPI_Pigeon2(Constants.PIGEON2_CAN_ID);
      m_gyro = pigeon;
      m_pigeonChecker = new PigeonStatusChecker(pigeon);
    } else {
      // Assumes "Chip Select" jumper is set to CS0
      m_gyro = new edu.wpi.first.wpilibj.ADXRS450_Gyro(edu.wpi.first.wpilibj.SPI.Port.kOnboardCS0);
      m_pigeonChecker = null;
    }

    // Gyro must be calibrated to initialize for use (generally immediately on
    // start-up).
    m_gyro.calibrate();
    m_gyro.reset();

    ////////////////////////////////////////
    // Odometry setup.

    // Notes:
    // 1) This *must* be done after the encoders have been reset to 0.
    // 2) We're building a "robot-oriented" view of the field (as in, our starting
    // position will be treated as the origin), rather than a "field-oriented" view.
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(0), new Pose2d());
  }

  /**
   * Stops the drive base.
   */
  public void stop() {
    tankDrive(0, 0);
  }

  /**
   * Sets the power for the left and right side of the drive base.
   *
   * @param leftPercent  % power to apply to left side (-1.0 to +1.0)
   * @param rightPercent % power to apply to right side (-1.0 to +1.0)
   */
  public void tankDrive(double leftPercent, double rightPercent) {
    var boundedLeft = Math.max(-1.0, Math.min(1.0, leftPercent));
    var boundedRight = Math.max(-1.0, Math.min(1.0, rightPercent));
    m_drive.tankDrive(boundedLeft, boundedRight);
  }

  /**
   * Arcade drive support. The calculated values will be squared to decrease
   * sensitivity at low speeds.
   *
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward
   *                  is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
   *                  Clockwise is positive.
   */
  public void arcadeDrive(double xSpeed, double zRotation) {
    this.arcadeDrive(xSpeed, zRotation, true);
  }

  /**
   * Arcade drive support.
   *
   * @param xSpeed       The robot's speed along the X axis [-1.0..1.0]. Forward
   *                     is positive.
   * @param zRotation    The robot's rotation rate around the Z axis [-1.0..1.0].
   *                     Clockwise is positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    m_drive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  /**
   * @return the current reading for the left encoder (in meters)
   */
  public double getLeftEncoderPosition() {
    return m_leftEncoder.getPosition();
  }

  /**
   * @return the current reading for the right encoder (in meters)
   */
  public double getRightEncoderPosition() {
    return m_rightEncoder.getPosition();
  }

  /**
   * @return the current speed for the left wheels (in meters/sec)
   */
  public double getLeftSpeed() {
    return m_leftEncoder.getVelocity();
  }

  /**
   * @return the current speed for the right wheels (in meters/sec)
   */
  public double getRightSpeed() {
    return m_rightEncoder.getVelocity();
  }

  /**
   * Resets both the left and right encoders to 0.
   */
  public void resetEncoders() {
    m_rightEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);
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
    // Update current info on faults.
    if (m_pigeonChecker != null) {
      m_pigeonChecker.run();
    }

    updateOdometry();
  }

  Rotation2d getGyroAngle() {
    return m_gyro.getRotation2d();
  }

  //////////////////////////////////////////////////////////////////
  // Trajectory-following support.

  public Pose2d GetPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }

  private void updateOdometry() {
    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());

    // Update the pose
    m_odometry.update(gyroAngle, getLeftEncoderPosition(), getRightEncoderPosition());
  }

  public void ResetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getGyroAngle());
  }

  public void TankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public double GetTrackWidth() {
    return m_tankWidth;
  }
}
