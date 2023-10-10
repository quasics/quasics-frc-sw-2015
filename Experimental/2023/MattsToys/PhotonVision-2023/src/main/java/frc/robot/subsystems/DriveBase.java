// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utils.BooleanSetter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveBase extends AbstractDriveBase {
  class SparkMaxEncoderWrapper implements BasicEncoderInterface {
    final RelativeEncoder encoder;

    SparkMaxEncoderWrapper(RelativeEncoder encoder) {
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

  /** Encoder used to determine distance left wheels have travelled. */
  final private BasicEncoderInterface m_leftEncoder;
  /** Encoder used to determine distance right wheels have travelled. */
  final private BasicEncoderInterface m_rightEncoder;

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

  /** Creates a new DriveBase. */
  public DriveBase() {
    super();

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
    // Note: settings are for Nike in 2022 build season.
    final boolean leftMotorsInverted = true;
    final boolean rightMotorsInverted = false;
    leftRear.setInverted(leftMotorsInverted);
    leftFront.setInverted(leftMotorsInverted);
    rightRear.setInverted(rightMotorsInverted);
    rightFront.setInverted(rightMotorsInverted);

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

    // "Drive safety" settings
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#motor-safety
    m_drive.setExpiration(0.1); // Set watchdog timeout to 0.1sec (should be default)
    m_drive.setSafetyEnabled(true); // Turn on "drive safety" checks (should be default)

    ////////////////////////////////////////
    // Configure the encoders.

    final double wheelCircumferenceMeters = Units.inchesToMeters(Constants.WHEEL_DIAMETER_INCHES);
    System.out.println("Wheel circumference (m): " + wheelCircumferenceMeters);

    // Compute conversion factor to be applied to convert encoder readings from
    // units in rotations (or RPM) to meters (or m/s).
    final double gearRatio = Constants.DRIVE_BASE_GEAR_RATIO_NIKE; // robotSettings.gearRatio;
    System.out.println("Using gear ratio: " + gearRatio);
    final double adjustmentForGearing = wheelCircumferenceMeters / gearRatio;
    System.out.println("Adjustment for gearing (m/rotation): " + adjustmentForGearing);

    // Further conversion factor from m/min to m/s (used for velocity).
    final double velocityAdjustment = adjustmentForGearing / 60;
    System.out.println("Velocity adjustment factor: " + velocityAdjustment);

    nativeLeftEncoder.setPositionConversionFactor(adjustmentForGearing);
    nativeRightEncoder.setPositionConversionFactor(adjustmentForGearing);

    nativeLeftEncoder.setVelocityConversionFactor(velocityAdjustment);
    nativeRightEncoder.setVelocityConversionFactor(velocityAdjustment);

    nativeLeftEncoder.setPosition(0);
    nativeRightEncoder.setPosition(0);

    // Hang onto encoders for future reference by superclass.
    m_leftEncoder = new SparkMaxEncoderWrapper(nativeLeftEncoder);
    m_rightEncoder = new SparkMaxEncoderWrapper(nativeRightEncoder);
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
  protected BasicEncoderInterface getLeftEncoder() {
    return m_leftEncoder;
  }

  @Override
  protected BasicEncoderInterface getRightEncoder() {
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
}
