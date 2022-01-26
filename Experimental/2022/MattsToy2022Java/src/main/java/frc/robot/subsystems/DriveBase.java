// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utils.BooleanSetter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {

  final private RelativeEncoder leftEncoder;
  final private RelativeEncoder rightEncoder;

  final private MotorControllerGroup leftMotors;
  final private MotorControllerGroup rightMotors;

  final private DifferentialDrive drive;
  final private BooleanSetter coastingEnabled;

  // Assumes "Chip Select" jumper is set to CS0
  final private ADXRS450_Gyro gyro = new ADXRS450_Gyro(edu.wpi.first.wpilibj.SPI.Port.kOnboardCS0);

  final private DifferentialDriveOdometry odometry;

  /** Creates a new DriveBase. */
  public DriveBase() {
    this.setName("DriveBase");

    // Create the individual motors.
    final CANSparkMax leftRear = new CANSparkMax(Constants.MotorIds.SparkMax.LEFT_REAR_DRIVE_MOTOR_ID,
        MotorType.kBrushless);
    final CANSparkMax rightRear = new CANSparkMax(Constants.MotorIds.SparkMax.RIGHT_REAR_DRIVE_MOTOR_ID,
        MotorType.kBrushless);
    final CANSparkMax leftFront = new CANSparkMax(Constants.MotorIds.SparkMax.LEFT_FRONT_DRIVE_MOTOR_ID,
        MotorType.kBrushless);
    final CANSparkMax rightFront = new CANSparkMax(Constants.MotorIds.SparkMax.RIGHT_FRONT_DRIVE_MOTOR_ID,
        MotorType.kBrushless);

    coastingEnabled = (tf) -> {
      leftRear.setIdleMode(tf ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
      rightRear.setIdleMode(tf ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
      leftFront.setIdleMode(tf ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
      rightFront.setIdleMode(tf ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
    };

    // Configure which ones are inverted/not.
    leftRear.setInverted(true);
    leftFront.setInverted(true);
    rightRear.setInverted(false);
    rightFront.setInverted(false);

    // Hang onto encoders for future reference.
    leftEncoder = leftRear.getEncoder();
    rightEncoder = rightRear.getEncoder();

    /////////////////////////////////
    // Set up the differential drive.
    leftMotors = new MotorControllerGroup(leftFront, leftRear);
    rightMotors = new MotorControllerGroup(rightFront, rightRear);

    drive = new DifferentialDrive(leftMotors, rightMotors);

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

    leftEncoder.setPositionConversionFactor(adjustmentForGearing);
    rightEncoder.setPositionConversionFactor(adjustmentForGearing);

    leftEncoder.setVelocityConversionFactor(velocityAdjustment);
    rightEncoder.setVelocityConversionFactor(velocityAdjustment);

    resetEncoders();

    ////////////////////////////////////////
    // Configure the gyro

    // Gyro must be calibrated to initialize for use (generally immediately on
    // start-up).
    gyro.calibrate();

    ////////////////////////////////////////
    // Odometry setup.

    // Notes:
    // 1) This *must* be done after the encoders have been reset to 0.
    // 2) We're building a "robot-oriented" view of the field (as in, our starting
    // position will be treated as the origin), rather than a "field-oriented" view.
    odometry = new DifferentialDriveOdometry(new Rotation2d(0), new Pose2d());
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
    drive.tankDrive(boundedLeft, boundedRight);
  }

  /**
   * Arcade drive support. The calculated values will be squared to decrease
   * sensitivity at low speeds.
   * 
   * @param xSpeed       The robot's speed along the X axis [-1.0..1.0]. Forward
   *                     is positive.
   * @param zRotation    The robot's rotation rate around the Z axis [-1.0..1.0].
   *                     Clockwise is positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void arcadeDrive​(double xSpeed, double zRotation) {
    this.arcadeDrive​(xSpeed, zRotation, true);
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
  public void arcadeDrive​(double xSpeed, double zRotation, boolean squareInputs) {
    drive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  /**
   * @return the current reading for the left encoder (in meters)
   */
  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition();
  }

  /**
   * @return the current reading for the right encoder (in meters)
   */
  public double getRightEncoderPosition() {
    return rightEncoder.getPosition();
  }

  /**
   * @return the current speed for the left wheels (in meters/sec)
   */
  public double getLeftSpeed() {
    return leftEncoder.getVelocity();
  }

  /**
   * @return the current speed for the right wheels (in meters/sec)
   */
  public double getRightSpeed() {
    return rightEncoder.getVelocity();
  }

  /**
   * Resets both the left and right encoders to 0.
   */
  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  /**
   * Enables/disabled "coast" mode on the motors (when stopped).
   * 
   * @param tf if true, enable "coast" mode; otherwise, enable "brake" mode
   */
  public void setCoastingEnabled(boolean tf) {
    coastingEnabled.set(tf);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    updateOdometry();
  }

  Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }

  //////////////////////////////////////////////////////////////////
  // Trajectory-following support.

  Pose2d GetPose() {
    return odometry.getPoseMeters();
  }

  DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }

  private void updateOdometry() {
    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());

    // Update the pose
    odometry.update(gyroAngle, getLeftEncoderPosition(), getRightEncoderPosition());
  }

  void ResetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getGyroAngle());
  }

  void TankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  // TODO: Define these methods, once we have good data to work with.

  // double GetTrackWidth() override {
  // return kTrackWidthMeters;
  // }
}
