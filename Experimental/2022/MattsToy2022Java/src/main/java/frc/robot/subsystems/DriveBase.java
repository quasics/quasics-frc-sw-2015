// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utils.BooleanSetter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {

  final private RelativeEncoder leftEncoder;
  final private RelativeEncoder rightEncoder;

  final private DifferentialDrive drive;
  final private BooleanSetter coastingEnabled;

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
    MotorControllerGroup leftMotors = new MotorControllerGroup(leftFront, leftRear);
    MotorControllerGroup rightMotors = new MotorControllerGroup(rightFront, rightRear);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    ////////////////////////////////////////
    // Configure the encoders.
    final double METERS_PER_INCH = 0.0254;
    final double wheelCircumferenceMeters = (Constants.WHEEL_DIAMETER_INCHES * METERS_PER_INCH) * Math.PI;
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
  }

  /**
   * Stops the drive base.
   */
  public void stop() {
    setPower(0, 0);
  }

  /**
   * Sets the power for the left and right side of the drive base.
   * 
   * @param leftPercent  % power to apply to left side (-1.0 to +1.0)
   * @param rightPercent % power to apply to right side (-1.0 to +1.0)
   */
  public void setPower(double leftPercent, double rightPercent) {
    // Set the power
    drive.tankDrive(leftPower, rightPower);
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
  }
}
