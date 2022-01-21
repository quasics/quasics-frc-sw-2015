// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {

  final private RelativeEncoder leftEncoder;
  final private RelativeEncoder rightEncoder;

  DifferentialDrive drive;

  /** Creates a new DriveBase. */
  public DriveBase() {
    this.setName("DriveBase");

    final CANSparkMax leftRear = new CANSparkMax(Constants.MotorIds.LEFT_REAR_DRIVE_MOTOR_ID,
        MotorType.kBrushless);
    final CANSparkMax rightRear = new CANSparkMax(Constants.MotorIds.RIGHT_REAR_DRIVE_MOTOR_ID,
        MotorType.kBrushless);
    final CANSparkMax leftFront = new CANSparkMax(Constants.MotorIds.LEFT_FRONT_DRIVE_MOTOR_ID,
        MotorType.kBrushless);
    final CANSparkMax rightFront = new CANSparkMax(Constants.MotorIds.RIGHT_FRONT_DRIVE_MOTOR_ID,
        MotorType.kBrushless);

    leftRear.setInverted(false);
    leftFront.setInverted(false);
    rightRear.setInverted(true);
    rightFront.setInverted(true);

    leftEncoder = leftRear.getEncoder();
    rightEncoder = rightRear.getEncoder();

    ////////////////////////////////////////
    // Configure the encoders.
    final double METERS_PER_INCH = 0.0254;
    final double wheelCircumferenceMeters = (Constants.WHEEL_DIAMETER_INCHES * METERS_PER_INCH) * Math.PI;

    // Convert (rotational) RPM to (linear) meters/min
    final double velocityAdjustmentForGearing = wheelCircumferenceMeters / Constants.DRIVE_BASE_GEAR_RATIO;

    // Adjust to meters/sec for convenience
    final double velocityAdjustment = velocityAdjustmentForGearing / 60;

    System.out.println("Wheel circumference: " + wheelCircumferenceMeters);
    System.out.println("Velocity adj. (gearing): " + velocityAdjustmentForGearing);
    System.out.println("Velocity adj. (final): " + velocityAdjustment);

    /////////////////////////////////
    // Set up the differential drive.
    MotorControllerGroup leftMotors = new MotorControllerGroup(leftFront, leftRear);
    MotorControllerGroup rightMotors = new MotorControllerGroup(rightFront, rightRear);

    leftEncoder.setVelocityConversionFactor(velocityAdjustment);
    rightEncoder.setVelocityConversionFactor(velocityAdjustment);

    drive = new DifferentialDrive(leftMotors, rightMotors);
  }

  public void stop() {
    setPower(0, 0);
  }

  public void setPower(double leftPercent, double rightPercent) {
    drive.tankDrive(leftPercent, rightPercent);
  }

  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightEncoderPosition() {
    return rightEncoder.getPosition();
  }

  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
