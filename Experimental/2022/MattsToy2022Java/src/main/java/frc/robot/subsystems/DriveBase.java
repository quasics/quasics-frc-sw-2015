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

  final private DifferentialDrive drive;

  private double leftPower = 0;
  private double rightPower = 0;

  /** Creates a new DriveBase. */
  public DriveBase() {
    this.setName("DriveBase");

    // Create the individual motors.
    final CANSparkMax leftRear = new CANSparkMax(Constants.MotorIds.LEFT_REAR_DRIVE_MOTOR_ID,
        MotorType.kBrushless);
    final CANSparkMax rightRear = new CANSparkMax(Constants.MotorIds.RIGHT_REAR_DRIVE_MOTOR_ID,
        MotorType.kBrushless);
    final CANSparkMax leftFront = new CANSparkMax(Constants.MotorIds.LEFT_FRONT_DRIVE_MOTOR_ID,
        MotorType.kBrushless);
    final CANSparkMax rightFront = new CANSparkMax(Constants.MotorIds.RIGHT_FRONT_DRIVE_MOTOR_ID,
        MotorType.kBrushless);

    // Configure which ones are inverted/not.
    leftRear.setInverted(false);
    leftFront.setInverted(false);
    rightRear.setInverted(true);
    rightFront.setInverted(true);

    // Hang onto encoders for future reference.
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
    // Remember these for "periodic", to prevent the DifferentialDrive from
    // being "starved" if a command doesn't update often enough.
    leftPower = leftPercent;
    rightPower = rightPercent;

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

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // Update the DifferentialDrive (so that it doesn't get cranky about infrequent
    // updates).
    drive.tankDrive(leftPower, rightPower);
  }
}
