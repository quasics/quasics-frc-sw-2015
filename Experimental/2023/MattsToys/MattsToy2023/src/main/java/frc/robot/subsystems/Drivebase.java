// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;

public class Drivebase extends AbstractDriveBase {

  /** Motor group for the left side. */
  final private MotorController m_leftMotors;

  /** Motor group for the right side. */
  final private MotorController m_rightMotors;

  /** Encoder for the left side distance computations. */
  final private RelativeEncoder m_leftEncoder;

  /** Encoder for the right side distance computations. */
  final private RelativeEncoder m_rightEncoder;

  /** Creates a new Drivebase. */
  public Drivebase() {
    super.setName("Drivebase");

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
    final boolean LEFT_INVERTED = false;
    final boolean RIGHT_INVERTED = false;
    leftRear.setInverted(LEFT_INVERTED);
    leftFront.setInverted(LEFT_INVERTED);
    rightRear.setInverted(RIGHT_INVERTED);
    rightFront.setInverted(RIGHT_INVERTED);

    /////////////////////////////////
    // Set up the differential drive.
    m_leftEncoder = leftRear.getEncoder();
    m_rightEncoder = rightRear.getEncoder();

    m_leftMotors = new MotorControllerGroup(leftFront, leftRear);
    m_rightMotors = new MotorControllerGroup(rightFront, rightRear);

    ////////////////////////////////////////
    // Configure the encoders.

    final double wheelCircumferenceMeters = edu.wpi.first.math.util.Units
        .inchesToMeters(Constants.WHEEL_DIAMETER_INCHES);
    System.out.println("Wheel circumference (m): " + wheelCircumferenceMeters);

    // Conversion factor from units in rotations (or RPM) to meters (or m/s).
    final double GEAR_RATIO = Constants.DRIVE_BASE_GEAR_RATIO;
    final double adjustmentForGearing = wheelCircumferenceMeters / GEAR_RATIO;
    System.out.println("Using gear ratio: " + GEAR_RATIO);
    System.out.println("Adjustment for gearing (m/rotation): " + adjustmentForGearing);

    // Further conversion factor from m/min to m/s (used for velocity).
    final double velocityAdjustment = adjustmentForGearing / 60;
    System.out.println("Velocity adj.: " + velocityAdjustment);

    m_leftEncoder.setPositionConversionFactor(adjustmentForGearing);
    m_rightEncoder.setPositionConversionFactor(adjustmentForGearing);

    m_leftEncoder.setVelocityConversionFactor(velocityAdjustment);
    m_rightEncoder.setVelocityConversionFactor(velocityAdjustment);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  @Override
  public void finalizeSetup() {
    super.configureDifferentialDrive(m_leftMotors, m_rightMotors);

    // "Drive safety" settings
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#motor-safety
    m_diffDrive.setExpiration(0.1); // Set watchdog timeout to 0.1sec (should be default)
    m_diffDrive.setSafetyEnabled(true); // Turn on "drive safety" checks (should be default)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public double getLeftDistanceMillimeters() {
    return m_leftEncoder.getPosition() * 1000;
  }

  @Override
  public double getRightDistanceMillimeters() {
    return m_rightEncoder.getPosition() * 1000;
  }

  @Override
  public double getWheelPlacementDiameterMillimeters() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }
}
