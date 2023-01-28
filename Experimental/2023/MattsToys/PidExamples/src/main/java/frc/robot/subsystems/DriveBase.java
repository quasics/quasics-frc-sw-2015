// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
  private final CANSparkMax m_leftFront = new CANSparkMax(Constants.MotorIds.SparkMax.LEFT_FRONT_DRIVE_MOTOR_ID,
      MotorType.kBrushless);
  private final CANSparkMax m_leftRear = new CANSparkMax(Constants.MotorIds.SparkMax.LEFT_REAR_DRIVE_MOTOR_ID,
      MotorType.kBrushless);
  private final CANSparkMax m_rightFront = new CANSparkMax(Constants.MotorIds.SparkMax.RIGHT_FRONT_DRIVE_MOTOR_ID,
      MotorType.kBrushless);
  private final CANSparkMax m_rightRear = new CANSparkMax(Constants.MotorIds.SparkMax.RIGHT_REAR_DRIVE_MOTOR_ID,
      MotorType.kBrushless);

  private final SparkMaxPIDController m_leftController;
  private final SparkMaxPIDController m_rightController;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDriveOdometry m_odometry;

  private final Pigeon2 m_pigeon = new Pigeon2(Constants.PIGEON2_CAN_ID);

  /** Creates a new DriveBase. */
  public DriveBase() {
    setName("DriveBase");

    final double GEAR_RATIO = Constants.DRIVE_BASE_GEAR_RATIO_SALLY;
    final double TRACK_WIDTH = Constants.TRACK_WIDTH_METERS_SALLY;

    m_rightRear.follow(m_rightFront);
    m_leftRear.follow(m_leftFront);

    m_leftController = m_leftFront.getPIDController();
    m_rightController = m_rightFront.getPIDController();

    m_leftEncoder = m_leftFront.getEncoder();
    m_rightEncoder = m_rightFront.getEncoder();

    // Update velocity reporting to be in meters (vs. rotations)
    final double wheelCircumference_meters = Math.PI * Constants.WHEEL_DIAMETER_METERS;
    final double distancePerRotation_meters = wheelCircumference_meters / GEAR_RATIO;
    m_leftEncoder.setPositionConversionFactor(distancePerRotation_meters);
    m_rightEncoder.setPositionConversionFactor(distancePerRotation_meters);

    // Update velocity reporting from RPM to m/s
    final double velocityConversionFactor = distancePerRotation_meters // converting revolutions to meters
        / 60; // converting minutes to seconds
    m_leftEncoder.setVelocityConversionFactor(velocityConversionFactor);
    m_rightEncoder.setVelocityConversionFactor(velocityConversionFactor);

    m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_pigeon.getYaw()),
        m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition());

    // simulation initiation
    if (Robot.isSimulation()) {
      // add sparks to physics simulator
      REVPhysicsSim.getInstance().addSparkMax(m_leftFront, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_leftRear, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_rightFront, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_rightRear, DCMotor.getNEO(1));
    }
  }

  public double getMaxSpeed() {
    return Constants.MAX_SPEED_METERS_PER_SEC;
  }

  public void stop() {
    m_leftController.setReference(0, ControlType.kDutyCycle);
    m_rightController.setReference(0, ControlType.kDutyCycle);
    m_rightFront.stopMotor();
    m_leftFront.stopMotor();
  }

  public void tankDrive(double leftPercent, double rightPercent) {
    final double leftVelocity = leftPercent * Constants.MAX_SPEED_METERS_PER_SEC;
    final double rightVelocity = rightPercent * Constants.MAX_SPEED_METERS_PER_SEC;

    m_leftController.setReference(leftVelocity, ControlType.kDutyCycle);
    m_rightController.setReference(rightVelocity, ControlType.kDutyCycle);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(m_pigeon.getYaw()),
        m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition());

    if (Robot.isSimulation()) {
      REVPhysicsSim.getInstance().run();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("DriveBase");

    // Report key drive base data
    builder.addDoubleProperty("Left distance (m)",
        m_leftEncoder::getPosition,
        null);
    builder.addDoubleProperty("Right distance (m)",
        m_rightEncoder::getPosition,
        null);
    builder.addDoubleProperty("Max speed",
        this::getMaxSpeed,
        null);
    builder.addDoubleProperty("Left velocity (m/s)",
        m_leftEncoder::getVelocity,
        null);
    builder.addDoubleProperty("Right velocity (m/s)",
        m_rightEncoder::getVelocity,
        null);
    builder.addDoubleProperty("Gyro angle",
        m_pigeon::getYaw,
        null);
  }
}
