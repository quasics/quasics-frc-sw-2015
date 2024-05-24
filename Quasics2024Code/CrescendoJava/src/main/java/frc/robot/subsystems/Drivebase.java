// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import static edu.wpi.first.units.Units.*;
import java.lang.Math;

import frc.robot.Constants.CanBusIds.SparkMax;

public class Drivebase extends SubsystemBase {
  private final DifferentialDriveKinematics m_kinematics;
  private static final Measure<Velocity<Distance>> ZERO_MPS = MetersPerSecond.of(0);


  final CANSparkMax m_leftLeader = new CANSparkMax(SparkMax.LEFT_LEADER_ID, MotorType.kBrushless);
  final CANSparkMax m_rightLeader = new CANSparkMax(SparkMax.RIGHT_LEADER_ID, MotorType.kBrushless);
  
  /** Maximum linear speed is 3 meters per second. */
  public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(3.0);

  /** Maximum rotational speed is 1/2 rotation per second. */
  public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI);

  public static final Measure<Distance> TRACK_WIDTH_METERS = Meters.of(0.5588);


  private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

  static final double ANDYMARK_6IN_PLACTION_DIAMETER_METERS = Units.inchesToMeters(6.0);
  static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * ANDYMARK_6IN_PLACTION_DIAMETER_METERS;
  static final double DRIVEBASE_GEAR_RATIO = 8.45;
  
  final private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    new Rotation2d(), 0, 0, new Pose2d());

  private final Pigeon2 m_pigeon = new Pigeon2(Constants.CanBusIds.PIGEON2_CAN_ID);

  /** Creates a new Drivebase. */
  public Drivebase() {
    m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    setupSmartDashboard();
  }

  @Override
  public void periodic() {
    updateOdometry();
    double yaw = getYaw();
    double leftDistance = m_leftEncoder.getPosition();
    double rightDistance = m_rightEncoder.getPosition();
    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putNumber("Left distance", leftDistance);
    SmartDashboard.putNumber("Right distance", rightDistance);

  }

  public double getYaw() {
    return m_pigeon.getAngle();
  }

  private void updateOdometry() {
    double angle = getYaw();
    double leftDistance = m_leftEncoder.getPosition();
    double rightDistance = m_rightEncoder.getPosition();

    m_odometry.update(new Rotation2d(angle), leftDistance, rightDistance);
  }

  public void setupSmartDashboard() {
      SmartDashboard.putData("set motor 6V", new InstantCommand(() -> setVoltages(6, 6)));
  }

  public void setVoltages(double leftVoltage, double rightVoltage) {
    m_leftLeader.setVoltage(leftVoltage);
    m_rightLeader.setVoltage(rightVoltage);
  }

  public final void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    setSpeeds(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
  }

  public void configureEncoders() {
    final double distanceScalingFactorForGearing = WHEEL_CIRCUMFERENCE_METERS / DRIVEBASE_GEAR_RATIO;
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;

    m_leftEncoder.setPositionConversionFactor(distanceScalingFactorForGearing);
    m_rightEncoder.setPositionConversionFactor(distanceScalingFactorForGearing);

    m_leftEncoder.setVelocityConversionFactor(velocityScalingFactor);
    m_rightEncoder.setVelocityConversionFactor(velocityScalingFactor);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void resetOdometry() {
    resetEncoders();
    m_odometry.resetPosition(m_pigeon.getRotation2d(), null, null, new Pose2d());
  }

  public void setSpeeds(double leftSpeed, double rightSpeed) {
    // clamp speeds between -1 and 1
    leftSpeed = leftSpeed > -1 ? leftSpeed : -1;
    leftSpeed = leftSpeed < 1 ? leftSpeed : 1;
    rightSpeed = rightSpeed > -1 ? rightSpeed : -1;
    rightSpeed = rightSpeed < 1 ? rightSpeed : 1;

    m_leftLeader.set(leftSpeed);
    m_rightLeader.set(rightSpeed);
  }

  public void stop() {
    setSpeeds(0, 0);
  }

  public void arcadeDrive(Measure<Velocity<Distance>> fSpeed, Measure<Velocity<Angle>> rSpeed) {
    setSpeeds(
        m_kinematics.toWheelSpeeds(new ChassisSpeeds(fSpeed, ZERO_MPS, rSpeed)));
  }

}
