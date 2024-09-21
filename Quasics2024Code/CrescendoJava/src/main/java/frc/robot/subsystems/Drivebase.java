// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.MutableMeasure.mutable;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.CanBusIds.SparkMax;
import java.lang.Math;

public class Drivebase extends SubsystemBase {
  private final DifferentialDriveKinematics m_kinematics;
  private static final Measure<Velocity<Distance>> ZERO_MPS = MetersPerSecond.of(0);

  final CANSparkMax m_leftLeader = new CANSparkMax(SparkMax.LEFT_LEADER_ID, MotorType.kBrushless);
  final CANSparkMax m_leftFollower =
      new CANSparkMax(SparkMax.LEFT_FOLLOWER_ID, MotorType.kBrushless);
  final CANSparkMax m_rightLeader = new CANSparkMax(SparkMax.RIGHT_LEADER_ID, MotorType.kBrushless);
  final CANSparkMax m_rightFollower =
      new CANSparkMax(SparkMax.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

  /** Maximum linear speed is 3 meters per second. */
  public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(1.0);

  /** Maximum rotational speed is 1/2 rotation per second. */
  public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI);

  public static final Measure<Distance> TRACK_WIDTH_METERS = Meters.of(0.5588);

  private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

  static final double ANDYMARK_6IN_PLACTION_DIAMETER_METERS = Units.inchesToMeters(6.0);
  static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * ANDYMARK_6IN_PLACTION_DIAMETER_METERS;
  static final double DRIVEBASE_GEAR_RATIO = 8.45;

  final private DifferentialDriveOdometry m_odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d());

  private final Pigeon2 m_pigeon = new Pigeon2(Constants.CanBusIds.PIGEON2_CAN_ID);

  /** Creates a new Drivebase. */
  public Drivebase() {
    m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    resetOdometry();
    setupSmartDashboard();
    configureEncoders();
    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);
  }

  @Override
  public void periodic() {
    updateOdometry();
    double yaw = getYaw();
    double leftDistance = m_leftEncoder.getPosition();
    double leftVelocity = m_leftEncoder.getVelocity();
    double rightDistance = m_rightEncoder.getPosition();
    double rightVelocity = m_rightEncoder.getVelocity();
    double leftVoltage = getLeftVoltage();
    double rightVoltage = getRightVoltage();


    CANSparkMax.IdleMode mode = m_leftLeader.getIdleMode();
    String drive;
    if (mode == CANSparkMax.IdleMode.kBrake) {
      drive = "Breaking Mode";
    } else {
      drive = "Coasting Mode";
    }
    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putNumber("Left distance", leftDistance);
    SmartDashboard.putNumber("Right distance", rightDistance);
    SmartDashboard.putNumber("Left velocity", leftVelocity);
    SmartDashboard.putNumber("Right velocity", rightVelocity);
    SmartDashboard.putNumber("Left voltage", leftVoltage);
    SmartDashboard.putNumber("Right voltage", rightVoltage);


    SmartDashboard.putString("Driving Mode", drive);
    Pose2d pose = getPose();
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
    SmartDashboard.putNumber("Angle", pose.getRotation().getDegrees());

  }

  public double getYaw() {
    return m_pigeon.getAngle();
  }

  private void updateOdometry() {
    double angle = getYaw();
    double leftDistance = m_leftEncoder.getPosition();
    double rightDistance = m_rightEncoder.getPosition();
    // todo: convert to radians better
    //m_odometry.update(new Rotation2d(angle / 180 * 3.141592), leftDistance, rightDistance);
    m_odometry.update(m_pigeon.getRotation2d(), leftDistance, rightDistance);

  }

  public void setupSmartDashboard() {
    SmartDashboard.putData("Reset odometry to (0, 0, 0deg)", new InstantCommand(() -> resetOdometry(new Pose2d(0, 0, new Rotation2d(Degrees.of(0))))));

    SmartDashboard.putData("Reset odometry to (0, 0, 45deg)", new InstantCommand(() -> resetOdometry(new Pose2d(0, 0, new Rotation2d(Degrees.of(45))))));
    SmartDashboard.putData("Reset odometry to (3, 0, 90deg)", new InstantCommand(() -> resetOdometry(new Pose2d(3, 0, new Rotation2d(Degrees.of(90))))));

  }

  public void setVoltages(double leftVoltage, double rightVoltage) {
    m_leftLeader.setVoltage(leftVoltage);
    m_rightLeader.setVoltage(rightVoltage);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    setSpeeds(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public void configureEncoders() {
    final double distanceScalingFactorForGearing =
        WHEEL_CIRCUMFERENCE_METERS / DRIVEBASE_GEAR_RATIO;
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;

    m_leftEncoder.setPositionConversionFactor(distanceScalingFactorForGearing);
    m_rightEncoder.setPositionConversionFactor(distanceScalingFactorForGearing);

    m_leftEncoder.setVelocityConversionFactor(velocityScalingFactor);
    m_rightEncoder.setVelocityConversionFactor(velocityScalingFactor);

    resetEncoders();
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void resetOdometry() {
    resetEncoders();
    m_pigeon.reset();
    m_odometry.resetPosition(new Rotation2d(), 0, 0, new Pose2d());
  }

  public void resetOdometry(Pose2d pose) {
    var rotation = new Rotation2d(getYaw() * Math.PI/180);
    /* 
    var left = m_leftEncoder.getPosition();
    var right = m_rightEncoder.getPosition();
    System.out.println("Rotation: " + rotation.toString() + ", left: " + left + ", right: " + right + ", pose: " + pose.toString());
    */
    //m_odometry.resetPosition(
    //    rotation, m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
    m_odometry.resetPosition(
        m_pigeon.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
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
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(fSpeed, ZERO_MPS, rSpeed)));
  }

  public void enableBreakingMode(boolean breaking) {
    CANSparkMax.IdleMode mode;
    if (breaking) {
      mode = CANSparkMax.IdleMode.kBrake;
    } else {
      mode = CANSparkMax.IdleMode.kCoast;
    }
    m_leftLeader.setIdleMode(mode);
    m_leftFollower.setIdleMode(mode);
    m_rightLeader.setIdleMode(mode);
    m_rightFollower.setIdleMode(mode);
  }

  public double getLeftVoltage() {
    return m_leftLeader.getAppliedOutput();
  }

  public double getRightVoltage() {
    return m_rightLeader.getAppliedOutput();
  }

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
 
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
      final double voltage = volts.in(Volts);
      setVoltages(voltage, voltage);
    }, log -> {
      final var leftVoltage = getLeftVoltage();
      final var rightVoltage = getRightVoltage();
      log.motor("drive-left")
      .voltage(m_appliedVoltage.mut_replace(leftVoltage, Volts))
        .linearPosition(m_distance.mut_replace(m_leftEncoder.getPosition(), Meters))
      .linearVelocity(m_velocity.mut_replace(m_leftEncoder.getVelocity(), MetersPerSecond));
      log.motor("drive-right")
      .voltage(m_appliedVoltage.mut_replace(rightVoltage, Volts))
      .linearPosition(m_distance.mut_replace(m_rightEncoder.getPosition(), Meters))
      .linearVelocity(m_velocity.mut_replace(m_rightEncoder.getVelocity(), MetersPerSecond));
    },
    this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
