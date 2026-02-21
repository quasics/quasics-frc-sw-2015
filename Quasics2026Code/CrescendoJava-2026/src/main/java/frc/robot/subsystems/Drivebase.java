// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import java.lang.Math;

public class Drivebase extends SubsystemBase {
  private final DifferentialDriveKinematics m_kinematics;
  private static final LinearVelocity ZERO_MPS = MetersPerSecond.of(0);

  final SparkMax m_leftLeader =
      new SparkMax(SparkMaxIds.LEFT_LEADER_ID, MotorType.kBrushless);
  final SparkMax m_leftFollower =
      new SparkMax(SparkMaxIds.LEFT_FOLLOWER_ID, MotorType.kBrushless);
  final SparkMax m_rightLeader =
      new SparkMax(SparkMaxIds.RIGHT_LEADER_ID, MotorType.kBrushless);
  final SparkMax m_rightFollower =
      new SparkMax(SparkMaxIds.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

  /** Maximum linear speed is 3 meters per second. */
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(1.0);

  /** Maximum rotational speed is 1 rotation per second. */
  public static final AngularVelocity MAX_ANGULAR_SPEED =
      RadiansPerSecond.of(Math.PI);

  public static final Distance TRACK_WIDTH_METERS = Meters.of(0.5588);

  private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

  static final double ANDYMARK_6IN_PLACTION_DIAMETER_METERS =
      Units.inchesToMeters(6.0);
  static final double WHEEL_CIRCUMFERENCE_METERS =
      Math.PI * ANDYMARK_6IN_PLACTION_DIAMETER_METERS;
  static final double DRIVEBASE_GEAR_RATIO = 8.45;

  final private DifferentialDriveOdometry m_odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d());

  private final DifferentialDrivePoseEstimator m_estimator;

  private final Pigeon2 m_pigeon =
      new Pigeon2(Constants.CanBusIds.PIGEON2_CAN_ID);

  final SparkMaxConfig m_leftFollowerConfig = new SparkMaxConfig();
  final SparkMaxConfig m_rightFollowerConfig = new SparkMaxConfig();

  final SparkMaxConfig m_leftLeaderConfig = new SparkMaxConfig();
  final SparkMaxConfig m_rightLeaderConfig = new SparkMaxConfig();

  final EncoderConfig m_encoderConfig = new EncoderConfig();

  /** Creates a new Drivebase. */
  public Drivebase() {
    m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    m_estimator = new DifferentialDrivePoseEstimator(
        m_kinematics, new Rotation2d(), 0, 0, new Pose2d());
    resetOdometry();
    setupSmartDashboard();
    configureEncoders();

    m_leftFollowerConfig.follow(SparkMaxIds.LEFT_LEADER_ID);
    m_rightFollowerConfig.follow(SparkMaxIds.RIGHT_LEADER_ID);

    m_leftFollower.configure(m_leftFollowerConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollower.configure(m_rightFollowerConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftLeader.configure(m_leftLeaderConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_rightLeader.configure(m_rightLeaderConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    updateOdometry();
    double leftDistance = m_leftEncoder.getPosition();
    double rightDistance = m_rightEncoder.getPosition();

    /*
    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putNumber("Left distance", leftDistance);
    SmartDashboard.putNumber("Right distance", rightDistance);
    SmartDashboard.putNumber("Left velocity", leftVelocity);
    SmartDashboard.putNumber("Right velocity", rightVelocity);
    SmartDashboard.putNumber("Left voltage", leftVoltage);
    SmartDashboard.putNumber("Right voltage", rightVoltage);
    */

    Pose2d pose = getPose();
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
    SmartDashboard.putNumber("Angle", pose.getRotation().getDegrees());

    m_estimator.update(m_pigeon.getRotation2d(), leftDistance, rightDistance);
  }

  public double getYaw() {
    return m_pigeon.getYaw().getValueAsDouble();
  }

  private void updateOdometry() {
    double leftDistance = m_leftEncoder.getPosition();
    double rightDistance = m_rightEncoder.getPosition();
    m_odometry.update(m_pigeon.getRotation2d(), leftDistance, rightDistance);
    m_estimator.update(m_pigeon.getRotation2d(), leftDistance, rightDistance);
    /*
    Optional<EstimatedRobotPose> result = m_vision.visionEstimator.update();
    if (result.isPresent()) {
      EstimatedRobotPose pose = result.get();
      Pose2d toPrint = pose.estimatedPose.toPose2d();
      SmartDashboard.putNumber("returned x", toPrint.getX());
      SmartDashboard.putNumber("returned y", toPrint.getY());
      SmartDashboard.putNumber("returned angle",
    toPrint.getRotation().getDegrees());
      m_estimator.addVisionMeasurement(pose.estimatedPose.toPose2d(),
    pose.timestampSeconds);

    }
    */
  }

  public void setupSmartDashboard() {
    SmartDashboard.putData("Reset odometry to (0, 0, 0deg)",
        new InstantCommand(()
                               -> resetOdometry(new Pose2d(
                                   0, 0, new Rotation2d(Degrees.of(0))))));

    SmartDashboard.putData("Reset odometry to (0, 0, 45deg)",
        new InstantCommand(()
                               -> resetOdometry(new Pose2d(
                                   0, 0, new Rotation2d(Degrees.of(45))))));
    SmartDashboard.putData("Reset odometry to (3, 0, 90deg)",
        new InstantCommand(()
                               -> resetOdometry(new Pose2d(
                                   3, 0, new Rotation2d(Degrees.of(90))))));
  }

  public void setVoltages(double leftVoltage, double rightVoltage) {
    m_leftLeader.setVoltage(leftVoltage);
    m_rightLeader.setVoltage(rightVoltage);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    setSpeeds(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
  }

  public Pose2d getPose() {
    return m_estimator.getEstimatedPosition();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public void configureEncoders() {
    final double distanceScalingFactorForGearing =
        WHEEL_CIRCUMFERENCE_METERS / DRIVEBASE_GEAR_RATIO;
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;

    m_leftLeaderConfig.encoder.positionConversionFactor(
        distanceScalingFactorForGearing);
    m_leftLeaderConfig.encoder.velocityConversionFactor(velocityScalingFactor);
    m_rightLeaderConfig.encoder.positionConversionFactor(
        distanceScalingFactorForGearing);
    m_rightLeaderConfig.encoder.velocityConversionFactor(velocityScalingFactor);
    m_leftFollowerConfig.encoder.positionConversionFactor(
        distanceScalingFactorForGearing);
    m_leftFollowerConfig.encoder.velocityConversionFactor(
        velocityScalingFactor);
    m_rightFollowerConfig.encoder.positionConversionFactor(
        distanceScalingFactorForGearing);
    m_rightFollowerConfig.encoder.velocityConversionFactor(
        velocityScalingFactor);

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
    m_estimator.resetPosition(new Rotation2d(), 0, 0, new Pose2d());
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_pigeon.getRotation2d(),
        m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
    m_estimator.resetPosition(m_pigeon.getRotation2d(), 0, 0, pose);
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

  public void arcadeDrive(LinearVelocity fSpeed, AngularVelocity rSpeed) {
    setSpeeds(m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(fSpeed, ZERO_MPS, rSpeed)));
  }

  public void enableBreakingMode(boolean breaking) {
    SparkBaseConfig.IdleMode mode;
    if (breaking) {
      mode = SparkBaseConfig.IdleMode.kBrake;
    } else {
      mode = SparkBaseConfig.IdleMode.kCoast;
    }
    m_leftLeaderConfig.idleMode(mode);
    m_leftFollowerConfig.idleMode(mode);
    m_rightLeaderConfig.idleMode(mode);
    m_rightFollowerConfig.idleMode(mode);
  }

  public double getLeftVoltage() {
    return m_leftLeader.getAppliedOutput();
  }

  public double getRightVoltage() {
    return m_rightLeader.getAppliedOutput();
  }
  /* ???
  private final MutVoltage m_appliedVoltage = mutable(Volts.of(0));
  private final MutDistance m_distance = mutable(Meters.of(0));
  private final MutLinearVelocity m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism((Voltage volts) -> {
      final double voltage = volts.in(Volts);
      setVoltages(voltage, voltage);
    }, log -> {
      final var leftVoltage = getLeftVoltage();
      final var rightVoltage = getRightVoltage();
      log.motor("drive-left")
      .voltage(m_appliedVoltage.mut_replace(leftVoltage, Volts))
        .linearPosition(m_distance.mut_replace(m_leftEncoder.getPosition(),
  Meters)) .linearVelocity(m_velocity.mut_replace(m_leftEncoder.getVelocity(),
  MetersPerSecond)); log.motor("drive-right")
      .voltage(m_appliedVoltage.mut_replace(rightVoltage, Volts))
      .linearPosition(m_distance.mut_replace(m_rightEncoder.getPosition(),
  Meters)) .linearVelocity(m_velocity.mut_replace(m_rightEncoder.getVelocity(),
  MetersPerSecond));
    },
    this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
  */
}
