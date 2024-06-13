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
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.MutableMeasure.mutable;
import java.lang.Math;

import frc.robot.Constants.CanBusIds.SparkMax;

public class Drivebase extends SubsystemBase {
  private final DifferentialDriveKinematics m_kinematics;
  private static final Measure<Velocity<Distance>> ZERO_MPS = MetersPerSecond.of(0);


  final CANSparkMax m_leftLeader = new CANSparkMax(SparkMax.LEFT_LEADER_ID, MotorType.kBrushless);
  final CANSparkMax m_leftFollower = new CANSparkMax(SparkMax.LEFT_FOLLOWER_ID, MotorType.kBrushless);
  final CANSparkMax m_rightLeader = new CANSparkMax(SparkMax.RIGHT_LEADER_ID, MotorType.kBrushless);
  final CANSparkMax m_rightFollower = new CANSparkMax(SparkMax.RIGHT_FOLLOWER_ID, MotorType.kBrushless);
  
  /** Maximum linear speed is 3 meters per second. */
  public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(1);

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
    double rightDistance = m_rightEncoder.getPosition();
    CANSparkMax.IdleMode mode = m_leftLeader.getIdleMode();
    String drive;
    if(mode == CANSparkMax.IdleMode.kBrake){
      drive = "Breaking Mode";
    } else{
      drive = "Coasting Mode";
    }
    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putNumber("Left distance", leftDistance);
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
    m_odometry.update(new Rotation2d(angle / 180 * 3.141592), leftDistance, rightDistance);
  }

  public void setupSmartDashboard() {

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
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());

  }

  public void configureEncoders() {
    final double distanceScalingFactorForGearing = WHEEL_CIRCUMFERENCE_METERS / DRIVEBASE_GEAR_RATIO;
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
    // untested
    //System.out.println("ANGLE: " + pose.getRotation().getDegrees());
    //m_pigeon.setYaw(pose.getRotation().getDegrees() + 360);
    // ???
    // bad function
    m_odometry.resetPosition(pose.getRotation(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);

  }

  public void setSpeeds(double leftSpeed, double rightSpeed) {
    // clamp speeds between -1 and 1
    leftSpeed = leftSpeed > -1 ? leftSpeed : -1;
    leftSpeed = leftSpeed < 1 ? leftSpeed : 1;
    rightSpeed = rightSpeed > -1 ? rightSpeed : -1;
    rightSpeed = rightSpeed < 1 ? rightSpeed : 1;

    System.out.println("Left speed: " + leftSpeed + ", Right sped: " + rightSpeed);

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

  public void enableBreakingMode(boolean breaking){
    CANSparkMax.IdleMode mode;
    if(breaking){
      mode = CANSparkMax.IdleMode.kBrake;
    } else {
      mode = CANSparkMax.IdleMode.kCoast;
    }
    m_leftLeader.setIdleMode(mode);
    m_leftFollower.setIdleMode(mode);
    m_rightLeader.setIdleMode(mode);
    m_rightFollower.setIdleMode(mode);
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
      log.motor("drive-left")
      .voltage(m_appliedVoltage.mut_replace(
        m_leftLeader.get() * RobotController.getBatteryVoltage(), Volts))
        .linearPosition(m_distance.mut_replace(m_leftEncoder.getPosition(), Meters))
        .linearVelocity(m_velocity.mut_replace(m_leftEncoder.getVelocity(), MetersPerSecond));
      log.motor("drive-right")
      .voltage(m_appliedVoltage.mut_replace(m_rightLeader.get() * RobotController.getBatteryVoltage(), Volts))
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
