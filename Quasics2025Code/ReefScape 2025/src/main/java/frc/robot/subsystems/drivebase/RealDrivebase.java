// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;
import static edu.wpi.first.units.Units.*;
import frc.robot.Constants;
import frc.robot.Constants.CanBusIds;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.OffsetGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;
import frc.robot.sensors.SparkMaxEncoderWrapper;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;

public class RealDrivebase extends IDrivebase {

  private static final double  kV = 473;
  private static final LinearVelocity ZERO_MPS = MetersPerSecond.of(0);

  private final Pigeon2 m_rawGyro = new Pigeon2(CanBusIds.PIGEON2_CAN_ID);

  final SparkMax m_leftLeader = new SparkMax(SparkMaxIds.LEFT_LEADER_ID, MotorType.kBrushless);
  final SparkMax m_leftFollower =
    new SparkMax(SparkMaxIds.LEFT_FOLLOWER_ID, MotorType.kBrushless);
  final SparkMax m_rightLeader = new SparkMax(SparkMaxIds.RIGHT_LEADER_ID, MotorType.kBrushless);
  final SparkMax m_rightFollower =
      new SparkMax(SparkMaxIds.RIGHT_FOLLOWER_ID, MotorType.kBrushless);
  final SparkMaxConfig m_leftLeaderConfig = new SparkMaxConfig();
  final SparkMaxConfig m_rightLeaderConfig = new SparkMaxConfig();

  public static final Distance TRACK_WIDTH_METERS = Meters.of(Constants.SallyConstants.TRACK_WIDTH);
  public static final Distance WHEEL_CIRCUMFERENCE = Inches.of(6 * Math.PI);
  public static final double GEAR_RATIO = 8.45;

  private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

  private final IGyro m_IGyro = IGyro.wrapGyro(m_rawGyro);
  private final IGyro m_offsetGyro = new OffsetGyro(m_IGyro);
  private final TrivialEncoder m_leftTrivialEncoder = new SparkMaxEncoderWrapper(m_leftEncoder);
  private final TrivialEncoder m_rightTrivialEncoder = new SparkMaxEncoderWrapper(m_rightEncoder);

  private final SparkClosedLoopController  m_leftPID = m_leftLeader.getClosedLoopController();
  private final SparkClosedLoopController  m_rightPID = m_rightLeader.getClosedLoopController();

  /** Creates a new Drivebase. */
  public RealDrivebase(RobotSettings.Robot robot) {
    super(robot);

    super.setName(getClass().getSimpleName());

    final double distanceScalingFactorForGearing = WHEEL_CIRCUMFERENCE.div(GEAR_RATIO).in(Meters);
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;

    m_leftLeaderConfig.encoder.positionConversionFactor(distanceScalingFactorForGearing);
    m_leftLeaderConfig.encoder.velocityConversionFactor(velocityScalingFactor);
    m_rightLeaderConfig.encoder.positionConversionFactor(distanceScalingFactorForGearing);
    m_rightLeaderConfig.encoder.velocityConversionFactor(velocityScalingFactor);

    m_leftLeaderConfig.closedLoop.p(0).i(0).d(0).velocityFF(0.1);
    m_rightLeaderConfig.closedLoop.p(0).i(0).d(0).velocityFF(0.1);

    m_leftLeader.configure(m_leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(m_rightLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }

  @Override
  protected void setMotorVoltages_HAL(double leftVoltage, double rightVoltage) {
      m_leftLeader.setVoltage(leftVoltage);
      m_rightLeader.setVoltage(rightVoltage);
  }

  @Override
  protected void setSpeeds_HAL(double leftSpeed, double rightSpeed) {
    
    // System.out.println(leftSpeed + " " + rightSpeed);
      m_leftLeader.set(leftSpeed);
      m_rightLeader.set(rightSpeed);

  }

  @Override
  protected void setSpeeds_HAL(DifferentialDriveWheelSpeeds speeds) {
    System.out.println(speeds);
    m_leftPID.setReference(speeds.leftMetersPerSecond, ControlType.kVelocity);
    m_rightPID.setReference(speeds.rightMetersPerSecond, ControlType.kVelocity);
  }

  protected TrivialEncoder getLeftEncoder_HAL() {
    return m_leftTrivialEncoder;
  }

  protected TrivialEncoder getRightEncoder_HAL() {
    return m_rightTrivialEncoder;
  }

  protected IGyro getGyro_HAL() {
    return m_offsetGyro;
  }

  @Override
  public double getLeftDistanceMeters() {
    return m_leftEncoder.getPosition();
  }

  @Override
  public double getRightDistanceMeters() {
    return m_rightEncoder.getPosition();
  }
}
