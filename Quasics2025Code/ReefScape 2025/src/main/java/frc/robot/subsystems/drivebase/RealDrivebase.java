// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanBusIds;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.OffsetGyro;
import frc.robot.sensors.SparkMaxEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

public class RealDrivebase extends IDrivebase {
  private static final LinearVelocity ZERO_MPS = MetersPerSecond.of(0);

  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(1.0);
  public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI);

  private final Pigeon2 m_rawGyro = new Pigeon2(CanBusIds.PIGEON2_CAN_ID);

  final SparkMax m_leftLeader = new SparkMax(SparkMaxIds.LEFT_LEADER_ID, MotorType.kBrushless);
  final SparkMax m_leftFollower = new SparkMax(SparkMaxIds.LEFT_FOLLOWER_ID, MotorType.kBrushless);
  final SparkMax m_rightLeader = new SparkMax(SparkMaxIds.RIGHT_LEADER_ID, MotorType.kBrushless);
  final SparkMax m_rightFollower =
      new SparkMax(SparkMaxIds.RIGHT_FOLLOWER_ID, MotorType.kBrushless);
  final SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
  final SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

  // change this
  public static final Distance TRACK_WIDTH_METERS = Meters.of(Constants.SallyConstants.TRACK_WIDTH);

  private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

  private final IGyro m_IGyro = IGyro.wrapGyro(m_rawGyro);
  private final IGyro m_offsetGyro = new OffsetGyro(m_IGyro);
  private final TrivialEncoder m_leftTrivialEncoder = new SparkMaxEncoderWrapper(m_leftEncoder);
  private final TrivialEncoder m_rightTrivialEncoder = new SparkMaxEncoderWrapper(m_rightEncoder);

  /** Creates a new Drivebase. */
  public RealDrivebase(RobotSettings.Robot robot) {
    super(robot);

    super.setName(getClass().getSimpleName());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected void setMotorVoltages_HAL(double leftVoltage, double rightVoltage) {
    m_leftLeader.set(leftVoltage);
    m_rightLeader.set(rightVoltage);
  }

  @Override
  protected void setSpeeds_HAL(double leftSpeed, double rightSpeed) {
    m_leftLeader.set(leftSpeed);
    m_rightLeader.set(rightSpeed);
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
}
