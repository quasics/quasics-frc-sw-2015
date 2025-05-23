// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.RevSupportFunctions.configureSparkMaxEncoderForDistance;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.Pigeon2Wrapper;
import frc.robot.sensors.SparkMaxEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.abstracts.AbstractDrivebase;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Defines a version of IDrivebase that runs on live (Quasics) hardware.
 *
 * TODO: Test this!!!!
 */
public class Drivebase extends AbstractDrivebase {
  // Hardware control/sensing.
  //

  /** The gyro (a Pigeon2) on the drive base. */
  private final IGyro m_wrappedGyro = new Pigeon2Wrapper(new Pigeon2(Constants.QuasicsDrivebaseCanIds.PIGEON2_CAN_ID));

  /** Left leading motor. */
  final private SparkMax m_leftLeader = new SparkMax(Constants.QuasicsDrivebaseCanIds.LEFT_LEADER_ID,
      MotorType.kBrushless);
  /** Right leading motor. */
  final private SparkMax m_rightLeader = new SparkMax(Constants.QuasicsDrivebaseCanIds.RIGHT_LEADER_ID,
      MotorType.kBrushless);

  /** Left encoder (as a TrivialEncoder). */
  final private TrivialEncoder m_leftEncoder = new SparkMaxEncoderWrapper(m_leftLeader.getEncoder());
  /** Right encoder (as a TrivialEncoder). */
  final private TrivialEncoder m_rightEncoder = new SparkMaxEncoderWrapper(m_rightLeader.getEncoder());

  /** Odometry for the robot, purely calculated from encoders/gyro. */
  final private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      new Rotation2d(),
      0, 0,
      new Pose2d());

  /** Drivetrain pose estimator. */
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  /** Kinematics definition for this drive base. */
  private final DifferentialDriveKinematics m_kinematics;

  /**
   * Constructor.
   *
   * @param config robot configuration data
   */
  public Drivebase(RobotConfig config) {
    super(config);

    final var driveConfig = config.drive();

    final double trackWidthMeters = driveConfig.trackWidth().in(Meters);

    m_kinematics = new DifferentialDriveKinematics(trackWidthMeters);

    final SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    final SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();

    configureSparkMaxEncoderForDistance(leftLeaderConfig, driveConfig.wheelRadius(), driveConfig.gearing());
    configureSparkMaxEncoderForDistance(rightLeaderConfig, driveConfig.wheelRadius(), driveConfig.gearing());

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightLeaderConfig.inverted(true);

    m_leftLeader.configure(
        leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(
        rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure the other motors to follow their leader
    configureMotorToFollow(Constants.QuasicsDrivebaseCanIds.LEFT_FOLLOWER_ID, m_leftLeader);
    configureMotorToFollow(Constants.QuasicsDrivebaseCanIds.RIGHT_FOLLOWER_ID, m_rightLeader);

    // Set up the pose estimator
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics,
        m_wrappedGyro.getRotation2d(), m_leftEncoder.getPosition().in(Meters),
        m_rightEncoder.getPosition().in(Meters),
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Radians.convertFrom(5, Degrees)) /* stateStdDevs */,
        VecBuilder.fill(0.5, 0.5, Radians.convertFrom(30, Degrees)) /* visionMeasurementStdDevs */);
  }

  /**
   * Configures a motor (specified via CAN ID) to follow another motor.
   *
   * @param followerId CAN ID for the motor to be configured as a follower
   * @param leader     the motor that should serve as leader
   */
  static void configureMotorToFollow(int followerId, SparkMax leader) {
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(leader, true);

    try (SparkMax follower = new SparkMax(followerId, MotorType.kBrushless)) {
      follower.configure(
          followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  @Override
  public TrivialEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  @Override
  public TrivialEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  @Override
  public IGyro getGyro() {
    return IGyro.readOnlyGyro(m_wrappedGyro);
  }

  @Override
  public void setMotorVoltages(Voltage left, Voltage right) {
    m_leftLeader.setVoltage(left);
    m_rightLeader.setVoltage(right);
  }

  @Override
  public Voltage getLeftVoltage() {
    return Volts.of(m_leftLeader.getAppliedOutput());
  }

  @Override
  public Voltage getRightVoltage() {
    return Volts.of(m_rightLeader.getAppliedOutput());
  }

  @Override
  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  @Override
  protected DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  @Override
  protected DifferentialDrivePoseEstimator getPoseEstimator() {
    return m_poseEstimator;
  }
}
