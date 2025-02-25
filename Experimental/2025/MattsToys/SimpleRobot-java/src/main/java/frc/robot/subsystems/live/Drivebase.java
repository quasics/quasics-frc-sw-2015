// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
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
import frc.robot.sensors.IGyro;
import frc.robot.sensors.Pigeon2Wrapper;
import frc.robot.sensors.SparkMaxEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.AbstractDrivebase;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Defines a version of IDrivebase that runs on live (Quasics) hardware.
 *
 * TODO: Test this!!!!
 */
public class Drivebase extends AbstractDrivebase {
  // Common CAN IDs for Quasics' robots.
  public static final int PIGEON2_CAN_ID = 1;
  public static final int LEFT_LEADER_ID = 2;
  public static final int LEFT_FOLLOWER_ID = 1;
  public static final int RIGHT_LEADER_ID = 4;
  public static final int RIGHT_FOLLOWER_ID = 3;

  // Hardware control/sensing.
  //

  // Gyro
  private final IGyro m_wrappedGyro = new Pigeon2Wrapper(new Pigeon2(PIGEON2_CAN_ID));

  // Leader motors
  final private SparkMax m_leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
  final private SparkMax m_rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);

  final private RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  final private RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

  final private TrivialEncoder m_leftTrivialEncoder = new SparkMaxEncoderWrapper(m_leftEncoder);
  final private TrivialEncoder m_rightTrivialEncoder = new SparkMaxEncoderWrapper(m_rightEncoder);

  /** Odometry for the robot, purely calculated from encoders/gyro. */
  final private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0,
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

    final double distanceScalingFactorForGearing = driveConfig.wheelRadius().div(driveConfig.gearing()).in(Meters);
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;

    leftLeaderConfig.encoder.positionConversionFactor(distanceScalingFactorForGearing)
        .velocityConversionFactor(velocityScalingFactor);
    rightLeaderConfig.encoder.positionConversionFactor(distanceScalingFactorForGearing)
        .velocityConversionFactor(velocityScalingFactor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightLeaderConfig.inverted(true);

    m_leftLeader.configure(
        leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(
        rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Make sure our encoders are zeroed out on startup.
    // m_leftEncoder.setPosition(0);
    // m_rightEncoder.setPosition(0);

    // Set up the pose estimator
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics,
        m_wrappedGyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Radians.convertFrom(5, Degrees)) /* stateStdDevs */,
        VecBuilder.fill(0.5, 0.5, Radians.convertFrom(30, Degrees)) /* visionMeasurementStdDevs */);
  }

  @Override
  public TrivialEncoder getLeftEncoder() {
    return m_leftTrivialEncoder;
  }

  @Override
  public TrivialEncoder getRightEncoder() {
    return m_rightTrivialEncoder;
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
