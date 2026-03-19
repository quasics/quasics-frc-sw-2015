// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.hardware.actuators.SparkMaxMotorControllerPlus;
import frc.robot.hardware.sensors.IGyro;
import frc.robot.hardware.sensors.Pigeon2Wrapper;
import frc.robot.hardware.sensors.SparkMaxEncoderWrapper;
import frc.robot.hardware.sensors.TrivialEncoder;

public class SparkDriveBase extends AbstractDrivebase {
  /** Track width (distance between left and right wheels) in meters. */
  public static final Distance TRACK_WIDTH = Meters.of(0.5588); /* 22 inches (from 2024) */

  private final TrivialEncoder m_leftEncoder;
  private final TrivialEncoder m_rightEncoder;

  private final IGyro m_gryo;

  @Override
  protected final IGyro getGyro() {
    return m_gryo;
  }

  @Override
  protected final TrivialEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  @Override
  protected final TrivialEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Creates a new SparkDriveBase, using default CAN IDs for the left/right
   * leaders.
   */
  public SparkDriveBase() {
    this(new SparkMax(SparkMaxIds.LEFT_LEADER_ID, MotorType.kBrushless),
        new SparkMax(SparkMaxIds.RIGHT_LEADER_ID, MotorType.kBrushless));
  }

  /**
   * Constructor.
   * 
   * @param leftLeader  left leader motor
   * @param rightLeader right leader motor
   */
  public SparkDriveBase(SparkMax leftLeader, SparkMax rightLeader) {
    super(new SparkMaxMotorControllerPlus(leftLeader),
        new SparkMaxMotorControllerPlus(rightLeader),
        TRACK_WIDTH);

    // Configure followers to follow the leaders.
    final SparkMax leftfollower = new SparkMax(SparkMaxIds.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    final SparkMax rightfollower = new SparkMax(SparkMaxIds.RIGHT_FOLLOWER_ID, MotorType.kBrushless);
    configureMotorControllersForLeadingAndFollowing(
        leftLeader, leftfollower);
    configureMotorControllersForLeadingAndFollowing(
        rightLeader, rightfollower);

    // Conversion factor from units in rotations (or RPM) to meters (or m/s).
    final Distance wheelCircumference = Constants.WHEEL_RADIUS.times(2 * Math.PI);
    final double distanceScalingFactorForGearing = wheelCircumference.div(Constants.DRIVEBASE_GEAR_RATIO).in(Meters);
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;
    System.out.println("Wheel circumference: " + wheelCircumference);
    System.out.println("Using gear ratio: " + Constants.DRIVEBASE_GEAR_RATIO);
    System.out.println("Adjustment for gearing (m/rotation): " + distanceScalingFactorForGearing);
    System.out.println("Velocity adj.: " + velocityScalingFactor);

    // Configure the leader motors
    final SparkMaxConfig leftConfig = new SparkMaxConfig();
    final SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig.encoder.positionConversionFactor(distanceScalingFactorForGearing);
    leftConfig.encoder.velocityConversionFactor(velocityScalingFactor);
    leftConfig.inverted(false);

    rightConfig.encoder.positionConversionFactor(distanceScalingFactorForGearing);
    rightConfig.encoder.velocityConversionFactor(velocityScalingFactor);
    rightConfig.inverted(true);

    leftLeader.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightLeader.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // // Configure the encoders.
    // // FINDME(Rylie, Robert): The SparkMax motors don't use WPILib Encoder
    // // objects. They use the RelativeEncoders that are specific to the Sparks. As
    // // a result, this whole section of the code was unfortunately just *wrong*.
    //
    // Encoder leftEncoder = new Encoder(1, 2);
    // Encoder rightEncoder = new Encoder(3, 4);
    // leftEncoder.setDistancePerPulse(getDistancePerPulse());
    // rightEncoder.setDistancePerPulse(getDistancePerPulse());
    //
    // // (I've replaced this with the use of the SparkMax's native encoders,
    // // below.)

    m_leftEncoder = new SparkMaxEncoderWrapper(leftLeader.getEncoder());
    m_rightEncoder = new SparkMaxEncoderWrapper(rightLeader.getEncoder());

    // Configure the gyro.
    Pigeon2 rawGyro = new Pigeon2(Constants.CanBusIds.PIGEON2_CAN_ID);
    m_gryo = new Pigeon2Wrapper(rawGyro);
  }

  /**
   * Configures a follower SparkMax motor controller to follow a leader SparkMax
   * motor controller.
   *
   * Note that this is important to do in code (instead of just setting the
   * followers to follow the leaders simply be using a configration app) to
   * ensure that the followers will be correctly configured even if a motor
   * controller gets swapped out (e.g., if a controller gets damaged and needs
   * to be replaced, or if we need to swap a controller from one side of the
   * drivebase to the other for some reason, etc.).
   *
   * @param leader   leader SparkMax motor controller (which the follower should
   *                 follow)
   * @param follower SparkMax motor controller that should be configured to
   *                 follow the leader
   */
  private void configureMotorControllersForLeadingAndFollowing(
      SparkMax leader, SparkMax follower) {
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    // Configure the motor to follow the leader
    // Pass second parameter of 'true' to invert the direction
    followerConfig.follow(leader);

    // Apply the configuration to the follower motor
    follower.configure(followerConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    // Configure the leader so that it is *not* a follower of anything (in case we
    // swap motor controllers around, and fail to set this up with the configuration
    // tool).
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.follow(0);
    leader.configure(leaderConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  // We've removed @Override periodic, but be sure to use super.periodic if we
  // ever need to resurrect it.
}
