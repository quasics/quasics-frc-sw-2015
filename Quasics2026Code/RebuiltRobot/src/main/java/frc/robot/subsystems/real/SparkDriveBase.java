// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.hardware.actuators.SparkMaxMotorControllerPlus;
import frc.robot.hardware.sensors.IGyro;
import frc.robot.hardware.sensors.TrivialEncoder;

public class SparkDriveBase extends AbstractDrivebase {
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

  /** Creates a new RealDrivebase. */
  public SparkDriveBase() {
    this(new SparkMax(SparkMaxIds.LEFT_LEADER_ID, MotorType.kBrushless),
        new SparkMax(SparkMaxIds.RIGHT_LEADER_ID, MotorType.kBrushless));
  }

  public SparkDriveBase(SparkMax leftLeader, SparkMax rightLeader) {
    super(new SparkMaxMotorControllerPlus(leftLeader),
        new SparkMaxMotorControllerPlus(rightLeader));

    // Configure followers to follow the leaders.
    final SparkMax leftfollower = new SparkMax(SparkMaxIds.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    final SparkMax rightfollower = new SparkMax(SparkMaxIds.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    configureMotorControllersForLeadingAndFollowing(
        leftLeader, leftfollower);
    configureMotorControllersForLeadingAndFollowing(
        rightLeader, rightfollower);

    // Configure the encoders.
    Encoder leftEncoder = new Encoder(1, 2);
    Encoder rightEncoder = new Encoder(3, 4);

    leftEncoder.setDistancePerPulse(getDistancePerPulse());
    rightEncoder.setDistancePerPulse(getDistancePerPulse());

    // TODO(DISCUSS): What about our encoders are missing information here...

    m_leftEncoder = TrivialEncoder.forWpiLibEncoder(leftEncoder);
    m_rightEncoder = TrivialEncoder.forWpiLibEncoder(rightEncoder);

    // Configure the gyro.
    //
    // TODO: Switch this to use the gyro that we're actually going to be using
    // on the test bed (Sally).
    //
    // FINDME(Robert): This needs to be updated, since Sally has a Pigeon2, not an
    // AnalogGyro.
    AnalogGyro gyro = new AnalogGyro(0);
    m_gryo = IGyro.wrapGyro(gyro);
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
    follower.configure(followerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Configure the leader so that it is *not* a follower of anything (in case we
    // swap motor controllers around, and fail to set this up with the configuration
    // tool).
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.follow(0);
    leader.configure(leaderConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // We've removed @Override periodic, but be sure to use super.periodic if we
  // ever need to resurrect it.
}
