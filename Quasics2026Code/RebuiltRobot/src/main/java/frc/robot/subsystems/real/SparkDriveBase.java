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
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;

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
    super(new SparkMax(SparkMaxIds.LEFT_LEADER_ID, MotorType.kBrushless),
        new SparkMax(SparkMaxIds.RIGHT_LEADER_ID, MotorType.kBrushless));

    // Configure followers to follow the leaders.
    final SparkMax leftfollower = new SparkMax(SparkMaxIds.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    final SparkMax rightfollower = new SparkMax(SparkMaxIds.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    configureMotorControllersForFollowing(
        (SparkMax) getLeftLeader(), leftfollower);
    configureMotorControllersForFollowing(
        (SparkMax) getRightLeader(), rightfollower);

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
    // FINDME(Robert): This needs to be updated, since we're Sally has a Pigeon2,
    // not an AnalogGyro.
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
   * @param leader   leader SparkMax motor controller that the follower should
   *                 follow
   * @param follower SparkMax motor controller that should be configured to
   *                 follow
   *                 the leader
   */
  private void configureMotorControllersForFollowing(
      SparkMax leader, SparkMax follower) {
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    // Configure the motor to follow the leader
    // Pass second parameter of 'true' to invert the direction
    followerConfig.follow(leader);

    // Apply the configuration to the follower motor
    follower.configure(followerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // TODO: Configure the leader so that it is *not* a follower of anything.
    //
    // FINDME(Robert): This is important to do to ensure that the leader motor
    // controllers are correctly configured even if they get swapped out. It can be
    // done with 1-2 lines of code.
  }

  // We've removed @Override periodic, but be sure to use super.periodic if we
  // ever need to resurrect it.
}
