// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class RealElevator extends AbstractElevator {
  SparkMax m_leader;
  // CODE_REVIEW: If you're configuring the motors in leader/follower mode, then
  // you shouldn't need to talk to the follower at all. (It will just, well,
  // "follow the leader".) In this case, you should probably only set up the
  // leader motor as a member of the class, and (if you need to) just set up the
  // the follower in the constructor via a local variable. This will make the code
  // easier to read.
  //
  // On the other hand, if you *aren't* configuring them in leader/follower mode,
  // then you should probably avoid using variable names that imply that you are.
  SparkMax m_follower;

  // CODE_REVIEW: These are only used in the constructor, so they should be local
  // variables there. This will make the code easier to read/maintain.
  SparkMaxConfig m_config = new SparkMaxConfig();
  SparkMaxConfig m_followerConfig = new SparkMaxConfig();
  SparkMaxConfig m_leaderConfig = new SparkMaxConfig();

  RelativeEncoder m_encoder;

  // private final SparkClosedLoopController m_pid =
  // m_leader.getClosedLoopController();

  /**
   * Creates a new Elevator.
   */
  public RealElevator() {
    m_follower = new SparkMax(SparkMaxIds.FOLLOWER_ELEVATOR_ID, MotorType.kBrushless);
    m_leader = new SparkMax(SparkMaxIds.LEADER_ELEVATOR_ID, MotorType.kBrushless);
    m_encoder = m_leader.getEncoder();

    m_followerConfig.inverted(false);
    m_leaderConfig.inverted(false);
    m_config.closedLoop.p(0.00).i(0.00).d(0.00).velocityFF(0.00);
    m_leader.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_follower.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_leader.configure(
        m_leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_follower.configure(
        m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setSpeed(double percentSpeed) {
    m_leader.set(percentSpeed);

    // CODE_REVIEW: OK. You're setting the follower to the negative of the leader.
    // That implies that this motor is inverted vs. the other one (which is
    // perfectly fine, and reasonably expected). So why not just set the follower to
    // inverted when you configure it in the constructor, so that you can use a
    // consistent value for the speeds?
    m_follower.set(-percentSpeed);
  }

  /*
   * public void setReference(double reference) {
   * m_pid.setReference(reference, ControlType.kPosition);
   * }
   */

  @Override
  public void setVoltage(double voltage) {
    m_leader.setVoltage(voltage);
    // CODE_REVIEW: Same as setSpeed(). If you're setting the follower to the
    // negative of the leader, then do you instead want to configure it as inverted?
    m_leader.setVoltage(-voltage);
  }

  @Override
  public void stop() {
    setSpeed(0);
  }

  @Override
  public void resetEncoders() {
    m_encoder.setPosition(0);
  }

  @Override
  public double getPosition() {
    return m_encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return m_encoder.getVelocity();
  }
}
