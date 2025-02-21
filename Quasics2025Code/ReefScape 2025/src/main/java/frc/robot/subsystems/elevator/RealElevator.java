// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class RealElevator extends AbstractElevator {
  private SparkMax m_leader = new SparkMax(SparkMaxIds.LEADER_ELEVATOR_ID, MotorType.kBrushless);
  // CODE_REVIEW: If you're configuring the motors in leader/follower mode, then
  // you shouldn't need to talk to the follower at all. (It will just, well,
  // "follow the leader".) In this case, you should probably only set up the
  // leader motor as a member of the class, and (if you need to) just set up the
  // the follower in the constructor via a local variable. This will make the code
  // easier to read.
  //
  // On the other hand, if you *aren't* configuring them in leader/follower mode,
  // then you should probably avoid using variable names that imply that you are.
  private SparkMax m_follower =
      new SparkMax(SparkMaxIds.FOLLOWER_ELEVATOR_ID, MotorType.kBrushless);

  DigitalInput m_limitSwitchUp = new DigitalInput(0);
  DigitalInput m_limitSwitchDown = new DigitalInput(1);

  private RelativeEncoder m_encoder;
  private double m_referenceRotations = 0;

  private final PIDController m_pid = new PIDController(0.00, 0.00, 0.00);
  private final ElevatorFeedforward m_elevatorFeedforward =
      new ElevatorFeedforward(0.00, 0.00, 0.00); // TODO: CHANGE

  /**
   * Creates a new Elevator.
   */
  public RealElevator() {
    m_encoder = m_leader.getEncoder();

    /*
     * SparkMaxConfig m_followerConfig = new SparkMaxConfig();
     * SparkMaxConfig m_leaderConfig = new SparkMaxConfig();
     *
     * m_followerConfig.follow(m_leader, true);
     * m_follower.configure(m_followerConfig, ResetMode.kResetSafeParameters,
     * PersistMode.kPersistParameters);
     *
     * m_leaderConfig.inverted(false);
     */

    SparkMaxConfig m_followerConfig = new SparkMaxConfig();
    SparkMaxConfig m_leaderConfig = new SparkMaxConfig();

    m_followerConfig.inverted(false);
    m_leaderConfig.inverted(false);

    m_leader.configure(
        m_leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_follower.configure(
        m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setSpeed(double percentSpeed) {
    if (ableToMove()) {
      m_leader.set(percentSpeed);

      // CODE_REVIEW: OK. You're setting the follower to the negative of the leader.
      // That implies that this motor is inverted vs. the other one (which is
      // perfectly fine, and reasonably expected). So why not just set the follower to
      // inverted when you configure it in the constructor, so that you can use a
      // consistent value for the speeds?
      m_follower.set(-percentSpeed);
    }
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

  public boolean ableToMove() {
    return (m_limitSwitchUp.get() == true && getVelocity() < 0)
        || (m_limitSwitchDown.get() == true && getVelocity() > 0);
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putBoolean("Limit switch Up", m_limitSwitchUp.get());
    SmartDashboard.putBoolean("Limit switch Down", m_limitSwitchDown.get());
    if (!ableToMove()) {
      stop();
    }
  }

  protected double getRotationsForPosition(TargetPosition position) {
    switch (position) {
      case kDontCare:
        return m_encoder.getPosition();
      // TODO: Define actual values for all cases other than "don't care".
      case kBottom:
        return 0;
      case kL1:
        return 1;
      case kL2:
        return 2;
    }

    System.err.println("**** Invalid/unexpected target position: " + position);
    return 0;
  }

  public void setTargetPosition(TargetPosition position) {
    m_referenceRotations = getRotationsForPosition(position);
  }

  // this is just for testing, don't actually use this
  public void setTargetRotations(double rotations) {
    m_referenceRotations = rotations;
  }
}
