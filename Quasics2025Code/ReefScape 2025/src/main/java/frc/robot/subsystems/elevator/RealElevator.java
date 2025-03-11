// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
  DigitalInput m_limitSwitchUp = new DigitalInput(1);
  DigitalInput m_limitSwitchDown = new DigitalInput(0);

  private RelativeEncoder m_encoder;
  private TargetPosition m_targetPosition = TargetPosition.kDontCare;

  private final double VELOCITY_DEADBAND = 1;

  // TODO: Tune PID values.
  private final PIDController m_pid = new PIDController(0.15, 0.00, 0.00);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.00, 0.0, 0.00);

  /**
   * Creates a new Elevator.
   */
  public RealElevator() {
    m_encoder = m_leader.getEncoder();

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(m_leader, true);

    try (SparkMax follower = new SparkMax(SparkMaxIds.FOLLOWER_ELEVATOR_ID, MotorType.kBrushless)) {
      follower.configure(followerConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    }

    // Configure the primary (leader) motor.
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.inverted(false);
    m_leader.configure(
        leaderConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_pid.setTolerance(1, 2);
  }

  @Override
  public void setSpeed(double percentSpeed) {
    // do not use this when using pid, only for manual control
    m_targetPosition = TargetPosition.kDontCare;
    System.out.println("Calling setSpeeds: " + percentSpeed);
    if (ableToMove(percentSpeed)) {
      m_leader.set(percentSpeed);
    }
  }

  @Override
  public void setVoltage(double voltage) {
    if (ableToMove(voltage)) {
      m_leader.setVoltage(voltage);
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

  public void resetEncoders(double position) {
    m_encoder.setPosition(position);
  }

  @Override
  public double getPosition() {
    return m_encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public boolean ableToMove(double speed) {
    // CODE_REVIEW/FIXME: The following logic appears to be incorrect. Can you check
    // it?
    //
    // Context: "!" has a higher precedence than "||", so the following line is
    // evaluating the negation of "up switch is false and speed < 0" (i.e., "up
    // switch is true or speeed >= 0"), while the second part is evaluating "down
    // switch is false and speed > 0" *without* negation. This means that the two
    // are testing very different things.

    return !((m_limitSwitchUp.get() == false && speed < 0)
        || (m_limitSwitchDown.get() == false && speed > 0));
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putBoolean("Limit switch Up", m_limitSwitchUp.get());
    SmartDashboard.putBoolean("Limit switch Down", m_limitSwitchDown.get());

    SmartDashboard.putBoolean("At elevator setpoint?", m_pid.atSetpoint());

    // SmartDashboard.putBoolean("Able to move", ableToMove());
    System.out.println(m_targetPosition);

    if (!ableToMove(m_encoder.getVelocity()) && Math.abs(m_encoder.getVelocity()) > VELOCITY_DEADBAND) {
      stop();
    }

    if (m_limitSwitchDown.get()) {
      // resetEncoders();
    }
    if (m_limitSwitchUp.get()) {
      // resetEncoders(getRotationsForPosition(AbstractElevator.TargetPosition.kL2));
    }

    if (m_targetPosition != TargetPosition.kDontCare) {
      double targetRotations = getRotationsForPosition(m_targetPosition);
      double velocity = m_encoder.getVelocity();
      double pidOutput = m_pid.calculate(m_encoder.getPosition(), targetRotations);
      double feedforward = m_feedforward.calculate(velocity);

      double output = MathUtil.clamp(pidOutput + feedforward, -12, 12);

      System.out.printf(
          "PID -> pos: %.02f, set: %.02f, vel: %.02f, pidOut: %.02f, ff: %.02f, output: %.02f, atSetpoint: %b%n",
          m_encoder.getPosition(), targetRotations, velocity, pidOutput, feedforward, output, m_pid.atSetpoint());

      setVoltage(output);
    }
  }

  protected double getRotationsForPosition(TargetPosition position) {
    switch (position) {
      case kDontCare:
        return m_encoder.getPosition();
      // TODO: Define actual values for all cases other than "don't care".
      // CODE_REVIEW/FIXME: If you're getting negative distances from the encoder,
      // that suggests that the leader is actually running an inverted direction. Why
      // not swap the IDs for leader and follower, and normalize these values so that
      // they're positive? (Or negate the positions being reported?)
      case kBottom:
        return 0;
      case kL1:
        return -74;
      case kL2:
        return -145;
      case kTop:
        return -194;
    }

    System.err.println("**** Invalid/unexpected target position: " + position);
    return 0;
  }

  public void setTargetPosition(TargetPosition position) {
    m_targetPosition = position;

  }
}
