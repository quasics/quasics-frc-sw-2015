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

  // TODO: Tune PID values.
  private final PIDController m_pid = new PIDController(0.25, 0.00, 0.00);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.00, 0.00, 0.00);

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
  }

  @Override
  public void setSpeed(double percentSpeed) {
    // do not use this when using pid, only for manual control
    m_targetPosition = TargetPosition.kDontCare;
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

  @Override
  public double getPosition() {
    return m_encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public boolean ableToMove(double speed) {
    return !((m_limitSwitchUp.get() == false && speed < 0)
        || (m_limitSwitchDown.get() == false && speed > 0));
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putBoolean("Limit switch Up", m_limitSwitchUp.get());
    SmartDashboard.putBoolean("Limit switch Down", m_limitSwitchDown.get());

    // SmartDashboard.putBoolean("Able to move", ableToMove());

    if (!ableToMove(m_encoder.getVelocity())) {
      stop();
    }

    if (m_targetPosition != TargetPosition.kDontCare) {
      double targetRotations = getRotationsForPosition(m_targetPosition);
      double velocity = m_encoder.getVelocity();
      double pidOutput = m_pid.calculate(m_encoder.getPosition(), targetRotations);
      double feedforward = m_feedforward.calculate(velocity);

      double output = MathUtil.clamp(pidOutput + feedforward, -12, 12);
      if (ableToMove(output)) {
        setVoltage(output);
      } else {
        System.out.printf("ERROR: limit switch stopped PID, output voltage: %0.2f", output);
      }

      System.out.printf(
          "PID -> pos: %.02f, set: %.02f, vel: %.02f, pidOut: %.02f, ff: %.02f, output: %.02f, atSetpoint: %b%n",
          m_encoder.getPosition(), targetRotations, velocity, pidOutput, feedforward, output, m_pid.atSetpoint());
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
        return -158;
      case kL2:
        return -258;
    }

    System.err.println("**** Invalid/unexpected target position: " + position);
    return 0;
  }

  public void setTargetPosition(TargetPosition position) {
    m_targetPosition = position;

  }
}
