// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class Elevator extends SubsystemBase {

  SparkMax m_follower;
  SparkMax m_leader;

  SparkMaxConfig m_config = new SparkMaxConfig();
  SparkMaxConfig m_followerConfig = new SparkMaxConfig();
  SparkMaxConfig m_leaderConfig = new SparkMaxConfig();

  RelativeEncoder m_encoder;

  //private final SparkClosedLoopController m_pid = m_leader.getClosedLoopController();


  /** Crea
   * tes a new Elevator. */
  public Elevator() {
    m_follower = new SparkMax(SparkMaxIds.FOLLOWER_ELEVATOR_ID, MotorType.kBrushless);
    m_leader = new SparkMax(SparkMaxIds.LEADER_ELEVATOR_ID, MotorType.kBrushless);
    m_encoder = m_leader.getEncoder();

    m_followerConfig.inverted(false);
    m_leaderConfig.inverted(false);
    m_config.closedLoop.p(0.00).i(0.00).d(0.00).velocityFF(0.00);
    m_leader.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_follower.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_leader.configure(m_leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_follower.configure(m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  
  public void setSpeed(double percentSpeed) {
    m_leader.set(percentSpeed);
    m_follower.set(-percentSpeed);
  }

  /*public void setReference(double reference) {
    m_pid.setReference(reference, ControlType.kPosition);
  }*/

  public void setVoltage(double voltage) {
    m_leader.setVoltage(voltage);
    m_leader.setVoltage(-voltage);
  }

  public void stop() {
    m_leader.set(0);
    m_follower.set(0);
  }

  public void resetEncoders() {
    m_encoder.setPosition(0);
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator encoder position", getPosition());
    SmartDashboard.putNumber("elevator encoder velocity", getVelocity());
  }
}
