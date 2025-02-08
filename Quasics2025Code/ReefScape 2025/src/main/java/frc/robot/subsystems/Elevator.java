// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class Elevator extends SubsystemBase {

  SparkMax m_follower;
  SparkMax m_leader;

  SparkMaxConfig m_followerConfig = new SparkMaxConfig();
  SparkMaxConfig m_leaderConfig = new SparkMaxConfig();

  RelativeEncoder m_encoder;

  /** Crea
   * tes a new Elevator. */
  public Elevator() {
    m_follower = new SparkMax(SparkMaxIds.FOLLOWER_ELEVATOR_ID, MotorType.kBrushless);
    m_leader = new SparkMax(SparkMaxIds.LEADER_ELEVATOR_ID, MotorType.kBrushless);
    m_encoder = m_leader.getEncoder();

    //m_followerConfig.follow(m_leader, true);
    //m_follower.configure(m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  
  public void setSpeed(double percentSpeed) {
    m_leader.set(percentSpeed);
    m_follower.set(-percentSpeed);
  }

  public void stop() {
    m_leader.set(0);
    m_follower.set(0);
  }

  public void resetEncoders() {
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator encoder position", m_encoder.getPosition());
    SmartDashboard.putNumber("elevator encoder velocity", m_encoder.getVelocity());
  }
}
