// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class Climbers extends SubsystemBase {
  /** Creates a new Climbers. */
  SparkMax m_leftClimber;
  SparkMax m_rightClimber;

  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;

  final SparkMaxConfig m_config = new SparkMaxConfig();

  static final double EXTENSION_SPEED = -0.10;
  static final double RETRACTION_SPEED = 0.10;

  public Climbers() {
    m_leftClimber = new SparkMax(SparkMaxIds.LEFT_CLIMBER_ID, MotorType.kBrushless);
    m_rightClimber = new SparkMax(SparkMaxIds.RIGHT_CLIMBER_ID, MotorType.kBrushless);

    m_leftEncoder = m_leftClimber.getEncoder();
    m_rightEncoder = m_rightClimber.getEncoder();

    m_config.idleMode(SparkBaseConfig.IdleMode.kBrake);

    m_leftClimber.configure(
        m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightClimber.configure(
        m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void StartExtending() {
    m_leftClimber.set(EXTENSION_SPEED);
    m_rightClimber.set(EXTENSION_SPEED);
  }

  public void StartRetracting() {
    m_leftClimber.set(RETRACTION_SPEED);
    m_rightClimber.set(RETRACTION_SPEED);
  }

  public void stop() {
    m_leftClimber.stopMotor();
    m_rightClimber.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
