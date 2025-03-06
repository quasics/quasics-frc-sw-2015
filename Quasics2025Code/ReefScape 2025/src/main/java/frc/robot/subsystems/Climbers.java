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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class Climbers extends SubsystemBase {
  private SparkMax m_leftClimber;

  // CODE_REVIEW/FIXME: You're not doing anything with this encoder. Are you
  // planning
  // to use it later? (If not, it should be removed.)
  private RelativeEncoder m_leftEncoder;

  private final SparkMaxConfig m_config = new SparkMaxConfig();

  /** Creates a new Climbers. */
  public Climbers() {
    m_leftClimber = new SparkMax(SparkMaxIds.LEFT_CLIMBER_ID, MotorType.kBrushless);

    m_leftEncoder = m_leftClimber.getEncoder();

    m_config.idleMode(SparkBaseConfig.IdleMode.kBrake);

    m_leftClimber.configure(
        m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // negative speed = extension
  public void startExtending(double EXTENSION_SPEED) {
    m_leftClimber.set(EXTENSION_SPEED);
  }

  // positive speed = retraction
  public void startRetracting(double RETRACTION_SPEED) {
    m_leftClimber.set(RETRACTION_SPEED);
  }

  public void stop() {
    m_leftClimber.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Current", m_leftClimber.getOutputCurrent());
  }
}
