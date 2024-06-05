// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConditionalConstants;
import frc.robot.Constants.CanBusIds.SparkMax;

public class AmpScorer extends SubsystemBase {
  CANSparkMax m_ampScorer;
  /** Creates a new AmpScorer. */
  public AmpScorer() {
    if (!ConditionalConstants.SALLY) {
      m_ampScorer = new CANSparkMax(SparkMax.AMP_MOTOR_ID, MotorType.kBrushless);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAmpScorerSpeed(double percentSpeed) {
    m_ampScorer.set(percentSpeed);
  }

  public void stop() {
    m_ampScorer.set(0);
  }
}
