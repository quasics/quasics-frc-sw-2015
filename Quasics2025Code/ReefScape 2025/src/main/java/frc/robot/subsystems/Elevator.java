// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class Elevator extends SubsystemBase {

  SparkMax m_leftElevator;
  SparkMax m_rightElevator; // should these be leader follower?

  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;

  static final double EXTENSION_SPEED = -1.0;
  static final double RETRACTION_SPEED = 1.0;
  
  /** Crea
   * tes a new Elevator. */
  public Elevator() {
    m_leftElevator = new SparkMax(SparkMaxIds.LEFT_CLIMBER_ID, MotorType.kBrushless);
    m_rightElevator = new SparkMax(SparkMaxIds.RIGHT_CLIMBER_ID, MotorType.kBrushless);
    m_leftEncoder = m_leftElevator.getEncoder();
    m_rightEncoder = m_rightElevator.getEncoder();
  }

  
  public void StartExtending() {
    m_leftElevator.set(EXTENSION_SPEED);
    m_rightElevator.set(EXTENSION_SPEED);
  }

  public void StartRetracting() {
    m_leftElevator.set(RETRACTION_SPEED);
    m_rightElevator.set(RETRACTION_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
