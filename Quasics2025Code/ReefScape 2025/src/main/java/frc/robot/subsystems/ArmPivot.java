// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.RelativeEncoder;


public class ArmPivot extends SubsystemBase {
  /** Creates a new ArmPivot. */
  //TODO: choose actual values for this in cases of simulation vs real.
  Encoder encoder = new Encoder(4, 5);

  SparkMax m_pivot;

  RelativeEncoder m_encoder;

  public ArmPivot() {
    m_pivot = new SparkMax(SparkMaxIds.ARM_PIVOT_ID, MotorType.kBrushless);
    m_encoder = m_pivot.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
