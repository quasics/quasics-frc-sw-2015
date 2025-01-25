// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class ArmRoller extends SubsystemBase {
  /** Creates a new ArmRoller. */

  SparkMax m_roller;

  RelativeEncoder m_encoder;

  public ArmRoller() {
    m_roller = new SparkMax(SparkMaxIds.ARM_ROLLER_ID, MotorType.kBrushless);
    m_encoder = m_roller.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
