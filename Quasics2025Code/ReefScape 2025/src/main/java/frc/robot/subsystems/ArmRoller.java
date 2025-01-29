// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRoller extends SubsystemBase {
  /** Creates a new ArmRoller. */
  
  TalonFX m_kraken = new TalonFX(0);

  public ArmRoller() {
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
