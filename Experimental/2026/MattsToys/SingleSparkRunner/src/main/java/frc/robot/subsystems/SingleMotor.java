// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotor extends SubsystemBase {

  SparkMax m_controller = new SparkMax(1, MotorType.kBrushless);

  /** Creates a new SingleMotor. */
  public SingleMotor() {}

  public void stop() { m_controller.set(0); }

  public void setSpeed(double percent) { m_controller.set(percent); }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
