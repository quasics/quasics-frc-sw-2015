// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleSparkMaxThing extends SubsystemBase {
  protected final SparkMax m_motor;

  /** Creates a new SingleSparkMaxThing. */
  public SingleSparkMaxThing(int canId) 
  {
    m_motor = new SparkMax(canId, MotorType.kBrushless);
  }

  public void setSpeed(double percent) {
    m_motor.set(percent);
  }

  public void stop() {
    setSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
