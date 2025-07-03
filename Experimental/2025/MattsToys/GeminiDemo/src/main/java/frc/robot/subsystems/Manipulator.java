// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
  private final Spark m_manipulatorMotor =
      new Spark(2); // Assuming PWM port 2 for manipulator motor

  /** Creates a new Manipulator. */
  public Manipulator() {}

  // Method to set the speed of the manipulator
  public void setSpeed(double speed) { m_manipulatorMotor.set(speed); }

  // Method to stop the manipulator
  public void stop() { m_manipulatorMotor.set(0); }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
  }
}