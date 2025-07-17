// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // Motor Controllers (replace Spark with your motor controller type, e.g.,
  // TalonFX, VictorSPX)
  private final Spark m_leftMotor =
      new Spark(0); // Assuming PWM port 0 for left motor
  private final Spark m_rightMotor =
      new Spark(1); // Assuming PWM port 1 for right motor

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Optionally invert one side of the drivetrain if your robot drives
    // backwards m_rightMotor.setInverted(true);
  }

  // Method to drive the robot with tank drive
  public void drive(double leftSpeed, double rightSpeed) {
    m_leftMotor.set(leftSpeed);
    m_rightMotor.set(rightSpeed);
  }

  // Method to stop the drivetrain
  public void stop() {
    m_leftMotor.set(0);
    m_rightMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    // Use this for any subsystem-specific periodic tasks, like updating sensor
    // data.
  }
}
