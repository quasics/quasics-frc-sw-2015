// Copyright (c) Matt Healy, Quasics, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoHost extends SubsystemBase {
  static final int SERVO_PWM_ID = 3;
  final Servo m_servo = new Servo(SERVO_PWM_ID);

  /** Creates a new ServoHost. */
  public ServoHost() {
  }

  /**
   * @param pos position for the servo (in the range [0.0,1.0])
   */
  public void setServoPosition(double pos) {
    m_servo.set(pos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
