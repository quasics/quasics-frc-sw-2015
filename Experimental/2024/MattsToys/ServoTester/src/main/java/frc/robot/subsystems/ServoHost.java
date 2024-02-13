// Copyright (c) Matt Healy, Quasics, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoHost extends SubsystemBase {
  static final int SERVO_PWM_ID = 3;
  public static final double FULLY_OPEN = +1.0;
  public static final double FULLY_CLOSED = -1.0;

  /**
   * Assumed to be an AndyMark Linear Servo Actuator L16-R 140 mm Stroke 35:1 6v.
   * 
   * @see https://www.andymark.com/products/actuator-l16-r-140mm-stroke-35-1-6v
   * @see http://tinyurl.com/45t2ffyc
   * @see https://www.chiefdelphi.com/t/anyone-have-servo-example-code/155951/9
   */
  final Servo m_servo = new Servo(SERVO_PWM_ID);

  /** Creates a new ServoHost. */
  public ServoHost() {
    // Configure the boundaries for this device, which are different from a
    // bog-standard servo.
    m_servo.setBoundsMicroseconds(
        2000, // max (per docs): 2.0 msec
        1800, // deadbandMax (per docs): 1.8 msec
        1500, // center (per docs): 1.5 msec
        1200, // deadbandMin (per docs): 1.2 msec
        1000 // min (per docs): 1.0 msec
    );

    fullyRetract();
  }

  public void fullyRetract() {
    m_servo.setSpeed(FULLY_CLOSED);
  }

  public void fullyExtend() {
    m_servo.setSpeed(FULLY_OPEN);
  }

  /**
   * @param pos position for the servo (in the range [0.0,1.0])
   */
  public void setServoPosition(double pos) {
    m_servo.setSpeed(Math.min(Math.max(FULLY_CLOSED, pos), FULLY_OPEN));
  }

  @Override
  public void periodic() {
    System.out.println("Servo position: " + m_servo.getSpeed());
  }
}
