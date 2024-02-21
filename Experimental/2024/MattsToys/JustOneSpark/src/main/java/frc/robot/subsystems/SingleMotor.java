// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static java.lang.Math.max;
import static java.lang.Math.min;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import java.util.List;
import java.util.LinkedList;

/**
 * Will drive one (or more) motors for testing purposes.
 */
public class SingleMotor extends SubsystemBase {
  // Motor set: anything added to this list gets controlled.
  List<CANSparkMax> m_motors = new LinkedList<CANSparkMax>();

  /** Creates a new SingleMotor subsystem. */
  public SingleMotor() {
    // Left motors
    m_motors.add(new CANSparkMax(1, MotorType.kBrushless));
    m_motors.add(new CANSparkMax(2, MotorType.kBrushless));

    // Right motors
    m_motors.add(new CANSparkMax(3, MotorType.kBrushless));
    m_motors.add(new CANSparkMax(4, MotorType.kBrushless));
  }

  /**
   * Sets power for the motor(s).
   * 
   * @param percent % power, in the range [-1.0, +1.0]
   */
  public void setPower(double percent) {
    percent = min(1.0, max(-1.0, percent));
    for (var motor : m_motors) {
      motor.set(percent);
    }
  }
}
