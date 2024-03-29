// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SingleMotor extends SubsystemBase {
  //CANSparkMax m_motor1 = new CANSparkMax(Constants.SINGLE_MOTOR_CAN_ID1, MotorType.kBrushless);
  //CANSparkMax m_motor2 = new CANSparkMax(Constants.SINGLE_MOTOR_CAN_ID2, MotorType.kBrushless);
  //CANSparkMax m_motor5 = new CANSparkMax(Constants.SINGLE_MOTOR_CAN_ID5, MotorType.kBrushless);
  CANSparkMax m_motor6 = new CANSparkMax(Constants.SINGLE_MOTOR_CAN_ID6, MotorType.kBrushless);
  //CANSparkMax m_motor3 = new CANSparkMax(Constants.SINGLE_MOTOR_CAN_ID3, MotorType.kBrushless);
  //CANSparkMax m_motor4 = new CANSparkMax(Constants.SINGLE_MOTOR_CAN_ID4, MotorType.kBrushless);
  /** Creates a new SingleMotor. */
  public SingleMotor() {}

  public void setPower(double percent) {
    //m_motor1.set(-percent);
    //m_motor2.set(percent);
    //m_motor5.set(-percent);
    m_motor6.set(-percent);
    //m_motor3.set(percent);
    //m_motor4.set(percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
