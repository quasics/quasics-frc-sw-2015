// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMax;
import frc.robot.ConditionalConstants;

public class IntakeRoller extends SubsystemBase {
  CANSparkMax m_intake;

  /** Creates a new IntakeRoller. */
  public IntakeRoller() {
    if (!ConditionalConstants.SALLY) {
      m_intake = new CANSparkMax(SparkMax.INTAKE_MOTOR_ID, MotorType.kBrushless);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRollerSpeed(double percentSpeed){
    m_intake.set(-percentSpeed);
  }

  public void stop(){
    m_intake.set(0);
  }
}
