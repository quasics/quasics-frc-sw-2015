// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMax;

public class Climbers extends SubsystemBase {

  final CANSparkMax m_leftClimber = new CANSparkMax(SparkMax.LEFT_CLIMBER_ID, MotorType.kBrushless);
  final CANSparkMax m_rightClimber = new CANSparkMax(SparkMax.RIGHT_CLIMBER_ID, MotorType.kBrushless);


  /** Creates a new Climbers. */
  public Climbers() {}

  public void StartExtending(){
    m_leftClimber.set(1.00);
    m_rightClimber.set(1.00);
  }

  public void StartRetracting(){
    m_leftClimber.set(-1.00);
    m_rightClimber.set(-1.00);
  }

  public void ExtendOneClimber(boolean isLeft){
    if(isLeft){
        m_leftClimber.set(1.00);
    }else{
      m_rightClimber.set(1.00);
    }
  }

  public void RetractOneClimber(boolean isLeft){
    if(isLeft){
        m_leftClimber.set(-1.00);
    }else{
      m_rightClimber.set(-1.00);
    }
  }

  public void stop(){
    m_leftClimber.stopMotor();
    m_rightClimber.stopMotor();
  }


  public void EnableBraking(boolean putInBreak){
    if(putInBreak){
      m_leftClimber.setIdleMode(IdleMode.kBrake);
      m_rightClimber.setIdleMode(IdleMode.kBrake);
    }else{
      m_leftClimber.setIdleMode(IdleMode.kCoast);
      m_rightClimber.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
