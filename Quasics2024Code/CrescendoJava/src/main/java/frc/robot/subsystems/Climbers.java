// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConditionalConstants;
import frc.robot.Constants.CanBusIds.SparkMax;

public class Climbers extends SubsystemBase {

  CANSparkMax m_leftClimber;
  CANSparkMax m_rightClimber;

  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;
  
  static final double EXTENSION_SPEED = 1.0;
  static final double RETRACTION_SPEED = -1.0;


  /** Creates a new Climbers. */
  public Climbers() {
    if (!ConditionalConstants.SALLY) {
      m_leftClimber = new CANSparkMax(SparkMax.LEFT_CLIMBER_ID, MotorType.kBrushless);
      m_rightClimber = new CANSparkMax(SparkMax.RIGHT_CLIMBER_ID, MotorType.kBrushless);
      m_leftEncoder = m_leftClimber.getEncoder();
      m_rightEncoder = m_rightClimber.getEncoder();

    }
  }

  public void StartExtending(){
    m_leftClimber.set(EXTENSION_SPEED);
    m_rightClimber.set(EXTENSION_SPEED);
  }

  public void StartRetracting(){
    m_leftClimber.set(RETRACTION_SPEED);
    m_rightClimber.set(RETRACTION_SPEED);
  }

  public void ExtendOneClimber(boolean isLeft){
    if(isLeft){
        m_leftClimber.set(EXTENSION_SPEED);
    }else{
      m_rightClimber.set(EXTENSION_SPEED);
    }
  }

  public void RetractOneClimber(boolean isLeft){
    if(isLeft){
        m_leftClimber.set(RETRACTION_SPEED);
    }else{
      m_rightClimber.set(RETRACTION_SPEED);
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

  public boolean IsFullyExtended(){
    if(GetLeftRevolutions() < -3 && GetRightRevolutions() < -3){
      return true;
    }
    return false;
  }

  public boolean IsFullyRetracted(){
    if(GetLeftRevolutions() >= 0  && GetRightRevolutions() >= 0){
      return true;
    }
    return false;
  }

  public double GetLeftRevolutions(){
    return m_leftEncoder.getPosition() / 42;
  }

  public double GetRightRevolutions(){
    return m_rightEncoder.getPosition() / 42;
  }

  public void ResetRevolutions(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void SetRevolutions(){
    m_leftEncoder.setPosition(-126);
    m_rightEncoder.setPosition(-126);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
