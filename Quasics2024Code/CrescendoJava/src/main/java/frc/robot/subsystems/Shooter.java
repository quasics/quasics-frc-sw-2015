// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMax;

public class Shooter extends SubsystemBase {
  //left shooter wheel is leader
  //right shooter wheel is follower
  //this is in case we fix it and want to go back to leader/follower instead of left right

  final CANSparkMax m_leftShooterMotor = new CANSparkMax(SparkMax.LEFT_SHOOTER_ID, MotorType.kBrushless);
  final CANSparkMax m_rightShooterMotor = new CANSparkMax(SparkMax.RIGHT_SHOOTER_ID, MotorType.kBrushless);
  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setSpeeds(double percentSpeed){
    m_leftShooterMotor.set(percentSpeed);
    m_rightShooterMotor.set(percentSpeed);
  }
}
