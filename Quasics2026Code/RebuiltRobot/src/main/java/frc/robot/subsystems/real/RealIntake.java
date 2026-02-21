// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.subsystems.interfaces.IIntake;


public class RealIntake extends SubsystemBase implements IIntake {

  private final SparkMax m_roller;
  private final SparkMax m_leftExtender;
  private final SparkMax m_rightExtender;

  /** Creates a new RealIntake. */
  public RealIntake() {

    m_roller = new SparkMax(SparkMaxIds.INTAKE_ROLLERS_ID, MotorType.kBrushless);
    m_leftExtender = new SparkMax(SparkMaxIds.LEFT_INTAKE_DEPLOYMENT_ID, MotorType.kBrushless);
    m_rightExtender = new SparkMax(SparkMaxIds.RIGHT_INTAKE_DEPLOYMENT_ID, MotorType.kBrushless);

  }


  @Override
  public void setRollerSpeed(double speed){

    m_roller.set(speed);

  }


  @Override
  public void stopRoller(){

    m_roller.set(0);

  }

  public double getRollerSpeed(){

    double speed = m_roller.get();
    return speed;

  }



  @Override
  public void setExtensionSpeed(double speed){

    m_leftExtender.set(speed);
    m_rightExtender.set(-speed);

  }


  @Override
  public void stopExtension(){

    m_leftExtender.set(0);
    m_rightExtender.set(0);

  }


  public double getExtensionSpeed(){

    double speed = m_leftExtender.get();
    return speed;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
