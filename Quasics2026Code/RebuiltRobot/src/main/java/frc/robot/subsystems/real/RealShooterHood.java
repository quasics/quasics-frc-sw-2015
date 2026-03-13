// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.subsystems.interfaces.IShooterHood;

public class RealShooterHood extends SubsystemBase implements IShooterHood {
  

  private final AbsoluteEncoder m_throughBoreEncoder;
  private final SparkMax m_hood;
  

  /** Creates a new RealShooterHood. */
  public RealShooterHood() {

    m_hood = new SparkMax(SparkMaxIds.HOOD_ID, MotorType.kBrushless);
    m_throughBoreEncoder = m_hood.getAbsoluteEncoder();

    final AbsoluteEncoderConfig throughBoreConfig = new AbsoluteEncoderConfig();

  }

  @Override
  public double getCurrentAngle(){

    double currentAngleDegrees = m_throughBoreEncoder.getPosition() * 360;
    
    return currentAngleDegrees;

  }



  @Override
  public void stopHood(){

    m_hood.set(0);

  }



  @Override
  public void moveHoodIn(double speed){

    m_hood.set(Math.abs(speed));

  }



  @Override
  public void moveHoodOut(double speed){

    m_hood.set(-Math.abs(speed));

  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
