// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RealDrivebase extends AbstractDrivebase {
  private SparkMax m_leftleader;
  private SparkMax m_leftfollower;
  private SparkMax m_rightleader;
  private SparkMax m_rightfollower;
  private DifferentialDrive m_robotDrive;

  /** Creates a new RealDrivebase. */
  public RealDrivebase() {
    // TODO: find actual SparkMax IDs, currents are placeholders.
    m_leftleader = new SparkMax(1, MotorType.kBrushless);
    m_leftfollower = new SparkMax(2, MotorType.kBrushless);
    m_rightleader = new SparkMax(3, MotorType.kBrushless);
    m_rightfollower = new SparkMax(4, MotorType.kBrushless);
    m_robotDrive = new DifferentialDrive(m_leftleader, m_rightleader);
    // add thriftynova support
  }

  public void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed) {
    // TODO: use diffDrive class to drive w arcade
    m_robotDrive.arcadeDrive(forwardspeed.magnitude(), turnspeed.magnitude());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
