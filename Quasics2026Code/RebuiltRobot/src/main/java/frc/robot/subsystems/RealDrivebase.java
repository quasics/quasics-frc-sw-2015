// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RealDrivebase extends AbstractDrivebase {
  private final SparkMax m_leftleader;
  private final SparkMax m_leftfollower;
  private final SparkMax m_rightleader;
  private final SparkMax m_rightfollower;

  /** Creates a new RealDrivebase. */
  public RealDrivebase() {
    // TODO: find actual SparkMax IDs, currents are placeholders.
    m_leftleader = new SparkMax(1, MotorType.kBrushless);
    m_leftfollower = new SparkMax(2, MotorType.kBrushless);
    m_rightleader = new SparkMax(3, MotorType.kBrushless);
    m_rightfollower = new SparkMax(4, MotorType.kBrushless);
    // add thriftynova support
  }

  public void arcadeDrive(float forwardspeed, float turnspeed) {
    // TODO: use diffDrive class to drive w arcade
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
