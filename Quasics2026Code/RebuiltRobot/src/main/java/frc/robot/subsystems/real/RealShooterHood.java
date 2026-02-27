// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.subsystems.interfaces.IShooterHood;

public class RealShooterHood extends SubsystemBase implements IShooterHood {

  // private final Encoder m_encoder;
  // private final SparkMax m_hoodAngleAdjuster;

  /** Creates a new RealShooterHood. */
  public RealShooterHood() {

    // m_hoodAngleAdjuster = new SparkMax(SparkMaxIds.HOOD_ID,
    // MotorType.kBrushless);
    // m_encoder = new Encoder()

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
