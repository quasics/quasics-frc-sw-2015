// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConditionalConstants;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class Shooter extends SubsystemBase {
  // left shooter wheel is leader
  // right shooter wheel is follower
  // this is in case we fix it and want to go back to leader/follower instead of
  // left right

  SparkMax m_leftShooterMotor;
  SparkMax m_rightShooterMotor;
  /** Creates a new Shooter. */
  public Shooter() {
    if (!ConditionalConstants.SALLY) {
      m_leftShooterMotor =
          new SparkMax(SparkMaxIds.LEFT_SHOOTER_ID, MotorType.kBrushless);
      m_rightShooterMotor =
          new SparkMax(SparkMaxIds.RIGHT_SHOOTER_ID, MotorType.kBrushless);
      // m_rightShooterMotor.follow(m_leftShooterMotor);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFlywheelSpeed(double percentSpeed) {
    m_leftShooterMotor.set(percentSpeed);

    m_rightShooterMotor.set(-percentSpeed);
  }

  public void stop() {
    m_leftShooterMotor.set(0);
    m_rightShooterMotor.set(0);
  }
}
