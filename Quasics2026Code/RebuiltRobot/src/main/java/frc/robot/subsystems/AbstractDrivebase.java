// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.DifferentialDriveAccelerationLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;

public abstract class AbstractDrivebase extends SubsystemBase {
  // TODO: this should come from a robot config
  private double m_maxMotorSpeedMPS = 3;

  /** Creates a new AbstractDrivebase. */
  public AbstractDrivebase() {
    // DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(0, 0,
    // 0);
  }

  public abstract void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed);

  protected abstract TrivialEncoder getLeftEncoder();

  protected abstract TrivialEncoder getRightEncoder();

  protected abstract IGyro getGyro();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double mpsToPercent(double speed) {
    return speed / m_maxMotorSpeedMPS;
  }
}
