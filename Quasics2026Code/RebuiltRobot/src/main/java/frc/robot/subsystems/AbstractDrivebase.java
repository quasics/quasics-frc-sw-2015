// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;

public abstract class AbstractDrivebase extends SubsystemBase {
  // TODO: this should come from a robot config
  private double m_maxMotorSpeedMPS = 3;
  private DifferentialDriveOdometry m_odometry;
  private DifferentialDriveKinematics m_kinematics;
  private final Field2d m_field = new Field2d();

  /** Creates a new AbstractDrivebase. */
  public AbstractDrivebase() {
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
    SmartDashboard.putData("Field", m_field);
  }

  public abstract void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed);

  protected abstract TrivialEncoder getLeftEncoder();

  protected abstract TrivialEncoder getRightEncoder();

  protected abstract IGyro getGyro();

  protected final DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  @Override
  public void periodic() {
    m_odometry.update(getGyro().getRotation2d(), getLeftEncoder().getPosition()
        .in(Meters), getRightEncoder().getPosition().in(Meters));
    m_field.setRobotPose(m_odometry.getPoseMeters());
    // This method will be called once per scheduler run
  }

  public double mpsToPercent(double speed) {
    return speed / m_maxMotorSpeedMPS;
  }
}
