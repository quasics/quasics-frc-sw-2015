// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;

public abstract class AbstractDrivebase extends SubsystemBase {
  // TODO: this should come from a robot config
  private static final double m_maxMotorSpeedMPS = 3;

  /** Track width (distance between left and right wheels) in meters. */
  // TODO: this should come from a robot config
  public static final Distance TRACK_WIDTH = Meters.of(0.5588); /* 22 inches (from 2024) */

  /** Kinematics calculator for the drivebase. */
  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH.in(Meters));

  // abstract only cares abt left and right, subclasses will need leader/follower
  // specification
  protected MotorController m_leftMotor;
  protected MotorController m_rightMotor;

  private DifferentialDriveOdometry m_odometry;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final Field2d m_field = new Field2d();

  /** Creates a new AbstractDrivebase. */
  public AbstractDrivebase(MotorController leftController, MotorController rightController) {
    m_leftMotor = leftController;
    m_rightMotor = rightController;

    m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics, new Rotation2d(), 0, 0, new Pose2d());
    SmartDashboard.putData("Field", m_field);
  }

  public abstract void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed);

  protected abstract TrivialEncoder getLeftEncoder();

  protected abstract TrivialEncoder getRightEncoder();

  protected abstract IGyro getGyro();

  protected final DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  protected final DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  @Override
  public void periodic() {
    m_odometry.update(getGyro().getRotation2d(), getLeftEncoder().getPosition()
        .in(Meters), getRightEncoder().getPosition().in(Meters));
    m_field.setRobotPose(m_odometry.getPoseMeters());
    m_poseEstimator.update(getGyro().getRotation2d(), getLeftEncoder().getPosition().in(Meters),
        getRightEncoder().getPosition().in(Meters));
    // This method will be called once per scheduler run

  }

  protected void setSpeeds(double leftSpeed, double rightSpeed) {
    m_leftMotor.set(mpsToPercent(leftSpeed));
    m_rightMotor.set(mpsToPercent(rightSpeed));
  }

  public double mpsToPercent(double speed) {
    return speed / m_maxMotorSpeedMPS;
  }

  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    getOdometry().resetPosition(getGyro().getRotation2d(), getLeftEncoder().getPosition(),
        getRightEncoder().getPosition(), pose);
    m_poseEstimator.resetPosition(getGyro().getRotation2d(), getLeftEncoder().getPosition().in(Meters),
        getRightEncoder().getPosition().in(Meters), pose);
  }
}
