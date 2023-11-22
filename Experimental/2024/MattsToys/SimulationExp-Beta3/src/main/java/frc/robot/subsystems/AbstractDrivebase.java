// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;

public abstract class AbstractDrivebase extends SubsystemBase {
  /** Maximum linear speed is 3 meters per second. */
  public static final double MAX_SPEED = 3.0;

  /** Maximum rotational speed is 1/2 rotation per second. */
  public static final double MAX_ANGULAR_SPEED = Math.PI;

  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDController;
  private final SimpleMotorFeedforward m_feedforward;
  private final DifferentialDriveKinematics m_kinematics;

  /** Creates a new AbstractDrivebase. */
  public AbstractDrivebase(
      double trackWidthMeters, double kP, double kI, double kD, double kS, double kV) {
    m_leftPIDController = new PIDController(kP, kI, kD);
    m_rightPIDController = new PIDController(kP, kI, kD);
    m_feedforward = new SimpleMotorFeedforward(kS, kV);
    m_kinematics = new DifferentialDriveKinematics(trackWidthMeters);
  }

  /** Update the robot's odometry. */
  public void updateOdometry() {
    getOdometry().update(
        getGyro().getRotation2d(), getLeftEncoder().getPosition(), getRightEncoder().getPosition());
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return getOdometry().getPoseMeters();
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    getLeftEncoder().reset();
    getRightEncoder().reset();
    getOdometry().resetPosition(getGyro().getRotation2d(), getLeftEncoder().getPosition(),
        getRightEncoder().getPosition(), pose);
  }

  /**
   * Controls the robot using arcade drive.
   *
   * @param xSpeed the speed for the x axis (in m/s)
   * @param rot    the rotation (in radians/s)
   */
  public final void arcadeDrive(double xSpeed, double rot) {
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
  }

  /** Sets speeds to the drivetrain motors. */
  public final void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Figure out the voltages we should need at the target speeds.
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    // Figure out the deltas, based on our current speed vs. the target speeds.
    double leftOutput =
        m_leftPIDController.calculate(getLeftEncoder().getVelocity(), speeds.leftMetersPerSecond);
    double rightOutput = m_rightPIDController.calculate(
        getRightEncoder().getVelocity(), speeds.rightMetersPerSecond);

    // OK, apply those to the actual hardware.
    setMotorVoltages(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  protected abstract DifferentialDriveOdometry getOdometry();

  protected abstract TrivialEncoder getLeftEncoder();

  protected abstract TrivialEncoder getRightEncoder();

  protected abstract IGyro getGyro();

  protected abstract void setMotorVoltages(double leftVoltage, double rightVoltage);
}
