// Copyright (c) 2026, Quasics Robotics and other contributors.
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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.logging.Logger;
import frc.robot.logging.Logger.Verbosity;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.interfaces.IDrivebase;

public abstract class AbstractDrivebase extends SubsystemBase implements IDrivebase {
  // TODO: this should come from a robot config
  private static final double m_maxMotorSpeedMPS = 3;

  /** Track width (distance between left and right wheels) in meters. */
  // TODO: this should come from a robot config
  public static final Distance TRACK_WIDTH = Meters.of(0.5588); /* 22 inches (from 2024) */

  /** Kinematics calculator for the drivebase. */
  private final DifferentialDriveKinematics m_kinematics;

  // Abstract only cares about Leaders
  // subclasses will do the configuration
  private final MotorController m_leftMotor;
  private final MotorController m_rightMotor;

  private final DifferentialDrive m_robotDrive;

  private final DifferentialDriveOdometry m_odometry;
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  // FINDME(Robert): Do you need/want to be doing this for the real hardware?
  // Putting it another way, are you expecting to need to use the simulation of
  // the field shown on the dashboard during match play?
  //
  // If so, then that's fine; if not, then this might be better in
  // simulation-specific code.
  private final Field2d m_field = new Field2d();

  private final Logger m_logger = new Logger(Logger.Verbosity.Info, "AbstractDriveBase");

  /** Creates a new AbstractDrivebase. */
  public AbstractDrivebase(
      MotorController leftController, MotorController rightController) {
    m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH.in(Meters));
    m_leftMotor = leftController;
    m_rightMotor = rightController;
    m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics, new Rotation2d(), 0, 0, new Pose2d());
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed) {
    m_robotDrive.arcadeDrive(forwardspeed.magnitude(), turnspeed.magnitude());
  }

  @Override
  public void setSpeeds(double leftSpeed, double rightSpeed) {
    m_leftMotor.set(mpsToPercent(leftSpeed));
    m_rightMotor.set(mpsToPercent(rightSpeed));
    m_logger.log("Left Speed set to " + leftSpeed, Verbosity.Debug);
    m_logger.log("Right Speed set to " + rightSpeed, Verbosity.Debug);

    m_robotDrive.feed();
  }

  @Override
  public double mpsToPercent(double speed) {
    return speed / m_maxMotorSpeedMPS;
  }

  @Override
  public Pose2d getOdometryPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    getOdometry().resetPosition(getGyro().getRotation2d(), getLeftEncoder().getPosition(),
        getRightEncoder().getPosition(), pose);
    m_poseEstimator.resetPosition(getGyro().getRotation2d(), getLeftEncoder().getPosition().in(Meters),
        getRightEncoder().getPosition().in(Meters), pose);
  }

  protected static double getDistancePerPulse() {
    return 2.0 * Math.PI * Constants.wheelRadius.in(Meters) / -4096.0;
  }

  protected final DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  protected final DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  // Slight code design complexity:
  // AbstractDrivebase is going to maintain
  // complete control over leaders
  protected MotorController getLeftLeader() {
    return m_leftMotor;
  }

  protected MotorController getRightLeader() {
    return m_rightMotor;
  }

  protected abstract TrivialEncoder getLeftEncoder();

  protected abstract TrivialEncoder getRightEncoder();

  protected abstract IGyro getGyro();

  //
  // Methods from SubsystemBase
  //

  @Override
  public void periodic() {
    // Update the odometry/pose estimation
    m_odometry.update(getGyro().getRotation2d(), getLeftEncoder().getPosition()
        .in(Meters), getRightEncoder().getPosition().in(Meters));
    m_poseEstimator.update(getGyro().getRotation2d(), getLeftEncoder().getPosition().in(Meters),
        getRightEncoder().getPosition().in(Meters));

    // Update the field simulation shown on the smart dashboard
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }
}
