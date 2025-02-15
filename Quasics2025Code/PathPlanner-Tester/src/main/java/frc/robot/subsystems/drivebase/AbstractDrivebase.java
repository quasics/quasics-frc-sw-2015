// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;

public abstract class AbstractDrivebase extends SubsystemBase {
  /** Creates a new AbstractDrivebase. */

  private final LTVUnicycleController m_autoController = new LTVUnicycleController(0.02);

  // Max linear speed is 3 meters per second
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);

  // Max rotational speed i 1/2 rotation per second (pi radians per second)
  public static final AngularVelocity MAX_ANGULAR_SPEED =
      RadiansPerSecond.of(Math.PI);

  public static final LinearVelocity ZERO_MPS = MetersPerSecond.of(0);

  private final DifferentialDriveKinematics m_kinematics;

  private final Distance trackWidthMeters = Meters.of(0.5588);

  public AbstractDrivebase() {
    m_kinematics = new DifferentialDriveKinematics(trackWidthMeters);

  }

  public void followTrajectory(DifferentialSample sample){
    Pose2d pose = getPose();
  }

  public final void stop() {}

  public final void arcadeDrive(LinearVelocity fSpeed, AngularVelocity rot) {
    fSpeed =
        fSpeed.gt(MAX_SPEED.unaryMinus()) ? fSpeed : MAX_SPEED.unaryMinus();
    fSpeed = fSpeed.lt(MAX_SPEED) ? fSpeed : MAX_SPEED;
    rot = rot.gt(MAX_ANGULAR_SPEED.unaryMinus())
              ? rot
              : MAX_ANGULAR_SPEED.unaryMinus();
    rot = rot.lt(MAX_ANGULAR_SPEED) ? rot : MAX_ANGULAR_SPEED;

    setSpeed(
        m_kinematics.toWheelSpeeds(new ChassisSpeeds(fSpeed, ZERO_MPS, rot)));
  }

  public void setSpeed(DifferentialDriveWheelSpeeds speeds) {}

  public void setSpeed(double leftSpeed, double rightSpeed) {}

  final private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0,
      new Pose2d());

  protected final DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    getLeftEncoder_HAL().reset();
    getRightEncoder_HAL().reset();
    getGyro_HAL().reset();
    m_odometry.resetPosition(getGyro_HAL().getRotation2d(), 0, 0, pose);
  }

  public void updateOdometry() {
    final Rotation2d rotation = getGyro_HAL().getRotation2d();
    final Distance leftDistanceMeters = getLeftEncoder_HAL().getPosition();
    final Distance rightDistanceMeters = getRightEncoder_HAL().getPosition();
    m_odometry.update(rotation, leftDistanceMeters.in(Meters), rightDistanceMeters.in(Meters));
  }

  protected abstract TrivialEncoder getLeftEncoder_HAL();

  protected abstract TrivialEncoder getRightEncoder_HAL();

  public abstract double getLeftDistanceMeters();

  public abstract double getRightDistanceMeters();

  protected abstract IGyro getGyro_HAL();

  protected abstract void setSpeeds_HAL(DifferentialDriveWheelSpeeds speeds);

  public Distance getLeftDistance() {
    return Meters.of(getLeftDistanceMeters());
  }

  public Distance getRightDistance() {
    return Meters.of(getRightDistanceMeters());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }
}
