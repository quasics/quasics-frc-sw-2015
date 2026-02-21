// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.logging.Logger;
import frc.robot.logging.Logger.Verbosity;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.interfaces.IDrivebase;
import java.util.function.Supplier;

public abstract class AbstractDrivebase
    extends SubsystemBase implements IDrivebase {
  // TODO: this should come from a robot config
  private static final double m_maxMotorSpeedMPS = 3;

  /** Track width (distance between left and right wheels) in meters. */
  // TODO: this should come from a robot config
  public static final Distance TRACK_WIDTH =
      Meters.of(0.5588); /* 22 inches (from 2024) */

  /** Kinematics calculator for the drivebase. */
  private final DifferentialDriveKinematics m_kinematics;

  // Abstract only cares about Leaders
  // subclasses will do the configuration
  private final MotorController m_leftMotor;
  private final MotorController m_rightMotor;

  private final DifferentialDrive m_robotDrive;

  private final DifferentialDriveOdometry m_odometry;
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  // Thoughts on this from Robert: Might be helpful for driveteam to have a
  // backup of being able to see the field display sometimes, so leave field
  // implemented for all cases
  private final Field2d m_field = new Field2d();

  private final Logger m_logger =
      new Logger(Logger.Verbosity.Info, "AbstractDriveBase");

  /** Creates a new AbstractDrivebase. */
  public AbstractDrivebase(
      MotorController leftController, MotorController rightController) {
    m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH.in(Meters));
    m_leftMotor = leftController;
    m_rightMotor = rightController;
    m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
    m_poseEstimator = new DifferentialDrivePoseEstimator(
        m_kinematics, new Rotation2d(), 0, 0, new Pose2d());
    SmartDashboard.putData("Field", m_field);
  }

  public static LinearVelocity getMaxMotorLinearSpeed() {
    return MetersPerSecond.of(m_maxMotorSpeedMPS);
  }

  public static AngularVelocity getMaxMotorTurnSpeed() {
    return DegreesPerSecond.of(m_maxMotorSpeedMPS);
  }

  @Override
  public void arcadeDrive(
      LinearVelocity forwardspeed, AngularVelocity turnspeed) {
    m_robotDrive.arcadeDrive(forwardspeed.magnitude(), turnspeed.magnitude());
  }

  @Override
  public void setSpeeds(LinearVelocity leftSpeed, LinearVelocity rightSpeed) {
    m_leftMotor.set(mpsToPercent(leftSpeed));
    m_rightMotor.set(mpsToPercent(rightSpeed));
    m_logger.log("Left Speed set to " + leftSpeed, Verbosity.Debug);
    m_logger.log("Right Speed set to " + rightSpeed, Verbosity.Debug);

    m_robotDrive.feed();
  }

  @Override
  public void setPercent(double leftPercent, double rightPercent) {
    m_leftMotor.set(leftPercent);
    m_rightMotor.set(rightPercent);

    m_robotDrive.feed();
  }

  @Override
  public void setVoltages(Voltage leftVoltage, Voltage rightVoltage) {
    // TODO(Robert): Implement this method (and then use it for
    // characterization, trajectory following, etc.). (It should be pretty
    // straightforward, doable with 2 lines of code.)
  }

  @Override
  public double mpsToPercent(LinearVelocity speed) {
    // TODO(ROBERT): Cap this - it shouldn't be greater than max speed.
    // Probably print a warning too so that we can fix whatever is commanding us
    // too high.
    return speed.in(MetersPerSecond) / m_maxMotorSpeedMPS;
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
    getOdometry().resetPosition(getGyro().getRotation2d(),
        getLeftEncoder().getPosition(), getRightEncoder().getPosition(), pose);
    m_poseEstimator.resetPosition(getGyro().getRotation2d(),
        getLeftEncoder().getPosition().in(Meters),
        getRightEncoder().getPosition().in(Meters), pose);
  }

  private Supplier<Pose2d> m_referencePositionSupplier = null;

  @Override
  public void setReferencePositionSupplier(Supplier<Pose2d> supplier) {
    m_referencePositionSupplier = supplier;
  }

  protected Pose2d getVisionPose() {
    if (m_referencePositionSupplier != null) {
      return m_referencePositionSupplier.get();
    } else {
      return null;
    }
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
    m_odometry.update(getGyro().getRotation2d(),
        getLeftEncoder().getPosition().in(Meters),
        getRightEncoder().getPosition().in(Meters));
    m_poseEstimator.update(getGyro().getRotation2d(),
        getLeftEncoder().getPosition().in(Meters),
        getRightEncoder().getPosition().in(Meters));

    // Update the field simulation shown on the smart dashboard
    m_field.setRobotPose(m_odometry.getPoseMeters());
    if (getVisionPose() != null) {
      // m_poseEstimator.addVisionMeasurement(getVisionPose(),
      // Timer.getFPGATimestamp());
    }
  }
}
