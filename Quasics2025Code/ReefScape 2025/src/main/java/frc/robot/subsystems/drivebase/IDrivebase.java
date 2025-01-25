// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

public abstract class IDrivebase extends SubsystemBase {
  // Max linear speed is 3 meters per second
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.0);

  // Max rotational speed is 1/2 rotations per second
  public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI);

  private static final boolean ENABLE_VOLTAGE_APPLICATON = true;

  protected static final LinearVelocity ZERO_MPS = MetersPerSecond.of(0);

  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  private final Distance m_driveBaseLengthWithBumpers;
  private final Distance m_driveBaseWidthWithBumpers;

  /** Creates a new IDrivebase. */
  public IDrivebase(RobotSettings.Robot robot) {
    this(robot.trackWidthMeters);
    super.setName(robot.name());
  }

  protected IDrivebase(Distance trackWidthMeters) {
    m_kinematics = new DifferentialDriveKinematics(trackWidthMeters);
    m_poseEstimator =
        new DifferentialDrivePoseEstimator(m_kinematics, new Rotation2d(), 0, 0, new Pose2d());

    // TODO: Move drive base dimensions into new data from the subclasses
    m_driveBaseLengthWithBumpers = Inches.of(29);
    m_driveBaseWidthWithBumpers = Inches.of(26);
  }

  public final void stop() {
    setSpeedsImpl(0, 0, false);
  }
  public final void arcadeDrive(LinearVelocity xSpeed, AngularVelocity rot) {
    if (xSpeed.gt(MAX_SPEED)) {
      xSpeed = MAX_SPEED;
    } else if (xSpeed.lt(MAX_SPEED.unaryMinus())) {
      xSpeed = MAX_SPEED.unaryMinus();
    }
    if (rot.gt(MAX_ANGULAR_SPEED)) {
      rot = MAX_ANGULAR_SPEED;
    } else if (rot.lt(MAX_ANGULAR_SPEED.unaryMinus())) {
      rot = MAX_ANGULAR_SPEED.unaryMinus();
    }

    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, ZERO_MPS, rot)));
  }

  public final void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    setSpeedsImpl(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond, false);
  }

  public final void setSpeedsImpl(
      double leftMetersPerSecond, double rightMetersPerSecond, boolean includePID) {
    /* TODO: get characterization values for new robot
     to make stabilization and deadband values*/

    // uses to be created deadband enforcer
    var leftStabilized = 0;
    var rightStablizied = 0;

    // uses to be finished feed forward (requires characterization values)
    var leftFeedforward = leftStabilized;
    var rightFeedforward = rightStablizied;

    // uses to be finished PID values
    double leftPIDOutput = includePID ? 0 : 0;
    double rightPIDOutput = includePID ? 0 : 0;

    // Applies to motors
    setSpeeds(leftMetersPerSecond, rightMetersPerSecond);
  }

  public void setMotorVoltages(double leftVoltage, double rightVoltage) {
    if (ENABLE_VOLTAGE_APPLICATON) {
      this.setMotorVoltages_HAL(leftVoltage, rightVoltage);
    }
  }

  public void setSpeeds(double leftSpeed, double rightSpeed) {
    this.setSpeeds(leftSpeed, rightSpeed);
  }

  public Distance getLengthIncludingBumpers() {
    return m_driveBaseLengthWithBumpers;
  }

  public Distance getWidthIncludingBumpers() {
    return m_driveBaseWidthWithBumpers;
  }

  final private DifferentialDriveOdometry m_odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d());

  protected final DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public void updateOdometry() {
    final Rotation2d rotation = getGyro_HAL().getRotation2d();
    final Distance leftDistanceMeters = getLeftEncoder_HAL().getPosition();
    final Distance rightDistanceMeters = getRightEncoder_HAL().getPosition();
    m_odometry.update(rotation, leftDistanceMeters.in(Meters), rightDistanceMeters.in(Meters));
    m_poseEstimator.update(rotation, leftDistanceMeters.in(Meters), rightDistanceMeters.in(Meters));
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    getLeftEncoder_HAL().reset();
    getRightEncoder_HAL().reset();
    m_odometry.resetPosition(getGyro_HAL().getRotation2d(), 0, 0, pose);
    m_poseEstimator.resetPosition(getGyro_HAL().getRotation2d(), 0, 0, pose);
  }

  @Override
  public void periodic() {
    updateOdometry();
    // This method will be called once per scheduler run
    Pose2d pose = getPose();
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
    SmartDashboard.putNumber("Pose angle", pose.getRotation().getDegrees());
  }

  protected abstract TrivialEncoder getLeftEncoder_HAL();
  protected abstract TrivialEncoder getRightEncoder_HAL();

  protected abstract IGyro getGyro_HAL();

  protected abstract void setMotorVoltages_HAL(double leftVoltage, double rightVoltage);
  protected abstract void setSpeeds_HAL(double leftSpeed, double rightSpeed);
}
