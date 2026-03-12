// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.hardware.actuators.IMotorControllerPlus;
import frc.robot.hardware.sensors.IGyro;
import frc.robot.hardware.sensors.TrivialEncoder;
import frc.robot.logging.Logger;
import frc.robot.logging.Logger.Verbosity;
import frc.robot.subsystems.interfaces.IDrivebase;
import java.util.function.Supplier;

public abstract class AbstractDrivebase
    extends SubsystemBase implements IDrivebase {
  /** Track width (distance between left and right wheels) in meters. */
  // TODO: this should come from a robot config
  public static final Distance TRACK_WIDTH = Meters.of(0.5588); /* 22 inches (from 2024) */

  /** Kinematics calculator for the drivebase. */
  private final DifferentialDriveKinematics m_kinematics;

  // Abstract only cares about Leaders
  // subclasses will do the configuration
  private final IMotorControllerPlus m_leftMotor;
  private final IMotorControllerPlus m_rightMotor;

  // FINDME(Rylie, Robert): Do we really _need_ to use a DifferentialDrive
  // object? Doing so means that we *must* ensure that it's "fed" with updated
  // values for left and right regularly, or else it can (potentially)
  // error-out.
  private final DifferentialDrive m_robotDrive;

  private DifferentialDriveOdometry m_odometry;
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  // Thoughts on this from Robert: Might be helpful for driveteam to have a
  // backup of being able to see the field display sometimes, so leave field
  // implemented for all cases
  private final Field2d m_field = new Field2d();

  private final Logger m_logger = new Logger(Logger.Verbosity.Info, "AbstractDriveBase");

  // sim bot
  final protected PIDController m_leftPidController = new PIDController(0.0, 0.0, 0.0);
  final protected PIDController m_rightPidController = new PIDController(0.0, 0.0, 0.0);
  final protected DifferentialDriveFeedforward m_feedforward = new DifferentialDriveFeedforward(3.5375, 0.19759, 3.5375,
      0.051438);

  /** Creates a new AbstractDrivebase. */
  public AbstractDrivebase(IMotorControllerPlus leftController,
      IMotorControllerPlus rightController) {
    m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH.in(Meters));
    m_leftMotor = leftController;
    m_rightMotor = rightController;
    m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
    m_poseEstimator = new DifferentialDrivePoseEstimator(
        m_kinematics, new Rotation2d(), 0, 0, new Pose2d());
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void updateStartingPosition(Pose2d pose) {
    getGyro().reset();
    getLeftEncoder().reset();
    getRightEncoder().reset();

    m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0, pose);
  }

  public void drivePID(ChassisSpeeds chassisSpeeds) {
    DifferentialDriveWheelSpeeds speeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
    double leftPidOutput = m_leftPidController.calculate(
        getLeftEncoder().getVelocity().in(MetersPerSecond),
        speeds.leftMetersPerSecond);
    double rightPidOutput = m_leftPidController.calculate(
        getRightEncoder().getVelocity().in(MetersPerSecond),
        speeds.rightMetersPerSecond);
    var feedforward = m_feedforward.calculate(
        getLeftEncoder().getVelocity().in(MetersPerSecond),
        speeds.leftMetersPerSecond,
        getRightEncoder().getVelocity().in(MetersPerSecond),
        speeds.rightMetersPerSecond, 0.02);
    setVoltages(
        leftPidOutput + feedforward.left, rightPidOutput + feedforward.right);
  }

  public static LinearVelocity getMaxMotorLinearSpeed() {
    return Constants.MAX_LINEAR_DRIVE_SPEED;
  }

  public static AngularVelocity getMaxMotorTurnSpeed() {
    return Constants.MAX_ROTATIONAL_SPEED;
  }

  public SysIdRoutine getSysIdRoutine(IDrivebase drivebase, Mode mode) {
    return new SysIdRoutine(new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (volts) -> this.setVoltages(
                volts, volts.times(mode == Mode.Linear ? 1 : -1)),
            log -> {
              final var leftVoltage = getLeftVoltage();
              final var leftPosition = getLeftEncoder().getPosition();
              final var leftVelocity = getLeftEncoder().getVelocity();
              final var rightVoltage = getRightVoltage();
              final var rightPosition = getRightEncoder().getPosition();
              final var rightVelocity = getRightEncoder().getVelocity();

              log.motor("drive-left")
                  .voltage(leftVoltage)
                  .linearPosition(leftPosition)
                  .linearVelocity(leftVelocity);
              log.motor("drive-right")
                  .voltage(rightVoltage)
                  .linearPosition(rightPosition)
                  .linearVelocity(rightVelocity);
            },
            this));
  }

  @Override
  public Command sysIdQuasistatic(
      IDrivebase drivebase, Mode mode, SysIdRoutine.Direction direction) {
    return getSysIdRoutine(drivebase, mode).quasistatic(direction);
  }

  @Override
  public Command sysIdDynamic(IDrivebase drivebase, IDrivebase.Mode mode,
      SysIdRoutine.Direction direction) {
    return getSysIdRoutine(drivebase, mode).dynamic(direction);
  }

  protected static AngularVelocity getCappedTurnSpeed(
      AngularVelocity turnSpeed) {
    return DegreesPerSecond.of(MathUtil.clamp(turnSpeed.in(DegreesPerSecond),
        -Constants.MAX_ROTATIONAL_SPEED.in(DegreesPerSecond),
        Constants.MAX_ROTATIONAL_SPEED.in(DegreesPerSecond)));
  }

  protected static LinearVelocity getCappedLinearSpeed(
      LinearVelocity linearSpeed) {
    return MetersPerSecond.of(MathUtil.clamp(linearSpeed.in(MetersPerSecond),
        -Constants.MAX_LINEAR_DRIVE_SPEED.in(MetersPerSecond),
        Constants.MAX_LINEAR_DRIVE_SPEED.in(MetersPerSecond)));
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

  protected Voltage getLeftVoltage() {
    return m_leftMotor.getVoltage();
  }

  protected Voltage getRightVoltage() {
    return m_rightMotor.getVoltage();
  }

  @Override
  public void setVoltages(Voltage leftVoltage, Voltage rightVoltage) {
    m_leftMotor.setVoltage(leftVoltage);
    m_rightMotor.setVoltage(rightVoltage);

    m_robotDrive.feed();
  }

  public void setVoltages(double leftVoltage, double rightVoltage) {
    m_leftMotor.setVoltage(leftVoltage);
    m_rightMotor.setVoltage(rightVoltage);

    m_robotDrive.feed();
  }

  @Override
  public double mpsToPercent(LinearVelocity speed) {
    // TODO(ROBERT): Cap this - it shouldn't be greater than max speed.
    // Probably print a warning too so that we can fix whatever is commanding us
    // too high.
    return speed.in(MetersPerSecond)
        / Constants.MAX_LINEAR_DRIVE_SPEED.in(MetersPerSecond);
  }

  @Override
  public double getHeading() {
    return getGyro().getRotation2d().getDegrees();
  }

  @Override
  public AngularVelocity getTurnRate() {
    return getGyro().getRate();
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

  @Override
  public Distance getLeftDistance() {
    return getLeftEncoder().getPosition();
  }

  @Override
  public Distance getRightDistance() {
    return getRightEncoder().getPosition();
  }

  @Override
  public double getLeftRawDistance() {
    return getLeftEncoder().getRawPosition();
  }

  @Override
  public double getRightRawDistance() {
    return getRightEncoder().getRawPosition();
  }

  @Override
  public LinearVelocity getLeftVelocity() {
    return getLeftEncoder().getVelocity();
  }

  @Override
  public LinearVelocity getRightVelocity() {
    return getRightEncoder().getVelocity();
  }

  private Supplier<Pose2d> m_referencePositionSupplier = null;

  @Override
  public void setReferencePositionSupplier(Supplier<Pose2d> supplier) {
    m_referencePositionSupplier = supplier;
  }

  @Override
  public ChassisSpeeds getSpeed() {
    return new ChassisSpeeds(
        getLeftVelocity(), getRightVelocity(), getTurnRate());
  }

  @Override
  public void setSpeed(ChassisSpeeds speed) {
    DifferentialDriveWheelSpeeds speedForMotor = m_kinematics.toWheelSpeeds(speed);
    setSpeeds(MetersPerSecond.of(speedForMotor.leftMetersPerSecond),
        MetersPerSecond.of(speedForMotor.rightMetersPerSecond));
    // m_logger.log("SetSpeed not yet implemented", Verbosity.Warn);
  }

  // @Override
  public void driveWithPid(ChassisSpeeds speed) {
    setSpeed(speed);
  }

  protected Pose2d getVisionPose() {
    if (m_referencePositionSupplier != null) {
      return m_referencePositionSupplier.get();
    } else {
      return null;
    }
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
  protected IMotorControllerPlus getLeftLeader() {
    return m_leftMotor;
  }

  protected IMotorControllerPlus getRightLeader() {
    return m_rightMotor;
  }

  @Override
  public boolean setBreakingMode(boolean enable) {
    if (getLeftLeader().canSetBrakeMode()
        && getRightLeader().canSetBrakeMode()) {
      getLeftLeader().setBrakeMode(enable);
      getRightLeader().setBrakeMode(enable);
      return true;
    } else {
      return false;
    }
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
      m_poseEstimator.addVisionMeasurement(getVisionPose(), Timer.getFPGATimestamp());
    }
  }
}
