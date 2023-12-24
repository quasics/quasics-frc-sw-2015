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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.DeadbandEnforcer;

public abstract class AbstractDrivebase extends SubsystemBase {
  /** Maximum linear speed is 3 meters per second. */
  public static final double MAX_SPEED = 3.0;

  /** Maximum rotational speed is 1/2 rotation per second. */
  public static final double MAX_ANGULAR_SPEED = Math.PI;

  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDController;
  private final SimpleMotorFeedforward m_feedforward;
  private final DifferentialDriveKinematics m_kinematics;

  /** Most recently set voltage for left side, for use in periodic(). */
  private double m_lastLeftVoltage = 0;

  /** Most recently set voltage for right side, for use in periodic(). */
  private double m_lastRightVoltage = 0;

  /**
   * Constructor.
   *
   * @param trackWidthMeters track width (from SysID using the "Drivetrain
   *                         (Angular)" test)
   * @param kP               kP value for PID control of motors
   * @param kI               kI value for PID control of motors
   * @param kD               kD value for PID control of motors
   * @param kS               voltage needed to overcome the drive motors' static
   *                         friction
   * @param kV               voltage scaling value used to hold at a given
   *                         velocity
   *
   * @see
   *      https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#the-permanent-magnet-dc-motor-feedforward-equation
   * @see
   *      https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/identification-routine.html#track-width
   */
  public AbstractDrivebase(
      double trackWidthMeters, double kP, double kI, double kD, double kS, double kV) {
    this(trackWidthMeters, kP, kI, kD, kS, kV, 0);
  }

  /**
   * Constructor.
   *
   * @param trackWidthMeters track width (from SysID using the "Drivetrain
   *                         (Angular)" test)
   * @param kP               kP value for PID control of motors
   * @param kI               kI value for PID control of motors
   * @param kD               kD value for PID control of motors
   * @param kS               voltage needed to overcome the drive motors' static
   *                         friction
   * @param kV               voltage scaling value used to hold a given velocity
   * @param kA               voltage scaling value used to hold a given
   *                         acceleration
   *
   * @see
   *      https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#the-permanent-magnet-dc-motor-feedforward-equation
   * @see
   *      https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/identification-routine.html#track-width
   */
  public AbstractDrivebase(
      double trackWidthMeters, double kP, double kI, double kD, double kS, double kV, double kA) {
    m_leftPIDController = new PIDController(kP, kI, kD);
    m_rightPIDController = new PIDController(kP, kI, kD);
    m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    m_kinematics = new DifferentialDriveKinematics(trackWidthMeters);
  }

  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public SimpleMotorFeedforward getMotorFeedforward() {
    return m_feedforward;
  }

  public double getKP() {
    return m_leftPIDController.getP();
  }

  public double getKI() {
    return m_leftPIDController.getI();
  }

  public double getKD() {
    return m_leftPIDController.getD();
  }

  /** @return current wheel speeds (in m/s) */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftEncoder().getVelocity(), getRightEncoder().getVelocity());
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

  public final void stop() {
    arcadeDrive(0, 0);
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
    setMotorVoltages(leftFeedforward + leftOutput, rightFeedforward + rightOutput);
  }

  /**
   * Applies the specified voltages to the motors (and remembers what was set, so
   * that we can periodically update it, as required for voltage compensation to
   * work properly).
   *
   * @param leftVoltage
   * @param rightVoltage
   *
   * @see edu.wpi.first.wpilibj.motorcontrol.MotorController#setVoltage
   */
  public void setMotorVoltages(double leftVoltage, double rightVoltage) {
    this.setMotorVoltagesImpl(leftVoltage, rightVoltage);
    m_lastLeftVoltage = leftVoltage;
    m_lastRightVoltage = rightVoltage;
  }

  @Override
  public void periodic() {
    setMotorVoltagesImpl(m_lastLeftVoltage, m_lastRightVoltage);
    updateOdometry();
  }

  /** Prevents us from pushing voltage/speed values too small for the motors. */
  final static DeadbandEnforcer m_voltageDeadbandEnforcer = new DeadbandEnforcer(-0.001);

  public static double convertVoltageToPercentSpeed(double volts) {
    final double inputVoltage = RobotController.getInputVoltage();
    final double mps = (volts / inputVoltage);
    final double speedPercentage = m_voltageDeadbandEnforcer.limit(mps / MAX_SPEED);
    return speedPercentage;
  }

  protected abstract DifferentialDriveOdometry getOdometry();

  protected abstract TrivialEncoder getLeftEncoder();

  protected abstract TrivialEncoder getRightEncoder();

  protected abstract IGyro getGyro();

  /**
   * Declared as public so that it can be used with RamseteCommand objects.
   *
   * @param leftVoltage  voltage for left-side motors
   * @param rightVoltage voltage for right-side motors
   */
  protected abstract void setMotorVoltagesImpl(double leftVoltage, double rightVoltage);
}
