// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  private static final boolean ENABLE_VOLTAGE_APPLICATON = true;

  protected static final boolean LOG_TO_SMARTDASHBOARD = false;

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

  void logValue(String label, double val) {
    if (LOG_TO_SMARTDASHBOARD) {
      SmartDashboard.putNumber(label, val);
    }
  }

  /**
   * Controls the robot using arcade drive.
   *
   * @param xSpeed the speed for the x axis (in m/s)
   * @param rot    the rotation (in radians/s)
   */
  public final void arcadeDrive(double xSpeed, double rot) {
    logValue("xSpeed", xSpeed);
    logValue("rotSpeed", rot);
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
  }

  final static DeadbandEnforcer speedEnforcer = new DeadbandEnforcer(0.1);

  /** Sets speeds to the drivetrain motors. */
  public final void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    logValue("leftSpeed", speeds.leftMetersPerSecond);
    logValue("rightSpeed", speeds.rightMetersPerSecond);

    var leftStabilized = speedEnforcer.limit(speeds.leftMetersPerSecond);
    var rightStabilized = speedEnforcer.limit(speeds.rightMetersPerSecond);
    logValue("leftStable", leftStabilized);
    logValue("rightStable", rightStabilized);

    // Figure out the voltages we should need at the target speeds.
    var leftFeedforward = m_feedforward.calculate(leftStabilized);
    var rightFeedforward = m_feedforward.calculate(rightStabilized);
    logValue("leftFF", leftFeedforward);
    logValue("rightFF", rightFeedforward);

    // Figure out the deltas, based on our current speed vs. the target speeds.
    double leftPidOutput = m_leftPIDController.calculate(getLeftEncoder().getVelocity(), speeds.leftMetersPerSecond);
    double rightPidOutput = m_rightPIDController.calculate(
        getRightEncoder().getVelocity(), speeds.rightMetersPerSecond);
    logValue("leftPid", leftPidOutput);
    logValue("rightPid", rightPidOutput);

    // OK, apply those to the actual hardware.
    setMotorVoltages(leftFeedforward + leftPidOutput, rightFeedforward + rightPidOutput);
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
    if (ENABLE_VOLTAGE_APPLICATON) {
      this.setMotorVoltagesImpl(leftVoltage, rightVoltage);
    }
    m_lastLeftVoltage = leftVoltage;
    m_lastRightVoltage = rightVoltage;
  }

  @Override
  public void periodic() {
    if (ENABLE_VOLTAGE_APPLICATON) {
      setMotorVoltagesImpl(m_lastLeftVoltage, m_lastRightVoltage);
    }
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

  /////////////////////////////////////////////////////////////////////////////////
  //
  // Hardware abstraction layer definition
  //
  /////////////////////////////////////////////////////////////////////////////////

  protected abstract DifferentialDriveOdometry getOdometry();

  protected abstract TrivialEncoder getLeftEncoder();

  protected abstract TrivialEncoder getRightEncoder();

  protected abstract IGyro getGyro();

  protected abstract double getLeftSpeedPercentage();

  protected abstract double getRightSpeedPercentage();

  /**
   * Declared as public so that it can be used with RamseteCommand objects.
   *
   * @param leftVoltage  voltage for left-side motors
   * @param rightVoltage voltage for right-side motors
   */
  protected abstract void setMotorVoltagesImpl(double leftVoltage, double rightVoltage);

  /////////////////////////////////////////////////////////////////////////////////
  //
  // SysId (profiling) support
  //
  /////////////////////////////////////////////////////////////////////////////////

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            final double rawVolts = volts.in(Volts);
            setMotorVoltagesImpl(rawVolts, rawVolts);
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // Record a frame for the left motors. Since these share an encoder, we consider
            // the entire group to be one motor.
            log.motor("drive-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        getLeftSpeedPercentage() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(getLeftEncoder().getPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(getLeftEncoder().getVelocity(), MetersPerSecond));
            // Record a frame for the right motors. Since these share an encoder, we
            // consider the entire group to be one motor.
            log.motor("drive-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        getRightSpeedPercentage() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(getRightEncoder().getPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(getRightEncoder().getVelocity(), MetersPerSecond));
          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in WPILog with this subsystem's name ("drive")
          this));

  /**
   * @return a Command for use in running quasistatic profiling in the
   *         specified direction.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * @return a Command for use in running dynamic profiling in the
   *         specified direction.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
