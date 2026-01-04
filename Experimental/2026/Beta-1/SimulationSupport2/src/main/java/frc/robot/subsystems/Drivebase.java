// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IDrivebasePlus;

/**
 * Drivebase subsystem for a differential (tank) drive robot.
 *
 * Notes:
 *
 * <li>I'm using PWMSparkMax controllers in this code because of a bug in the current (Beta1)
 * version of RevLib, which is causing crashes during simulation, at least under MacOS.  (A [bug
 * report](https://github.com/wpilibsuite/2026Beta/issues/29) has been filed.)
 *
 * <li>This class implements "open loop" control only; there is no PID
 * control or other feedback mechanisms here.
 *
 * <li>Simulation support is provided in a subclass, SimDrivebase.  This is simply to keep
 * simulation-specific code separate from "real" robot code, in order to provide greater clarity as
 * an example; this functionality could easily be merged into this class instead.
 *
 * <li>This class adds explicit PID-based velocity control, in addition to the basic "direct"
 * control.  All direct control driving methods route through ther tankDrive(double, double) method,
 * which sets the mode accordingly; the new method tankDriveWithPID() switches to PID control
 * mode.  When switching to PID control mode, the PID controllers are reset to avoid sudden jumps.
 *
 * </ul>
 */
public class Drivebase extends SubsystemBase implements IDrivebasePlus {
  // Supported control modes.
  enum Mode { DIRECT_CONTROL, PID_CONTROL }

  //
  // Constants
  //

  /** Default starting pose for the robot. */
  protected static final Pose2d DEFAULT_STARTING_POSE = new Pose2d(0, 0, new Rotation2d());

  /** Encoder ticks per revolution. */
  public static final int ENCODER_TICKS_PER_REVOLUTION = -4096;

  /** Wheel diameter in inches. */
  public final static Distance WHEEL_DIAMETER = Inches.of(6);

  /** Gearing ratio from motor to wheel. */
  public final static double GEAR_RATIO = 8.45;

  /** Track width (distance between left and right wheels) in meters. */
  public final static Distance TRACK_WIDTH = Meters.of(0.5588); /* 22 in (from 2024) */

  /** Zero linear velocity. (A potentially useful constant.) */
  public final static LinearVelocity ZERO_MPS = MetersPerSecond.of(0.0);

  /** Zero rotational velocity. (A potentially useful constant.) */
  public final static AngularVelocity ZERO_TURNING = RadiansPerSecond.of(0.0);

  /** Maximum linear velocity that we'll allow/assume in our code. */
  public final static LinearVelocity MAX_SPEED = MetersPerSecond.of(3.5);

  /** Maximum rotational velocity for arcade drive. */
  public final static AngularVelocity MAX_ROTATION = Units.DegreesPerSecond.of(120.0);

  /** Kinematics calculator for the drivebase. */
  public final static DifferentialDriveKinematics KINEMATICS =
      new DifferentialDriveKinematics(TRACK_WIDTH.in(Meters));

  /** Zero wheel speeds. (A potentially useful constant.) */
  private final static DifferentialDriveWheelSpeeds ZERO_WHEEL_SPEEDS =
      new DifferentialDriveWheelSpeeds(0.0, 0.0);

  /**
   * Value for voltage required to overcome static friction (used in feedforward calculations;
   * computed via SysID tool).
   */
  public final static double Ks = 0.014183;

  /**
   * Velocity gain (in volts/(m/s)) to move at a given velocity (used in feedforward
   * calculations; computed via SysID tool).
   */
  public final static double Kv = 1.9804;

  /**
   * Acceleration gain (in volts/(m/s^2)) to accelerate at a given rate (used in feedforward
   * calculations; computed via SysID tool).
   */
  public final static double Ka = 0.19169;

  /**
   * Value for the "unit converter" from velocity error (m/s) to motor effort (Volts) under PID
   * control, computed via SysID.
   */
  public final static double Kp = 1.6662;

  /** Feedforward calculator for the drivebase. */
  public final static SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(Ks, Kv, Ka);

  //
  // Core definitions
  //

  /** Left-side motor controller. */
  final protected PWMSparkMax m_leftController = new PWMSparkMax(Ports.LEFT_MOTOR_PWM_PORT);

  /** Right-side motor controller. */
  final protected PWMSparkMax m_rightController = new PWMSparkMax(Ports.RIGHT_MOTOR_PWM_PORT);

  /** Left-side encoder. */
  protected final Encoder m_leftEncoder =
      new Encoder(Ports.LEFT_ENCODER_A_DIO_PORT, Ports.LEFT_ENCODER_B_DIO_PORT);

  /** Right-side encoder. */
  protected final Encoder m_rightEncoder =
      new Encoder(Ports.RIGHT_ENCODER_A_DIO_PORT, Ports.RIGHT_ENCODER_B_DIO_PORT);

  /** Gyro sensor. */
  final protected AnalogGyro m_rawGyro = new AnalogGyro(Ports.GYRO_CHANNEL_PORT);

  /** Odometry calculator. */
  protected DifferentialDriveOdometry m_odometry =
      new DifferentialDriveOdometry(new Rotation2d(Degrees.of(m_rawGyro.getAngle())),
          m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), DEFAULT_STARTING_POSE);

  /** Current driving control mode. */
  protected Mode m_mode = Mode.DIRECT_CONTROL;

  /** PID controller for left side velocity control. */
  PIDController m_leftPID = new PIDController(Kp, 0.0, 0.0);

  /** PID controller for right side velocity control. */
  PIDController m_rightPID = new PIDController(Kp, 0.0, 0.0);

  /** Creates a new Drivebase. */
  public Drivebase() {
    setName(SUBSYSTEM_NAME);

    // Set up the encoders
    m_rightEncoder.setReverseDirection(true);
    m_leftEncoder.setReverseDirection(false);
    configureEncoderForDistance(m_leftEncoder, WHEEL_DIAMETER);
    configureEncoderForDistance(m_rightEncoder, WHEEL_DIAMETER);
  }

  /**
   * Updates a SparkMaxConfig to work with distance-based values (meters and
   * meters/sec), rather than the native rotation-based units (rotations and RPM).
   *
   * @param encoder       the encoder being configured
   * @param outerDiameter distance of the object (wheel, sprocket, etc.) being
   *                      turned
   */
  protected static void configureEncoderForDistance(Encoder encoder, Distance outerDiameter) {
    encoder.setDistancePerPulse(Math.PI * WHEEL_DIAMETER.in(Meters) / ENCODER_TICKS_PER_REVOLUTION);
  }

  /**
   * Sets the speeds of the left and right sides of the drivetrain. (Note:
   * operates directly; no PID.)
   *
   * Note that this is an alternative to using "classic" tank driving; this method
   * uses physical wheel speeds, and winds up passing them through to driveTank().
   *
   * @param speeds the desired wheel speeds
   *
   * @see #driveTank(double, double)
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    if (speeds == null) {
      speeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);
    }

    // Convert the wheel speeds to motor power levels.
    final double leftOutput = speeds.leftMetersPerSecond / MAX_SPEED.in(MetersPerSecond);
    final double rightOutput = speeds.rightMetersPerSecond / MAX_SPEED.in(MetersPerSecond);

    // Set the motor outputs.
    driveTank(leftOutput, rightOutput);
  }

  /**
   * Drive the robot using arcade drive. (Note: operates directly; no PID.)
   *
   * Note that this is an alternative to using "classic" arcade driving; this
   * method uses physical wheel speeds, and winds up passing them through to
   * driveArcade().
   *
   * @param speed    The linear velocity to drive at.
   * @param rotation The angular velocity to rotate at.
   *
   * @see #driveArcade(double, double)
   */
  public void driveArcade(LinearVelocity speed, AngularVelocity rotation) {
    if (speed == null) {
      speed = ZERO_MPS;
    }
    if (rotation == null) {
      rotation = ZERO_TURNING;
    }

    // Calculate the left and right wheel speeds based on the inputs.
    final DifferentialDriveWheelSpeeds wheelSpeeds =
        KINEMATICS.toWheelSpeeds(new ChassisSpeeds(speed, ZERO_MPS, rotation));

    // Set the speeds of the left and right sides of the drivetrain.
    setSpeeds(wheelSpeeds);
  }

  //
  // Methods from SubsystemBase
  //
  @Override
  public void periodic() {
    m_odometry.update(m_rawGyro.getRotation2d(),
        new DifferentialDriveWheelPositions(
            m_leftEncoder.getDistance(), m_rightEncoder.getDistance()));
  }

  //
  // Methods from IDrivebase
  //

  @Override
  public void driveTank(double leftSpeed, double rightSpeed) {
    m_mode = Mode.DIRECT_CONTROL;

    // Don't let the values go outside of [-100%, +100%].
    double clampedLeftSpeed = MathUtil.clamp(leftSpeed, -1.0, +1.0);
    double clampedRightSpeed = MathUtil.clamp(rightSpeed, -1.0, +1.0);

    m_leftController.set(clampedLeftSpeed);
    m_rightController.set(clampedRightSpeed);
  }

  @Override
  public void driveArcade(double forward, double rotation) {
    // Don't let the values go outside of [-100%, +100%].
    double clampedSpeedPercentage = MathUtil.clamp(forward, -1.0, +1.0);
    double clampedRotationPercentage = MathUtil.clamp(rotation, -1.0, +1.0);

    driveArcade(
        MAX_SPEED.times(clampedSpeedPercentage), MAX_ROTATION.times(clampedRotationPercentage));
  }

  @Override
  public LinearVelocity getMaxLinearSpeed() {
    return MAX_SPEED;
  }

  @Override
  public AngularVelocity getMaxRotationalSpeed() {
    return MAX_ROTATION;
  }

  @Override
  public DifferentialDriveKinematics getKinematics() {
    return KINEMATICS;
  }

  //
  // Methods from IDrivebasePlus
  //

  @Override
  public Pose2d getEstimatedPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void tankDriveVolts(Voltage leftVoltage, Voltage rightVoltage) {
    m_leftController.setVoltage(leftVoltage);
    m_rightController.setVoltage(rightVoltage);
  }

  @Override
  public void driveTankWithPID(ChassisSpeeds speeds) {
    // If we've just switched to PID control, reset the controllers.
    if (m_mode != Mode.PID_CONTROL) {
      m_leftPID.reset();
      m_rightPID.reset();
    }
    m_mode = Mode.PID_CONTROL;

    // Convert speeds to m/s.
    DifferentialDriveWheelSpeeds wheelSpeeds;
    if (speeds != null) {
      wheelSpeeds = KINEMATICS.toWheelSpeeds(speeds);
    } else {
      wheelSpeeds = ZERO_WHEEL_SPEEDS;
    }

    // Calculate feedforward and PID outputs.
    final double leftFeedforward = FEEDFORWARD.calculate(wheelSpeeds.leftMetersPerSecond);
    final double rightFeedforward = FEEDFORWARD.calculate(wheelSpeeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPID.calculate(m_leftEncoder.getRate(), wheelSpeeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPID.calculate(m_rightEncoder.getRate(), wheelSpeeds.rightMetersPerSecond);

    // Apply voltages to the motors.
    tankDriveVolts(
        Volts.of(leftOutput + leftFeedforward), Volts.of(rightOutput + rightFeedforward));
  }

  @Override
  public void setSpeeds(ChassisSpeeds speeds) {
    setSpeeds(KINEMATICS.toWheelSpeeds(speeds));
  }

  @Override
  public Distance getLeftPosition() {
    return Meters.of(m_leftEncoder.getDistance());
  }

  @Override
  public LinearVelocity getLeftVelocity() {
    return MetersPerSecond.of(m_leftEncoder.getRate());
  }

  @Override
  public Voltage getLeftVoltage() {
    return Volts.of(m_leftController.getVoltage());
  }

  @Override
  public Distance getRightPosition() {
    return Meters.of(m_rightEncoder.getDistance());
  }

  @Override
  public LinearVelocity getRightVelocity() {
    return MetersPerSecond.of(m_rightEncoder.getRate());
  }

  @Override
  public Voltage getRightVoltage() {
    return Volts.of(m_rightController.getVoltage());
  }

  @Override
  public double getKa() {
    return Ka;
  }

  @Override
  public double getKs() {
    return Ks;
  }

  @Override
  public double getKv() {
    return Kv;
  }

  @Override
  public double getKp() {
    return Kp;
  }
}
