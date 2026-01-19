// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.actuators.IMotorControllerPlus;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.interfaces.IDrivebasePlus;
import frc.robot.util.BulletinBoard;
import frc.robot.util.RobotConfigs.DriveConfig;

import java.io.IOException;

/**
 * Drivebase subsystem for a differential (tank) drive robot.
 *
 * Notes:
 *
 * <li>This class implements "open loop" control only; there is no PID
 * control or other feedback mechanisms here.
 *
 * <li>This class adds explicit PID-based velocity control, in addition to the
 * basic "direct" control. All direct control driving methods route through
 * 7 * ther tankDrive(double, double) method, which sets the mode accordingly;
 * the
 * new method tankDriveWithPID() switches to PID control mode. When switching
 * to PID control mode, the PID controllers are reset to avoid sudden jumps.
 *
 * </ul>
 */
public class DrivebaseBase extends SubsystemBase implements IDrivebasePlus {
  /** Supported control modes. */
  enum Mode {
    /** Direct control mode (no PID). */
    DIRECT_CONTROL,
    /** PID-based velocity control mode. */
    PID_CONTROL,
  }

  //
  // Constants
  //

  /** Default starting pose for the robot. */
  protected static final Pose2d DEFAULT_STARTING_POSE = new Pose2d(0, 0, new Rotation2d());

  /** Encoder ticks per revolution. */
  public static final int ENCODER_TICKS_PER_REVOLUTION = -4096;

  /** Wheel diameter in inches. */
  public static final Distance WHEEL_DIAMETER = Inches.of(6);

  /** Gearing ratio from motor to wheel. */
  public static final double GEAR_RATIO = 8.45;

  /** Track width (distance between left and right wheels) in meters. */
  public static final Distance TRACK_WIDTH = Meters.of(0.5588); /* 22 inches (from 2024) */

  /** Zero linear velocity. (A potentially useful constant.) */
  public static final LinearVelocity ZERO_MPS = MetersPerSecond.of(0.0);

  /** Zero rotational velocity. (A potentially useful constant.) */
  public static final AngularVelocity ZERO_TURNING = RadiansPerSecond.of(0.0);

  /** Maximum linear velocity that we'll allow/assume in our code. */
  public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.5);

  /** Maximum rotational velocity for arcade drive. */
  public static final AngularVelocity MAX_ROTATION = DegreesPerSecond.of(360);

  /** Kinematics calculator for the drivebase. */
  public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH.in(Meters));

  /** Zero wheel speeds. (A potentially useful constant.) */
  private static final DifferentialDriveWheelSpeeds ZERO_WHEEL_SPEEDS = new DifferentialDriveWheelSpeeds(0.0, 0.0);

  /**
   * Value for voltage required to overcome static friction (used in feedforward
   * calculations; computed via SysID tool).
   */
  public static final double Ks = 0.014183;

  /**
   * Velocity gain (in volts/(m/s)) for linear movement at a given velocity
   * (used in feedforward calculations; computed via SysID tool).
   */
  public static final double Kv = 1.9804;

  /**
   * Acceleration gain (in volts/(m/s^2)) for linear acceleration (used in
   * feedforward calculations; computed via SysID tool).
   */
  public static final double Ka = 0.19169;

  /**
   * Velocity gain for angular/rotational movement at a given velocity (used in
   * feedforward calculations; computed via SysID tool).
   */
  public static final double Kv_Angular = 2.6332;

  /**
   * Acceleration gain for angular acceleration (used in feedforward
   * calculations; computed via SysID tool).
   */
  public static final double Ka_Angular = 0.5226;

  /**
   * Value for the "unit converter" from velocity error (m/s) to motor effort
   * (Volts) under PID control, computed via SysID.
   */
  public static final double Kp = 1.6662;

  /** Feedforward calculator for the drivebase. */
  public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(Ks, Kv, Ka);

  //
  // Core definitions
  //

  protected final DriveConfig m_config;

  /** Left-side motor controller. */
  protected final IMotorControllerPlus m_leftController;

  /** Right-side motor controller. */
  protected final IMotorControllerPlus m_rightController;

  /** Left-side encoder. */
  protected final TrivialEncoder m_leftTrivialEncoder;

  /** Right-side encoder. */
  protected final TrivialEncoder m_rightTrivialEncoder;

  /** Gyro sensor. */
  protected final IGyro m_rawGyro;

  /** Odometry calculator. */
  protected DifferentialDriveOdometry m_odometry;

  /** PID controller for left side velocity control. */
  private final PIDController m_leftPID;

  /** PID controller for right side velocity control. */
  private final PIDController m_rightPID;

  /** Current driving control mode. */
  protected Mode m_mode = Mode.DIRECT_CONTROL;

  /** Creates a new Drivebase. */
  protected DrivebaseBase(DriveConfig config,
      IMotorControllerPlus leftController, IMotorControllerPlus rightController,
      TrivialEncoder leftEncoder, TrivialEncoder rightEncoder, IGyro gyro) {
    setName(SUBSYSTEM_NAME);
    m_config = config;

    //
    // Allocate the hardware components
    //

    m_leftController = leftController;
    m_rightController = rightController;

    // Set up the encoders
    m_leftTrivialEncoder = leftEncoder;
    m_rightTrivialEncoder = rightEncoder;

    m_rawGyro = gyro;

    //
    //

    /** Odometry calculator. */
    m_odometry = new DifferentialDriveOdometry(
        new Rotation2d(m_rawGyro.getAngle()),
        m_leftTrivialEncoder.getPosition().in(Meters), m_rightTrivialEncoder.getPosition().in(
            Meters),
        DEFAULT_STARTING_POSE);

    /** PID controller for left side velocity control. */
    m_leftPID = new PIDController(m_config.leftPid().kP(), m_config.leftPid().kI(), m_config.leftPid().kD());

    /** PID controller for right side velocity control. */
    m_rightPID = new PIDController(m_config.rightPid().kP(), m_config.rightPid().kI(), m_config.rightPid().kD());
  }

  protected static Encoder getConfiguredController(int portId1, int portId2, boolean inverted) {
    final Encoder encoder = new Encoder(portId1, portId2);
    encoder.setReverseDirection(inverted);
    configureEncoderForDistance(encoder, WHEEL_DIAMETER);
    return encoder;
  }

  /**
   * Updates a SparkMaxConfig to work with distance-based values (meters and
   * meters/sec), rather than the native rotation-based units (rotations and
   * RPM).
   *
   * @param encoder       the encoder being configured
   * @param outerDiameter distance of the object (wheel, sprocket, etc.) being
   *                      turned
   */
  protected static void configureEncoderForDistance(
      Encoder encoder, Distance outerDiameter) {
    encoder.setDistancePerPulse(
        Math.PI * WHEEL_DIAMETER.in(Meters) / ENCODER_TICKS_PER_REVOLUTION);
  }

  /**
   * Sets the speeds of the left and right sides of the drivetrain. (Note:
   * operates directly; no PID.)
   *
   * Note that this is an alternative to using "classic" tank driving; this
   * method uses physical wheel speeds, and winds up passing them through to
   * driveTank().
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
    final DifferentialDriveWheelSpeeds wheelSpeeds = KINEMATICS
        .toWheelSpeeds(new ChassisSpeeds(speed, ZERO_MPS, rotation));

    // Set the speeds of the left and right sides of the drivetrain.
    setSpeeds(wheelSpeeds);
  }

  /**
   * Utility method to set the motor voltages. (Shared by several methods.)
   *
   * @param leftVoltage  voltage for the left side
   * @param rightVoltage voltage for the right side
   */
  private void setMotorVoltages(Voltage leftVoltage, Voltage rightVoltage) {
    m_leftController.setVoltage(leftVoltage);
    m_rightController.setVoltage(rightVoltage);
  }

  //
  // Methods from SubsystemBase
  //
  @Override
  public void periodic() {
    m_odometry.update(m_rawGyro.getRotation2d(),
        new DifferentialDriveWheelPositions(
            m_leftTrivialEncoder.getPosition(), m_rightTrivialEncoder.getPosition()));

    // Publish the odometry-based pose to the bulletin board.
    BulletinBoard.common.updateValue(ODOMETRY_KEY, getEstimatedPose());
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

    driveArcade(MAX_SPEED.times(clampedSpeedPercentage),
        MAX_ROTATION.times(clampedRotationPercentage));
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
    m_mode = Mode.DIRECT_CONTROL;
    setMotorVoltages(leftVoltage, rightVoltage);
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

    final double leftOutput = m_leftPID.calculate(
        m_leftTrivialEncoder.getVelocity().in(MetersPerSecond), wheelSpeeds.leftMetersPerSecond);
    final double rightOutput = m_rightPID.calculate(
        m_rightTrivialEncoder.getVelocity().in(MetersPerSecond), wheelSpeeds.rightMetersPerSecond);

    // Apply voltages to the motors.
    setMotorVoltages(Volts.of(leftOutput + leftFeedforward),
        Volts.of(rightOutput + rightFeedforward));
  }

  @Override
  public void setSpeeds(ChassisSpeeds speeds) {
    setSpeeds(KINEMATICS.toWheelSpeeds(speeds));
  }

  @Override
  public Distance getLeftPosition() {
    return m_leftTrivialEncoder.getPosition();
  }

  @Override
  public LinearVelocity getLeftVelocity() {
    return m_leftTrivialEncoder.getVelocity();
  }

  @Override
  public Voltage getLeftVoltage() {
    return m_leftController.getVoltage();
  }

  @Override
  public Distance getRightPosition() {
    return m_rightTrivialEncoder.getPosition();
  }

  @Override
  public LinearVelocity getRightVelocity() {
    return m_rightTrivialEncoder.getVelocity();
  }

  @Override
  public Voltage getRightVoltage() {
    return m_rightController.getVoltage();
  }

  @Override
  public AngularVelocity getAngularVelocity() {
    return m_rawGyro.getRate();
  }

  @Override
  public DriveConfig getConfig() {
    return m_config;
  }

  //
  // Methods from Closeable interface (primarily for unit testing support)
  //

  @Override
  public void close() throws IOException {
    // Close (destroy) our various components.
    m_leftController.close();
    m_rightController.close();
    m_leftTrivialEncoder.close();
    m_rightTrivialEncoder.close();
    m_rawGyro.close();
  }
}
