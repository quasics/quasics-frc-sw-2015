// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IDrivebase;

/**
 * Drivebase subsystem for a differential (tank) drive robot.
 *
 * Notes:
 *
 * <li>I'm using PWMSparkMax controllers in this code because of a bug in the current (Beta1)
 * version of RevLib, which is causing crashes during simulation, at least under MacOS.
 *
 * <li>This class implements "open loop" control only; there is no PID
 * control or other feedback mechanisms here.
 *
 * <li>Simulation support is provided in a subclass, SimDrivebase.  This is simply to keep
 * simulation-specific code separate from "real" robot code, in order to provide greater clarity as
 * an example; this functionality could easily be merged into this class instead.
 *
 * </ul>
 */
public class Drivebase extends SubsystemBase implements IDrivebase {
  //
  // Constants
  //

  /** Encoder ticks per revolution. */
  protected static final int ENCODER_TICKS_PER_REVOLUTION = -4096;

  /** Wheel diameter in inches. */
  final protected static Distance WHEEL_DIAMETER = Inches.of(6);

  /** Gearing ratio from motor to wheel. */
  final protected static double GEAR_RATIO = 8.45;

  /** Track width (distance between left and right wheels) in meters. */
  final protected static Distance TRACK_WIDTH = Meters.of(0.5588); /* 22 in (from 2024) */

  /** Zero linear velocity. (A potentially useful constant.) */
  final protected static LinearVelocity ZERO_MPS = MetersPerSecond.of(0.0);

  /** Zero rotational velocity. (A potentially useful constant.) */
  final protected static AngularVelocity ZERO_TURNING = RadiansPerSecond.of(0.0);

  /** Maximum linear velocity that we'll allow/assume in our code. */
  final protected static LinearVelocity MAX_SPEED = MetersPerSecond.of(3.5);

  /** Maximum rotational velocity for arcade drive. */
  final protected static AngularVelocity MAX_ROTATION = Units.DegreesPerSecond.of(120.0);

  //
  // Core definitions
  //

  /** Left-side motor controller. */
  final protected PWMSparkMax leftController = new PWMSparkMax(Ports.LEFT_MOTOR_PWM_PORT);

  /** Right-side motor controller. */
  final protected PWMSparkMax rightController = new PWMSparkMax(Ports.RIGHT_MOTOR_PWM_PORT);

  /** Left-side encoder. */
  protected final Encoder leftEncoder =
      new Encoder(Ports.LEFT_ENCODER_A_DIO_PORT, Ports.LEFT_ENCODER_B_DIO_PORT);

  /** Right-side encoder. */
  protected final Encoder rightEncoder =
      new Encoder(Ports.RIGHT_ENCODER_A_DIO_PORT, Ports.RIGHT_ENCODER_B_DIO_PORT);

  /** Gyro sensor. */
  final protected AnalogGyro rawGyro = new AnalogGyro(Ports.GYRO_CHANNEL_PORT);

  /** Kinematics calculator for the drivebase. */
  final protected DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(TRACK_WIDTH.in(Meters));

  /** Creates a new Drivebase. */
  public Drivebase() {
    setName(SUBSYSTEM_NAME);

    // Set up the encoders
    rightEncoder.setReverseDirection(true);
    leftEncoder.setReverseDirection(false);
    configureEncoderForDistance(leftEncoder, WHEEL_DIAMETER);
    configureEncoderForDistance(rightEncoder, WHEEL_DIAMETER);
  }

  /**
   * Updates a SparkMaxConfig to work with distance-based values (meters and
   * meters/sec), rather than the native rotation-based units (rotations and RPM).
   *
   * @param config        the encoder being configured
   * @param outerDiameter distance of the object (wheel, sprocket, etc.) being
   *                      turned
   * @param gearRatio     gearing ratio of the motor to the object being turned
   *                      (i.e., given a ratio of 1 turn of the external object
   *                      for every N turns of the motor, this would be N)
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
        m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, ZERO_MPS, rotation));

    // Set the speeds of the left and right sides of the drivetrain.
    setSpeeds(wheelSpeeds);
  }

  //
  // Methods from IDrivebase
  //

  @Override
  public void driveTank(double leftSpeed, double rightSpeed) {
    // Don't let the values go outside of [-100%, +100%].
    double clampedLeftSpeed = MathUtil.clamp(leftSpeed, -1.0, +1.0);
    double clampedRightSpeed = MathUtil.clamp(rightSpeed, -1.0, +1.0);

    leftController.set(clampedLeftSpeed);
    rightController.set(clampedRightSpeed);
  }

  @Override
  public void driveArcade(double forward, double rotation) {
    // Don't let the values go outside of [-100%, +100%].
    double clampedSpeedPercentage = MathUtil.clamp(forward, -1.0, +1.0);
    double clampedRotationPercentage = MathUtil.clamp(rotation, -1.0, +1.0);

    driveArcade(
        MAX_SPEED.times(clampedSpeedPercentage), MAX_ROTATION.times(clampedRotationPercentage));
  }
}
