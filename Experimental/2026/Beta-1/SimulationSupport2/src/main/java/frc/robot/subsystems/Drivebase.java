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
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.simulated.SimulationUxSupport;

/** Drivebase subsystem for a differential (tank) drive robot. */
public class Drivebase extends SubsystemBase implements IDrivebase {
  private static final int ENCODER_TICKS_PER_REVOLUTION = -4096;
  /** Wheel diameter in inches. */
  final static Distance WHEEL_DIAMETER_INCHES = Inches.of(6);
  /** Gearing ratio from motor to wheel. */
  final static double GEAR_RATIO = 8.45;
  /** Track width (distance between left and right wheels) in meters. */
  final static Distance TRACK_WIDTH = Meters.of(0.5588); /* 22 in (from 2024) */

  /** Zero linear velocity. (A potentially useful constant.) */
  final static LinearVelocity ZERO_MPS = MetersPerSecond.of(0.0);

  /** Zero rotational velocity. (A potentially useful constant.) */
  final static AngularVelocity ZERO_TURNING = RadiansPerSecond.of(0.0);

  /** Maximum linear velocity that we'll allow/assume in our code. */
  final static LinearVelocity MAX_SPEED = MetersPerSecond.of(3.5);

  /** Maximum rotational velocity for arcade drive. */
  final static AngularVelocity MAX_ROTATION = Units.DegreesPerSecond.of(120.0);

  //
  // Core definitions
  //

  final private PWMSparkMax leftController = new PWMSparkMax(Ports.LEFT_MOTOR_PWM_PORT);
  final private PWMSparkMax rightController = new PWMSparkMax(Ports.RIGHT_MOTOR_PWM_PORT);

  private final Encoder leftEncoder =
      new Encoder(Ports.LEFT_ENCODER_A_DIO_PORT, Ports.LEFT_ENCODER_B_DIO_PORT);
  private final Encoder rightEncoder =
      new Encoder(Ports.RIGHT_ENCODER_A_DIO_PORT, Ports.RIGHT_ENCODER_B_DIO_PORT);

  final AnalogGyro rawGyro = new AnalogGyro(Ports.GYRO_CHANNEL_PORT);

  final private DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(TRACK_WIDTH.in(Meters));

  //
  // Simulation support
  //
  final EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  final EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
  final AnalogGyroSim m_gyroSim = new AnalogGyroSim(rawGyro);

  /**
   * Linear system describing the drive train.
   *
   * Notice that this data will (had better!) look *remarkably* similar to the
   * computed "feed forward" values for this simulated drive base, since they
   * are... well, the actual/ideal values defining that.
   *
   * @see frc.robot.utils.RobotConfigs.DriveFeedForwardConfig
   */
  final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);

  /** Simulation driver for the overall drive train. */
  final DifferentialDrivetrainSim m_drivetrainSimulator =
      new DifferentialDrivetrainSim(m_drivetrainSystem,
          // Drive motor type and count
          DCMotor.getNEO(4), GEAR_RATIO, TRACK_WIDTH.in(Meters), WHEEL_DIAMETER_INCHES.in(Meters),
          // configure for no noise in measurements
          null);

  /**
   * Updates a SparkMaxConfig to work with distance-based values (meters and
   * meters/sec), rather than the native rotation-based units (rotations and RPM).
   *
   * @param config        the object being configured
   * @param outerDiameter distance of the object (wheel, sprocket, etc.) being
   *                      turned
   * @param gearRatio     gearing ratio of the motor to the object being turned
   *                      (i.e., given a ratio of 1 turn of the external object
   *                      for every N turns of the motor, this would be N)
   */
  private static void configureEncoderForDistance(Encoder encoder, Distance outerDiameter) {
    encoder.setDistancePerPulse(
        Math.PI * WHEEL_DIAMETER_INCHES.in(Meters) / ENCODER_TICKS_PER_REVOLUTION);
  }

  /** Creates a new Drivebase. */
  public Drivebase() {
    setName(SUBSYSTEM_NAME);

    rightEncoder.setReverseDirection(true);

    configureEncoderForDistance(leftEncoder, WHEEL_DIAMETER_INCHES);
    configureEncoderForDistance(rightEncoder, WHEEL_DIAMETER_INCHES);
  }

  @Override
  public void driveTank(double leftSpeed, double rightSpeed) {
    // Don't let the values go outside of [-100%, +100%].
    double clampedLeftSpeed = MathUtil.clamp(leftSpeed, -1.0, +1.0);
    double clampedRightSpeed = MathUtil.clamp(rightSpeed, -1.0, +1.0);

    leftController.set(clampedLeftSpeed);
    rightController.set(clampedRightSpeed);
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

  @Override
  public void driveArcade(double forward, double rotation) {
    // Don't let the values go outside of [-100%, +100%].
    double clampedSpeedPercentage = MathUtil.clamp(forward, -1.0, +1.0);
    double clampedRotationPercentage = MathUtil.clamp(rotation, -1.0, +1.0);

    driveArcade(
        MAX_SPEED.times(clampedSpeedPercentage), MAX_ROTATION.times(clampedRotationPercentage));
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

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(leftController.get() * RoboRioSim.getVInVoltage(),
        rightController.get() * RoboRioSim.getVInVoltage());

    // Simulated clock ticks forward
    m_drivetrainSimulator.update(0.02);

    // Update the simulated encoders and gyro
    leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    // Update the field simulation
    SimulationUxSupport.instance.updateFieldRobotPose(m_drivetrainSimulator.getPose());
  }
}
