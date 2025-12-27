// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

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
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IDrivebase;

/** Drivebase subsystem for a differential (tank) drive robot. */
public class Drivebase extends SubsystemBase implements IDrivebase {
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

  final private SparkMax leftLeader = new SparkMax(1, MotorType.kBrushless);
  final private SparkMax rightLeader = new SparkMax(2, MotorType.kBrushless);

  final private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH.in(Meters));

  //
  // Simulation support
  //
  private final SparkMaxSim leftLeaderSim = new SparkMaxSim(leftLeader, DCMotor.getNEO(1));
  private final SparkMaxSim rightLeaderSim = new SparkMaxSim(rightLeader, DCMotor.getNEO(1));

  /**
   * Linear system describing the drive train.
   *
   * Notice that this data will (had better!) look *remarkably* similar to the
   * computed "feed forward" values for this simulated drive base, since they
   * are... well, the actual/ideal values defining that.
   *
   * @see frc.robot.utils.RobotConfigs.DriveFeedForwardConfig
   */
  final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);

  /** Simulation driver for the overall drive train. */
  final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(
      m_drivetrainSystem,
      // Drive motor type and count
      DCMotor.getNEO(4),
      GEAR_RATIO,
      TRACK_WIDTH.in(Meters),
      WHEEL_DIAMETER_INCHES.in(Meters),
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
  public static void configureSparkMaxEncoderForDistance(
      SparkMaxConfig config, Distance outerDiameter, double gearRatio) {
    final double distanceScalingFactorForGearing = outerDiameter.div(gearRatio).in(Meters);
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;

    config.encoder.positionConversionFactor(distanceScalingFactorForGearing)
        .velocityConversionFactor(velocityScalingFactor);
  }

  /** Creates a new Drivebase. */
  public Drivebase() {
    setName(SUBSYSTEM_NAME);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    configureSparkMaxEncoderForDistance(leftConfig, WHEEL_DIAMETER_INCHES, GEAR_RATIO);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.apply(leftConfig);
    rightConfig.inverted(true);

    leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void driveTank(double leftSpeed, double rightSpeed) {
    // Don't let the values go outside of [-100%, +100%].
    double clampedLeftSpeed = MathUtil.clamp(leftSpeed, -1.0, +1.0);
    double clampedRightSpeed = MathUtil.clamp(rightSpeed, -1.0, +1.0);

    leftLeader.set(clampedLeftSpeed);
    rightLeader.set(clampedRightSpeed);
  }

  @Override
  public void driveArcade(double forward, double rotation) {
    // Don't let the values go outside of [-100%, +100%].
    double clampedSpeedPercentage = MathUtil.clamp(forward, -1.0, +1.0);
    double clampedRotationPercentage = MathUtil.clamp(rotation, -1.0, +1.0);

    arcadeDrive(
        MAX_SPEED.times(clampedSpeedPercentage), MAX_ROTATION.times(clampedRotationPercentage));
  }

  /**
   * Drive the robot using arcade drive.
   *
   * Note: operates directly; no PID, but clamped to MAX_SPEED.
   *
   * @param speed    The linear velocity to drive at.
   * @param rotation The angular velocity to rotate at.
   */
  public void arcadeDrive(LinearVelocity speed, AngularVelocity rotation) {
    if (speed == null) {
      speed = ZERO_MPS;
    }
    if (rotation == null) {
      rotation = ZERO_TURNING;
    }

    // Calculate the left and right wheel speeds based on the inputs.
    final DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics
        .toWheelSpeeds(new ChassisSpeeds(speed, ZERO_MPS, rotation));

    // Set the speeds of the left and right sides of the drivetrain.
    setSpeeds(wheelSpeeds);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Convert the wheel speeds to motor power levels.
    double leftOutput = speeds.leftMetersPerSecond / MAX_SPEED.in(MetersPerSecond);
    double rightOutput = speeds.rightMetersPerSecond / MAX_SPEED.in(MetersPerSecond);

    // Set the motor outputs.
    driveTank(leftOutput, rightOutput);
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(leftLeader.get() * RoboRioSim.getVInVoltage(),
        rightLeader.get() * RoboRioSim.getVInVoltage());

    // Simulated clock ticks forward
    m_drivetrainSimulator.update(0.02);

    leftLeaderSim.getAbsoluteEncoderSim()
        .setPosition(m_drivetrainSimulator.getLeftPositionMeters());
    rightLeaderSim.getAbsoluteEncoderSim()
        .setPosition(m_drivetrainSimulator.getRightPositionMeters());

    leftLeaderSim.getAbsoluteEncoderSim()
        .setVelocity(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    rightLeaderSim.getAbsoluteEncoderSim()
        .setVelocity(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
  }
}
