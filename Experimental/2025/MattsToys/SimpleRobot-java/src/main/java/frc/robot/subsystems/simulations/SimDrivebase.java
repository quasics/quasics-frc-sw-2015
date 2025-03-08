// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.simulations.SimulationPorts.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.abstracts.AbstractDrivebase;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Defines a version of IDrivebase that runs under (full) simulation.
 */
public class SimDrivebase extends AbstractDrivebase {
  private static final int kEncoderResolutionTicksPerRevolution = -4096;

  // "Hardware" allocation
  private final PWMSparkMax m_left = new PWMSparkMax(LEFT_DRIVE_PWM_ID);
  private final PWMSparkMax m_right = new PWMSparkMax(RIGHT_DRIVE_PWM_ID);
  private final Encoder m_leftEncoder = new Encoder(LEFT_DRIVE_ENCODER_PORT_A, LEFT_DRIVE_ENCODER_PORT_B);
  private final Encoder m_rightEncoder = new Encoder(RIGHT_DRIVE_ENCODER_PORT_A, RIGHT_DRIVE_ENCODER_PORT_B);
  private final IGyro m_wrappedGyro;

  final private TrivialEncoder m_leftTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder);
  final private TrivialEncoder m_rightTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder);

  /** Odometry for the robot, purely calculated from encoders/gyro. */
  final private DifferentialDriveOdometry m_odometry;

  /** Drivetrain pose estimator. */
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  /////////////////////////////////////////////////////////////////////////////////////
  // Simulated "hardware" and other simulation-specific objects.
  final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  final AnalogGyroSim m_gyroSim;
  final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  final DifferentialDrivetrainSim m_drivetrainSimulator;
  final Field2d m_fieldSim = new Field2d();

  /**
   * Creates a new SimDrivebase.
   *
   * @param config configuration of the robot being targeted
   */
  public SimDrivebase(RobotConfig config) {
    super(config);

    final var driveConfig = config.drive();

    final double trackWidthMeters = driveConfig.trackWidth().in(Meters);

    m_drivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem, DCMotor.getCIM(2),
        driveConfig.gearing(), driveConfig.trackWidth().in(Meters), trackWidthMeters, null);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_right.setInverted(true);

    // Set the distance per pulse (in meters) for the drive encoders. We can simply
    // use the distance traveled for one rotation of the wheel divided by the
    // encoder resolution.
    m_leftEncoder.setDistancePerPulse(
        2 * Math.PI * driveConfig.wheelRadius().in(Meters) / kEncoderResolutionTicksPerRevolution);
    m_rightEncoder.setDistancePerPulse(
        2 * Math.PI * driveConfig.wheelRadius().in(Meters) / kEncoderResolutionTicksPerRevolution);

    // Make sure our encoders are zeroed out on startup.
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    // Set up the gyro
    final AnalogGyro rawGyro = new AnalogGyro(GYRO_CHANNEL); // also used in simulation setup
    m_wrappedGyro = IGyro.wrapGyro(rawGyro);

    // Set up the odometry and pose estimator
    m_odometry = new DifferentialDriveOdometry(m_wrappedGyro.getRotation2d(),
        m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), new Pose2d());
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics,
        m_wrappedGyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(),
        new Pose2d(), VecBuilder.fill(0.05, 0.05, Radians.convertFrom(5, Degrees)),
        VecBuilder.fill(0.5, 0.5, Radians.convertFrom(30, Degrees)));

    //
    // Pure simulation support
    //
    m_gyroSim = new AnalogGyroSim(rawGyro);

    // Add the simulated field to the smart dashboard
    SmartDashboard.putData("Field", m_fieldSim);
  }

  @Override
  public void periodic() {
    super.periodic();

    // Update the field simulator to reflect refreshed odometry.
    m_fieldSim.setRobotPose(getPose());
    m_fieldSim.getObject("Estimated pose").setPose(getEstimatedPose());
  }

  /**
   * Updates the simulation of the drive (position, encoders, current draw, etc.).
   */
  private void updateSimulation() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(m_left.get() * RoboRioSim.getVInVoltage(),
        m_right.get() * RobotController.getInputVoltage());

    // Simulated clock ticks forward
    m_drivetrainSimulator.update(0.02);

    // Update the encoders, based on what the drive train simulation says happend.
    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    SimulationUxSupport.instance.postCurrentDraw(m_drivetrainSimulator.getCurrentDrawAmps());
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    if (!DriverStation.isDisabled()) {
      updateSimulation();
    }
  }

  @Override
  public void setMotorVoltages(Voltage left, Voltage right) {
    m_left.setVoltage(left);
    m_right.setVoltage(right);

    logValue("left", left);
    logValue("right", left);
  }

  @Override
  public TrivialEncoder getLeftEncoder() {
    return m_leftTrivialEncoder;
  }

  @Override
  public TrivialEncoder getRightEncoder() {
    return m_rightTrivialEncoder;
  }

  @Override
  public IGyro getGyro() {
    return IGyro.readOnlyGyro(m_wrappedGyro);
  }

  @Override
  public Voltage getLeftVoltage() {
    return Volts.of(m_left.getVoltage());
  }

  @Override
  public Voltage getRightVoltage() {
    return Volts.of(m_right.getVoltage());
  }

  @Override
  protected DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  @Override
  protected DifferentialDrivePoseEstimator getPoseEstimator() {
    return m_poseEstimator;
  }
}
