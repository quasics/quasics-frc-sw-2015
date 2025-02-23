// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.simulations.SimulationPorts.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Defines a version of IDrivebase that runs under (full) simulation.
 */
public class SimDrivebase extends SubsystemBase implements IDrivebase {
  public static final int kEncoderResolutionTicksPerRevolution = -4096;

  private final DifferentialDriveKinematics m_kinematics;

  // "Hardware" allocation
  private final PWMSparkMax m_left = new PWMSparkMax(LEFT_DRIVE_PWM_ID);
  private final PWMSparkMax m_right = new PWMSparkMax(RIGHT_DRIVE_PWM_ID);
  private final Encoder m_leftEncoder = new Encoder(LEFT_DRIVE_ENCODER_PORT_A, LEFT_DRIVE_ENCODER_PORT_B);
  private final Encoder m_rightEncoder = new Encoder(RIGHT_DRIVE_ENCODER_PORT_A, RIGHT_DRIVE_ENCODER_PORT_B);
  private final IGyro m_wrappedGyro;

  final private TrivialEncoder m_leftTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder);
  final private TrivialEncoder m_rightTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder);

  /** Odometry for the robot, purely calculated from encoders/gyro. */
  final private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0,
      new Pose2d());

  /** Drivetrain pose estimator. */
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  private final LTVUnicycleController unicycleController = new LTVUnicycleController(0.02);

  /////////////////////////////////////////////////////////////////////////////////////
  // Simulated "hardware" and other simulation-specific objects.
  final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  final AnalogGyroSim m_gyroSim;
  final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  final DifferentialDrivetrainSim m_drivetrainSimulator;
  final Field2d m_fieldSim = new Field2d();

  /** Creates a new SimDrivebase. */
  public SimDrivebase(RobotConfig config) {
    setName(SUBSYSTEM_NAME);

    final var driveConfig = config.drive();

    final double trackWidthMeters = driveConfig.trackWidth().in(Meters);

    m_kinematics = new DifferentialDriveKinematics(trackWidthMeters);

    m_drivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem,
        DCMotor.getCIM(2),
        driveConfig.gearing(),
        driveConfig.trackWidth().in(Meters),
        trackWidthMeters, null);

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

    // Set up the pose estimator
    m_poseEstimator = new DifferentialDrivePoseEstimator(
        m_kinematics,
        m_wrappedGyro.getRotation2d(),
        m_leftEncoder.getDistance(), m_rightEncoder.getDistance(),
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Radians.convertFrom(5, Degrees)),
        VecBuilder.fill(0.5, 0.5, Radians.convertFrom(30, Degrees)));

    m_leftPidController = new PIDController(driveConfig.pid().kP(), driveConfig.pid().kI(), driveConfig.pid().kD());
    m_rightPidController = new PIDController(driveConfig.pid().kP(), driveConfig.pid().kI(), driveConfig.pid().kD());
    m_feedforward = new DifferentialDriveFeedforward(
        driveConfig.feedForward().linear().kV().in(Volts),
        driveConfig.feedForward().linear().kA(),
        driveConfig.feedForward().angular().kV().in(Volts),
        driveConfig.feedForward().angular().kA());

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

    updateOdometry(m_odometry, m_poseEstimator);

    publishData(m_leftPidController, m_rightPidController);

    // Update the field simulator to reflect refreshed odometry.
    m_fieldSim.setRobotPose(getPose());
    m_fieldSim.getObject("Estimated pose").setPose(getEstimatedPose());
  }

  // Note: this method will be called once per scheduler run
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(m_left.get() * RobotController.getInputVoltage(),
        m_right.get() * RobotController.getInputVoltage());

    // Simulated clock ticks forward
    m_drivetrainSimulator.update(0.02);

    // Update the encoders, based on what the drive train simulation says happend.
    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    // Publish the data for any that need it.
    // BulletinBoard.common.updateValue(SIMULATOR_POSE_KEY,
    // m_drivetrainSimulator.getPose());
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
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  @Override
  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
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
  public ChassisSpeeds getCurrentSpeeds() {
    return new ChassisSpeeds(getLeftVelocity(), getRightVelocity(), m_wrappedGyro.getRate());
  }

  /**
   * Resets robot odometry (e.g., if we know that we've been placed at a
   * specific position/angle on the field, such as at the start of a match).
   */
  public void resetOdometry(Pose2d pose) {
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();
    // m_gyroSim.setAngle(pose.getRotation().getDegrees());

    // Update the pose information in the simulator.
    m_drivetrainSimulator.setPose(pose);

    m_odometry.resetPosition(
        m_wrappedGyro.getRotation2d(),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance(),
        pose);
    m_poseEstimator.resetPosition(
        m_wrappedGyro.getRotation2d(),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance(),
        pose);
  }

  @Override
  public void setMotorSpeeds(double leftPercentage, double rightPercentage) {
    // Clamp speeds to the range [-1.0, 1.0].
    leftPercentage = Math.max(-1.0, Math.min(1.0, leftPercentage));
    rightPercentage = Math.max(-1.0, Math.min(1.0, rightPercentage));

    m_left.set(leftPercentage);
    m_right.set(rightPercentage);
  }

  @Override
  public void setMotorVoltages(Voltage left, Voltage right) {
    m_left.setVoltage(left);
    m_right.setVoltage(right);

    logValue("left", left);
    logValue("right", left);
  }

  @Override
  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        m_wrappedGyro.getRotation2d(),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance(),
        pose);
    m_poseEstimator.resetPosition(
        m_wrappedGyro.getRotation2d(),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance(),
        pose);
  }

  final PIDController m_leftPidController;
  final PIDController m_rightPidController;
  final DifferentialDriveFeedforward m_feedforward;

  @Override
  public void driveWithPid(ChassisSpeeds speeds) {
    driveWithPid(m_kinematics.toWheelSpeeds(speeds));
  }

  @Override
  public void driveWithPid(DifferentialDriveWheelSpeeds wheelSpeeds) {
    // var leftStabilized =
    // wheelSpeedsDeadband.limit(wheelSpeeds.leftMetersPerSecond);
    // var rightStabilized =
    // wheelSpeedsDeadband.limit(wheelSpeeds.rightMetersPerSecond);
    // logValue("leftStable", leftStabilized);
    // logValue("rightStable", rightStabilized);

    // Figure out the voltages we should need at the target speeds.
    final var feedforwardVolts = m_feedforward.calculate(
        getLeftVelocity().in(MetersPerSecond),
        wheelSpeeds.leftMetersPerSecond,
        getRightVelocity().in(MetersPerSecond),
        wheelSpeeds.rightMetersPerSecond,
        0.020);
    logValue("FF left", feedforwardVolts.left);
    logValue("FF right", feedforwardVolts.right);

    // Figure out the deltas, based on our current speed vs. the target speeds.
    double leftPidOutput = m_leftPidController.calculate(
        getLeftVelocity().in(MetersPerSecond), wheelSpeeds.leftMetersPerSecond);
    double rightPidOutput = m_rightPidController.calculate(
        getRightVelocity().in(MetersPerSecond), wheelSpeeds.rightMetersPerSecond);
    logValue("leftPid", leftPidOutput);
    logValue("rightPid", rightPidOutput);

    // OK, apply those to the actual hardware.
    setMotorVoltages(Volts.of(feedforwardVolts.left + leftPidOutput),
        Volts.of(feedforwardVolts.right + rightPidOutput));
  }

  @Override
  public LTVUnicycleController getLtvUnicycleController() {
    return unicycleController;
  }
}
