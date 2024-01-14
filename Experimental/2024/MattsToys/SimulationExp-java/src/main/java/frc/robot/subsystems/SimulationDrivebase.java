// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;

public class SimulationDrivebase extends AbstractDrivebase {
  private static final double kTrackWidthMeters = 0.381 * 2;
  private static final double kWheelRadiusMeters = 0.0508;
  private static final int kEncoderResolutionTicksPerRevolution = -4096;

  // PID constants obtained via SysId logging analysis.
  private static final double kP = 1.3195; // Sample was 8.5
  private static final double kI = 0;
  private static final double kD = 0;

  // Motor gains obtained via SysId logging analysis.
  private static final double kS = 0.013809; // Sample was 1
  private static final double kV = 1.9805;   // Sample was 3
  private static final double kA = 0.19208;  // Sample was 0

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;
  private final TrivialEncoder m_leftTrivialEncoder;
  private final TrivialEncoder m_rightTrivialEncoder;

  private final IGyro m_wrappedGyro;

  private final DifferentialDriveOdometry m_odometry;

  // Objects used in simulation mode.
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  private final DifferentialDrivetrainSim m_drivetrainSimulator =
      new DifferentialDrivetrainSim(m_drivetrainSystem, DCMotor.getCIM(2), 8,
                                    kTrackWidthMeters, kWheelRadiusMeters,
                                    null);
  private final Field2d m_fieldSim = new Field2d();
  private final AnalogGyroSim m_gyroSim;
  private final EncoderSim m_leftEncoderSim;
  private final EncoderSim m_rightEncoderSim;

  // Hardware allocation
  final PWMSparkMax m_leftLeader = new PWMSparkMax(1);
  final PWMSparkMax m_leftFollower = new PWMSparkMax(2);
  final PWMSparkMax m_rightLeader = new PWMSparkMax(3);
  final PWMSparkMax m_rightFollower = new PWMSparkMax(4);

  /** Subsystem constructor. */
  public SimulationDrivebase() {
    super(kTrackWidthMeters, kP, kI, kD, kS, kV, kA);

    super.setName(getClass().getSimpleName());

    m_leftEncoder = new Encoder(0, 1);
    m_rightEncoder = new Encoder(2, 3);
    m_leftTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder);
    m_rightTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder);

    final AnalogGyro rawGyro = new AnalogGyro(0);
    m_wrappedGyro = IGyro.wrapGyro(rawGyro);

    // Initial odometry; it will be updated in periodic().
    m_odometry = new DifferentialDriveOdometry(
        m_wrappedGyro.getRotation2d(), m_leftTrivialEncoder.getPosition(),
        m_rightTrivialEncoder.getPosition());

    // Finish configuring the hardware.
    configureDriveMotorsAndSensors();

    // Set up simulation
    m_gyroSim = new AnalogGyroSim(rawGyro);
    m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    SmartDashboard.putData("Field", m_fieldSim);
  }

  private void configureDriveMotorsAndSensors() {
    // Set up leader/follower relationships.
    m_leftLeader.addFollower(m_leftFollower);
    m_rightLeader.addFollower(m_rightFollower);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLeader.setInverted(true);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadiusMeters /
                                      kEncoderResolutionTicksPerRevolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadiusMeters /
                                       kEncoderResolutionTicksPerRevolution);

    // Make sure our encoders are zeroed out on startup.
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  // ---------------------------------------------------------------------------
  // Implementations of abstract functions from the base class.
  // ---------------------------------------------------------------------------

  @Override
  protected void setMotorVoltagesImpl(double leftVoltage, double rightVoltage) {
    m_leftLeader.setVoltage(leftVoltage);
    m_rightLeader.setVoltage(rightVoltage);
  }

  @Override
  protected DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  @Override
  protected TrivialEncoder getLeftEncoder() {
    return m_leftTrivialEncoder;
  }

  @Override
  protected TrivialEncoder getRightEncoder() {
    return m_rightTrivialEncoder;
  }

  @Override
  protected IGyro getGyro() {
    return m_wrappedGyro;
  }

  // ---------------------------------------------------------------------------
  // Simulation support.
  // ---------------------------------------------------------------------------

  @Override
  public void resetOdometry(Pose2d pose) {
    super.resetOdometry(pose);

    // Update the pose information in the simulator.
    m_drivetrainSimulator.setPose(pose);
  }

  @Override
  public void periodic() {
    super.periodic();

    // Update published field simulation data.
    var pose = getOdometry().getPoseMeters();
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
    m_fieldSim.setRobotPose(pose);
  }

  /**
   * Update our simulation. This should be run every robot loop in simulation.
   */
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(
        m_leftLeader.get() * RobotController.getInputVoltage(),
        m_rightLeader.get() * RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.02);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(
        m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(
        m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(
        m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  protected double getLeftSpeedPercentage() { return m_leftLeader.get(); }

  protected double getRightSpeedPercentage() { return m_rightLeader.get(); }
}
