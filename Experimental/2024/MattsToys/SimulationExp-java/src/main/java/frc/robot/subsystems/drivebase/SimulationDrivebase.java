// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
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
import frc.robot.utils.BulletinBoard;
import frc.robot.utils.RobotSettings;

public class SimulationDrivebase extends AbstractDrivebase {
  public static final String SIMULATOR_POSE_KEY = "SimDrive.SimPose";

  private static final Measure<Distance> kWheelRadius = Meters.of(0.0508);
  private static final int kEncoderResolutionTicksPerRevolution = -4096;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;
  private final TrivialEncoder m_leftTrivialEncoder;
  private final TrivialEncoder m_rightTrivialEncoder;

  private final IGyro m_wrappedGyro;

  // Objects used in simulation mode.
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
      0.3);
  private final DifferentialDrivetrainSim m_drivetrainSimulator;
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
  public SimulationDrivebase(RobotSettings.Robot robot) {
    super(RobotSettings.Robot.Simulator);
    super.setName(getClass().getSimpleName());

    m_leftEncoder = new Encoder(0, 1);
    m_rightEncoder = new Encoder(2, 3);

    final AnalogGyro rawGyro = new AnalogGyro(0);
    m_wrappedGyro = IGyro.wrapGyro(rawGyro);

    // Finish configuring the hardware.
    configureDriveMotorsAndSensors();

    // Set up simulation
    m_drivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem,
        DCMotor.getCIM(2), 8,
        robot.trackWidthMeters.in(Meters), kWheelRadius.in(Meters),
        null);
    m_gyroSim = new AnalogGyroSim(rawGyro);
    m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    SmartDashboard.putData("Field", m_fieldSim);

    m_leftTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder, m_leftEncoderSim);
    m_rightTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder, m_rightEncoderSim);
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
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius.in(Meters) /
        kEncoderResolutionTicksPerRevolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius.in(Meters) /
        kEncoderResolutionTicksPerRevolution);

    // Make sure our encoders are zeroed out on startup.
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  // ---------------------------------------------------------------------------
  // Implementations of abstract functions from the base class.
  // ---------------------------------------------------------------------------

  @Override
  protected void setMotorVoltages_HAL(double leftVoltage, double rightVoltage) {
    m_leftLeader.setVoltage(leftVoltage);
    m_rightLeader.setVoltage(rightVoltage);

    logValue("Left volts", leftVoltage);
    logValue("Right volts", rightVoltage);
  }

  @Override
  protected TrivialEncoder getLeftEncoder_HAL() {
    return m_leftTrivialEncoder;
  }

  @Override
  protected TrivialEncoder getRightEncoder_HAL() {
    return m_rightTrivialEncoder;
  }

  @Override
  protected IGyro getGyro_HAL() {
    return m_wrappedGyro;
  }

  protected double getLeftSpeedPercentage_HAL() {
    return m_leftLeader.get();
  }

  protected double getRightSpeedPercentage_HAL() {
    return m_rightLeader.get();
  }

  protected void tankDrivePercent_HAL(double leftPercent, double rightPercent) {
    m_leftLeader.setVoltage(leftPercent);
    m_rightLeader.setVoltage(rightPercent);

    logValue("Left volts", leftPercent);
    logValue("Right volts", rightPercent);
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
    m_fieldSim.getObject("Estimated pose").setPose(getEstimatedPose());
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

    // Publish the data for any that need it.
    BulletinBoard.common.updateValue(SIMULATOR_POSE_KEY, m_drivetrainSimulator.getPose());
  }

  protected double getLeftVoltage_HAL() {
    return convertPercentSpeedToVoltage(m_leftLeader.get());
  }

  protected double getRightVoltage_HAL() {
    return convertPercentSpeedToVoltage(m_rightLeader.get());
  }
}
