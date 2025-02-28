// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import frc.robot.Constants;
import frc.robot.Constants.SimulationPorts;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
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
import edu.wpi.first.units.measure.Voltage;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

public class SimulationDrivebase extends AbstractDrivebase {

  private static final Distance kWheelRadius = Meters.of(0.0508);
  private static final int kEncoderResolutionTicksPerRevolution = -4096;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;
  private final TrivialEncoder m_leftTrivialEncoder;
  private final TrivialEncoder m_rightTrivialEncoder;

  private final IGyro m_wrappedGyro;

  private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
      0.3);
  private final DifferentialDrivetrainSim m_drivetrainSim;
  private final Field2d m_fieldSim = new Field2d();
  private final AnalogGyroSim m_gyroSim;
  private final EncoderSim m_leftEncoderSim;
  private final EncoderSim m_rightEncoderSim;

  final PWMSparkMax m_leftLeader = new PWMSparkMax(SimulationPorts.LEFT_FRONT_DRIVE_PWM_ID);
  final PWMSparkMax m_leftFollower = new PWMSparkMax(SimulationPorts.LEFT_REAR_DRIVE_PWM_ID);
  final PWMSparkMax m_rightLeader = new PWMSparkMax(SimulationPorts.RIGHT_FRONT_DRIVE_PWM_ID);
  final PWMSparkMax m_rightFollower = new PWMSparkMax(SimulationPorts.RIGHT_REAR_DRIVE_PWM_ID);

  private final double distancePerPulse;

  /** Creates a new SimulationDrivebase. */
  public SimulationDrivebase(RobotSettings.Robot robot) {
    super(RobotSettings.Robot.Simulator);
    super.setName(getClass().getSimpleName());

    m_leftEncoder = new Encoder(SimulationPorts.LEFT_DRIVE_ENCODER_PORT_A, SimulationPorts.LEFT_DRIVE_ENCODER_PORT_B);
    m_rightEncoder = new Encoder(SimulationPorts.RIGHT_DRIVE_ENCODER_PORT_A,
        SimulationPorts.RIGHT_DRIVE_ENCODER_PORT_B);

    final AnalogGyro rawGyro = new AnalogGyro(0);
    m_wrappedGyro = IGyro.wrapGyro(rawGyro);

    configureDriveMotorsAndSensors();

    m_drivetrainSim = new DifferentialDrivetrainSim(m_drivetrainSystem,
        DCMotor.getCIM(2), 8, robot.trackWidthMeters.in(Meters), kWheelRadius.in(Meters), null);
    m_gyroSim = new AnalogGyroSim(rawGyro);
    m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    m_rightEncoderSim = new EncoderSim(m_rightEncoder);

    m_leftTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder, m_leftEncoderSim);
    m_rightTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder, m_rightEncoderSim);

    // Set the distance per pulse (in meters) for the drive encoders. We can simply
    // use the distance traveled for one rotation of the wheel divided by the
    // encoder resolution.

    distancePerPulse = 2 * Math.PI * kWheelRadius.in(Meters) / kEncoderResolutionTicksPerRevolution;

    m_leftEncoder.setDistancePerPulse(
        distancePerPulse);
    m_rightEncoder.setDistancePerPulse(
        distancePerPulse);

    // Make sure our encoders are zeroed out on startup.
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    SmartDashboard.putData("field", m_fieldSim);
  }

  private void configureDriveMotorsAndSensors() {
    m_leftLeader.addFollower(m_leftFollower);
    m_rightLeader.addFollower(m_rightFollower);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLeader.setInverted(true);

  }

  @Override
  public void resetOdometry(Pose2d pose) {
    super.resetOdometry(pose);

    // Update the pose information in the simulator.
    m_drivetrainSim.setPose(pose);
  }

  @Override
  protected void setMotorVoltages_HAL(double leftVoltage, double rightVoltage) {
    m_leftLeader.setVoltage(leftVoltage);
    m_rightLeader.setVoltage(rightVoltage);
  }

  @Override
  protected void setSpeeds_HAL(double leftSpeed, double rightSpeed) {
    m_leftLeader.set(leftSpeed);
    m_rightLeader.set(rightSpeed);
  }

  @Override
  protected void setSpeeds_HAL(DifferentialDriveWheelSpeeds speeds) {
    double leftPercent = speeds.leftMetersPerSecond / 6.06;
    double rightPercent = speeds.rightMetersPerSecond / 6.06;
    setSpeeds(leftPercent, rightPercent);
  }

  @Override
  public Voltage getLeftVoltage() {
    return Volts.of(m_leftLeader.getVoltage());
  }

  @Override
  public Voltage getRightVoltage() {
    return Volts.of(m_rightLeader.getVoltage());
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

  @Override
  public void periodic() {
    super.periodic();

    // Update the position for the robot that is shown in the simulated field, using
    // the odometry data that will have been computed by the base class.
    var pose = getOdometry().getPoseMeters();
    m_fieldSim.setRobotPose(pose);
    m_fieldSim.getObject("Estimated pose").setPose((getEstimatedPose()));
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSim.setInputs(m_leftLeader.get() * RobotController.getInputVoltage(),
        m_rightLeader.get() * RobotController.getInputVoltage());

    // Simulated clock ticks forward
    m_drivetrainSim.update(0.02);

    // Update the encoders and gyro, based on what the drive train simulation says
    // happend.
    m_leftEncoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSim.getHeading().getDegrees());
  }

}