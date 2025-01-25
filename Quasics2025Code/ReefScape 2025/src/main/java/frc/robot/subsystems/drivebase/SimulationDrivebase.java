// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SimulationPorts;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

public class SimulationDrivebase extends IDrivebase {
  private static final Distance kWheelRadius = Meters.of(0.0508);
  private static final int kEncoderResolutionTicksPerRevoltion = -4096;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;
  private final TrivialEncoder m_leftTrivialEncoder;
  private final TrivialEncoder m_rightTrivialEncoder;

  private final IGyro m_wrappedGyro;

  private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  private final DifferentialDrivetrainSim m_drivetrainSim;
  private final Field2d m_fieldSim = new Field2d();
  private final AnalogGyroSim m_gyroSim;
  private final EncoderSim m_leftEncoderSim;
  private final EncoderSim m_rightEncoderSim;

  final PWMSparkMax m_leftLeader = new PWMSparkMax(SimulationPorts.LEFT_FRONT_DRIVE_PWM_ID);
  final PWMSparkMax m_leftFollower = new PWMSparkMax(SimulationPorts.LEFT_REAR_DRIVE_PWM_ID);
  final PWMSparkMax m_rightLeader = new PWMSparkMax(SimulationPorts.RIGHT_FRONT_DRIVE_PWM_ID);
  final PWMSparkMax m_rightFollower = new PWMSparkMax(SimulationPorts.RIGHT_REAR_DRIVE_PWM_ID);

  /** Creates a new SimulationDrivebase. */
  public SimulationDrivebase(RobotSettings.Robot robot) {
    super(RobotSettings.Robot.Simulator);
    super.setName(getClass().getSimpleName());

    m_leftEncoder = new Encoder(
        SimulationPorts.LEFT_DRIVE_ENCODER_PORT_A, SimulationPorts.LEFT_DRIVE_ENCODER_PORT_B);
    m_rightEncoder = new Encoder(
        SimulationPorts.RIGHT_DRIVE_ENCODER_PORT_A, SimulationPorts.RIGHT_DRIVE_ENCODER_PORT_B);

    final AnalogGyro rawGyro = new AnalogGyro(0);
    m_wrappedGyro = IGyro.wrapGyro(rawGyro);

    configureDriveMotorsAndSensors();

    m_drivetrainSim = new DifferentialDrivetrainSim(m_drivetrainSystem, DCMotor.getCIM(2), 8,
        robot.trackWidthMeters.in(Meters), kWheelRadius.in(Meters), null);
    m_gyroSim = new AnalogGyroSim(rawGyro);
    m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    SmartDashboard.putData("field", m_fieldSim);

    m_leftTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder, m_leftEncoderSim);
    m_rightTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder, m_rightEncoderSim);
  }

  private void configureDriveMotorsAndSensors() {
    m_leftLeader.addFollower(m_leftFollower);
    m_rightLeader.addFollower(m_rightFollower);

    m_rightLeader.setInverted(true);
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    super.resetOdometry(pose);

    // Update the pose information in the simulator.
    m_drivetrainSim.setPose(pose);
  }

  @Override
  public void periodic() {
    super.periodic();

    var pose = getOdometry().getPoseMeters();
    m_fieldSim.setRobotPose(pose);
    m_fieldSim.getObject("Estimated pose").setPose((getEstimatedPose()));
  }

  @Override
  protected void setMotorVoltages_HAL(double leftVoltage, double rightVoltage) {
    m_leftLeader.setVoltage(leftVoltage);
    m_rightLeader.setVoltage(rightVoltage);
  }

  @Override
  protected void setSpeeds_HAL(double leftSpeed, double rightSpeed) {
    m_leftLeader.setVoltage(leftSpeed);
    m_rightLeader.setVoltage(rightSpeed);
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
  public void simulationPeriodic() {
    super.simulationPeriodic();
  }
}
