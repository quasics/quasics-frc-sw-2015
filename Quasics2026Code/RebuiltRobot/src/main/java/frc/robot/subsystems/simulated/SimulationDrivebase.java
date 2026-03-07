// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulated;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants;
import frc.robot.Constants.PwmPortIds;
import frc.robot.hardware.actuators.IMotorControllerPlus;
import frc.robot.hardware.sensors.IGyro;
import frc.robot.hardware.sensors.TrivialEncoder;
import frc.robot.logging.Logger;
import frc.robot.logging.Logger.Verbosity;
import frc.robot.subsystems.real.AbstractDrivebase;

public class SimulationDrivebase extends AbstractDrivebase {
  private static final int GYRO_CHANNEL = 0;
  private static final int LEFT_ENCODER_CHANNEL_A = 1;
  private static final int LEFT_ENCODER_CHANNEL_B = 2;
  private static final int RIGHT_ENCODER_CHANNEL_A = 3;
  private static final int RIGHT_ENCODER_CHANNEL_B = 4;

  private final Logger m_logger = new Logger(Logger.Verbosity.Info, "SimulatedDriveBase");

  private final TrivialEncoder m_leftEncoder;
  private final TrivialEncoder m_rightEncoder;
  private final EncoderSim m_leftEncoderSim;
  private final EncoderSim m_rightEncoderSim;

  private final IGyro m_gyro;
  private final AnalogGyroSim m_gyroSim;

  private DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
      Constants.GEARING,
      KitbotWheelSize.kSixInch, // 6" diameter wheels.
      null // No measurement noise.
  );

  /** Creates a new SimulationDrivebase. */
  public SimulationDrivebase() {
    super(IMotorControllerPlus.forPWMMotorController(new PWMSparkMax(
        PwmPortIds.SIMULATED_LEFT_MOTOR_CHANNEL)),
        IMotorControllerPlus.forPWMMotorController(new PWMSparkMax(PwmPortIds.SIMULATED_RIGHT_MOTOR_CHANNEL)));
    AnalogGyro gyro = new AnalogGyro(GYRO_CHANNEL);
    m_gyroSim = new AnalogGyroSim(gyro);
    m_gyro = IGyro.wrapGyro(gyro);

    Encoder leftEncoder = new Encoder(LEFT_ENCODER_CHANNEL_A, LEFT_ENCODER_CHANNEL_B);
    Encoder rightEncoder = new Encoder(RIGHT_ENCODER_CHANNEL_A, RIGHT_ENCODER_CHANNEL_B);

    System.out.println(Constants.WHEEL_RADIUS);
    System.out.println(2.0 * Math.PI * Constants.WHEEL_RADIUS.in(Meters) / -4096.0);
    System.out.println(getDistancePerPulse());
    leftEncoder.setDistancePerPulse(getDistancePerPulse());
    rightEncoder.setDistancePerPulse(getDistancePerPulse());

    m_leftEncoderSim = new EncoderSim(leftEncoder);
    m_rightEncoderSim = new EncoderSim(rightEncoder);
    m_leftEncoder = TrivialEncoder.forWpiLibEncoder(leftEncoder, m_leftEncoderSim);
    m_rightEncoder = TrivialEncoder.forWpiLibEncoder(rightEncoder, m_rightEncoderSim);
  }

  protected static double getDistancePerPulse() {
    return 2.0 * Math.PI * Constants.WHEEL_RADIUS.in(Meters) / -4096.0;
  }

  @Override
  protected final IGyro getGyro() {
    return m_gyro;
  }

  @Override
  protected final TrivialEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  @Override
  protected final TrivialEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  @Override
  public void simulationPeriodic() {
    // Log starting conditions
    m_logger.log(
        "Left motor controller = " + getLeftLeader().get(), Verbosity.Debug);
    m_logger.log(
        "Right motor controller = " + getRightLeader().get(), Verbosity.Debug);

    // Update the simulation (m_driveSim), based on 1/50th of a second passing
    // TODO: Add abstractDriveBase getDriveSpeeds
    m_driveSim.setInputs(
        getLeftLeader().get() * RobotController.getInputVoltage(),
        getRightLeader().get() * RobotController.getInputVoltage());
    m_driveSim.update(0.02);

    // getHeading returns counterclockwise positive, Gyros are clockwise
    // positive, so we need to invert the measurement
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

    // Update the encoders
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());

    // Log the ending conditions
    m_logger.log(
        "Left encoder = " + getLeftEncoder().getPosition(), Verbosity.Debug);
    m_logger.log(
        "Right encoder = " + getRightEncoder().getPosition(), Verbosity.Debug);
  }
}
