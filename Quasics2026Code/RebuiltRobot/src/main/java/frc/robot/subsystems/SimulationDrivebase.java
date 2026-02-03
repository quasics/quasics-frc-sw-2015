// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.Constants;
import frc.robot.logging.Logger;
import frc.robot.logging.Logger.Verbosity;

public class SimulationDrivebase extends AbstractDrivebase {
  private Encoder m_leftEncoder = new Encoder(1, 2);
  private Encoder m_rightEncoder = new Encoder(3, 4);
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  private AnalogGyroSim m_gyroSim;
  private TrivialEncoder m_mainLeftEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder, m_leftEncoderSim);
  private TrivialEncoder m_mainRightEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder, m_rightEncoderSim);
  private IGyro m_mainGyro;
  private final Field2d m_field = new Field2d();
  private final Logger m_logger = new Logger(Logger.Verbosity.Info, "SimulatedDriveBase");

  protected final IGyro getGyro() {
    return m_mainGyro;
  }

  protected final TrivialEncoder getLeftEncoder() {
    return m_mainLeftEncoder;
  }

  protected final TrivialEncoder getRightEncoder() {
    return m_mainRightEncoder;
  }

  private DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
      KitbotGearing.k12p75, // 12.75:1 if this changes, we may have to use a new diffDrivetrain sim
      KitbotWheelSize.kSixInch, // 6" diameter wheels.
      null // No measurement noise.
  );

  private static final double TICKS_PER_REVOLUTION = -4096;

  /** Creates a new SimulationDrivebase. */
  public SimulationDrivebase() {
    super(new PWMSparkMax(0), new PWMSparkMax(1));
    AnalogGyro gyro = new AnalogGyro(0);
    m_gyroSim = new AnalogGyroSim(gyro);
    m_mainGyro = IGyro.wrapGyro(gyro);

    // TODO(DISCUSS): Difference between putting this call up here vs
    // simulationPeriodic.
    // - How many times is it called?
    // - Why wouldn't we want to keep calling this? (mjh: you don't.)
    // - EncoderSim vs Encoder
    //
    // FINDME(Robert) - Take another look at these two lines. Are you configuring
    // the correct things? (mjh: hint - you aren't.)
    m_leftEncoderSim.setDistancePerPulse(2.0 * Math.PI * Constants.wheelRadius.in(Meters) / TICKS_PER_REVOLUTION);
    m_rightEncoderSim.setDistancePerPulse(2.0 * Math.PI * Constants.wheelRadius.in(Meters) / TICKS_PER_REVOLUTION);
  }

  // TODO(DISCUSS): What changes when we remove this override? Why?
  /*
   * @Override
   * public void periodic() {
   * var pose = getOdometry().getPoseMeters();
   * m_field.setRobotPose(pose);
   * m_field.getObject("Estimated Drivebase Pose").setPose(getEstimatedPose());
   * // This method will be called once per scheduler run
   * }
   */

  public void simulationPeriodic() {
    // TODO: Add abstractDriveBase getDriveSpeeds
    m_driveSim.setInputs(getLeftLeader().get() * RobotController.getInputVoltage(),
        getRightLeader().get() * RobotController.getInputVoltage());
    m_driveSim.update(0.02);
    m_logger.log("Left motor controller = " + getLeftLeader().get(), Verbosity.Debug);
    m_logger.log("Right motor controller = " + getRightLeader().get(), Verbosity.Debug);
    // getHeading returns counterclockwise positive, Gyros are clockwise positive
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_logger.log("Left encoder = " + getLeftEncoder().getPosition(), Verbosity.Debug);
    m_logger.log("Right encoder = " + getRightEncoder().getPosition(), Verbosity.Debug);
  }

}
