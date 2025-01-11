// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.simulations.Constants.*;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IDrivebase;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

public class SimDrivebase extends SubsystemBase implements IDrivebase {
  // private final PIDController m_leftPIDController = new PIDController(1.3195, 0, 0);
  // private final PIDController m_rightPIDController = new PIDController(1.3195, 0, 0);
  final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kRobotTrackWidthMeters.baseUnitMagnitude());

  // Hardware allocation
  final PWMSparkMax m_left = new PWMSparkMax(LEFT_DRIVE_PWM_ID);
  final PWMSparkMax m_right = new PWMSparkMax(RIGHT_DRIVE_PWM_ID);

  final Encoder m_leftEncoder = new Encoder(LEFT_DRIVE_ENCODER_PORT_A, LEFT_DRIVE_ENCODER_PORT_B);
  final Encoder m_rightEncoder =
      new Encoder(RIGHT_DRIVE_ENCODER_PORT_A, RIGHT_DRIVE_ENCODER_PORT_B);
  final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  // Objects used in simulation mode.
  final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  final DifferentialDrivetrainSim m_drivetrainSimulator =
      new DifferentialDrivetrainSim(m_drivetrainSystem, DCMotor.getCIM(2), 8,
          kRobotTrackWidthMeters.baseUnitMagnitude(), kWheelRadiusMeters.baseUnitMagnitude(), null);
  final Field2d m_fieldSim = new Field2d();

  /** Creates a new SimDrivebase. */
  public SimDrivebase() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_right.setInverted(true);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadiusMeters.baseUnitMagnitude()
        / kEncoderResolutionTicksPerRevolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadiusMeters.baseUnitMagnitude()
        / kEncoderResolutionTicksPerRevolution);

    // Make sure our encoders are zeroed out on startup.
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    // Add the simulated field to the smart dashboard
    SmartDashboard.putData("Field", m_fieldSim);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

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

    // m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    // Publish the data for any that need it.
    // BulletinBoard.common.updateValue(SIMULATOR_POSE_KEY, m_drivetrainSimulator.getPose());
  }

  @Override
  public void setSpeed(double leftPercentage, double rightPercentage) {
    m_left.set(leftPercentage);
    m_right.set(rightPercentage);
  }

  public Distance getLeftPositionMeters() {
    return Meters.of(m_leftEncoder.getDistance());
  }

  public Distance getRightPositionMeters() {
    return Meters.of(m_rightEncoder.getDistance());
  }
}
