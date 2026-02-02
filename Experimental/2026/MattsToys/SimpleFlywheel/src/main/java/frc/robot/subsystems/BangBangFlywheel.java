// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BangBangFlywheel extends SubsystemBase {
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;

  private double m_setpointRadiansPerSec = 0.0;

  private final PWMSparkMax m_flywheelMotor = new PWMSparkMax(kMotorPort);
  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);

  private final BangBangController m_bangBangController = new BangBangController();

  // Gains are for example purposes only - must be determined for your own robot!
  public static final double kFlywheelKs = 0.0001; // V
  public static final double kFlywheelKv = 0.000195; // V/RPM
  public static final double kFlywheelKa = 0.0003; // V/(RPM/s)
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kFlywheelKs, kFlywheelKv,
      kFlywheelKa);

  // Simulation classes help us simulate our robot

  // Reduction between motors and encoder, as output over input. If the flywheel
  // spins slower than the motors, this number should be greater than one.
  private static final double kFlywheelGearing = 1.0 / 8.45;

  // 1/2 MR²
  private static final double kFlywheelMomentOfInertia = 0.008;

  private final DCMotor m_gearbox = DCMotor.getNEO(1);

  private final LinearSystem<N1, N1, N1> m_plant = LinearSystemId.createFlywheelSystem(m_gearbox, kFlywheelGearing,
      kFlywheelMomentOfInertia);

  private final FlywheelSim m_flywheelSim = new FlywheelSim(m_plant, m_gearbox);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  /** Creates a new BangBangFlywheel. */
  public BangBangFlywheel() {
    // Add bang-bang controller to SmartDashboard and networktables.
    SmartDashboard.putData(m_bangBangController);
  }

  public void setSpeed(AngularVelocity velocity) {
    m_setpointRadiansPerSec = velocity.in(RadiansPerSecond);
  }

  public AngularVelocity getSpeed() {
    return RadiansPerSecond.of(m_encoder.getRate());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Set setpoint and measurement of the bang-bang controller
    double bangOutput = m_bangBangController.calculate(m_encoder.getRate(), m_setpointRadiansPerSec) * 12.0;

    // Controls a motor with the output of the BangBang controller and a
    // feedforward. The feedforward is reduced slightly to avoid overspeeding
    // the shooter.
    final double volts = bangOutput + 0.9 * m_feedforward.calculate(m_setpointRadiansPerSec);
    m_flywheelMotor.setVoltage(volts);

    SmartDashboard.putNumber(
        "Flywheel speed (RPM)",
        Units.radiansPerSecondToRotationsPerMinute(m_encoder.getRate()));
    SmartDashboard.putNumber(
        "Flywheel setpoint (RPM)",
        Units.radiansPerSecondToRotationsPerMinute(m_setpointRadiansPerSec));
    SmartDashboard.putNumber("Flywheel volts", volts);
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated velocities to our simulated encoder
    m_flywheelSim.setInputVoltage(m_flywheelMotor.get() * RobotController.getInputVoltage());
    m_flywheelSim.update(0.02);
    m_encoderSim.setRate(m_flywheelSim.getAngularVelocityRadPerSec());
  }
}
