// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulated;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.live.ShooterHoodSubsystem;

/**
 * Sample implementation of the IShooterHood subsystem, for use under
 * simulation. This class simulates the behavior of the shooter hood mechanism,
 * including the motor, absolute encoder, and the physical dynamics of the hood.
 * 
 * The simulation uses a SingleJointedArmSim to model the hood as a
 * single-jointed arm driven by a motor, and updates the simulated encoder
 * readings based on the arm's position and velocity.
 */
public class SimShooterHoodSubsystem extends ShooterHoodSubsystem {
  /** Simulaton support for the SparkMax motor controller. */
  private SparkMaxSim m_motorSim;

  /**
   * Simulation support for the absolute encoder. (This is assumed to be a REV
   * Through-Bore encoder.)
   */
  private SparkAbsoluteEncoderSim m_absEncSim;

  /**
   * Simulation of the physical dynamics of the shooter hood, modeled as a single
   * joint arm driven by a motor. The parameters (gear ratio, moment of inertia,
   * etc.) would need to be tuned based on the actual mechanism design and
   * testing.
   * 
   * The angle limits are set based on the kMinPosDegrees and kMaxPosDegrees
   * constants defined in the IShooterHood interface.
   */
  private SingleJointedArmSim m_hoodSim;

  /** Constructor. */
  public SimShooterHoodSubsystem() {
    super();
    m_motorSim = new SparkMaxSim(m_motor, DCMotor.getNEO(1));
    m_absEncSim = m_motorSim.getAbsoluteEncoderSim();
    m_hoodSim = new SingleJointedArmSim(
        DCMotor.getNEO(1),
        100.0,
        0.01,
        0.2,
        Units.degreesToRadians(kMinPosDegrees),
        Units.degreesToRadians(kMaxPosDegrees),
        true,
        Units.degreesToRadians(kMinPosDegrees));
  }

  @Override
  public void simulationPeriodic() {
    m_hoodSim.setInput(m_motorSim.getBusVoltage() * m_motorSim.getAppliedOutput());
    m_hoodSim.update(0.020);

    // Convert back to degrees before injecting into the SparkMaxSim
    double currentAngleDegrees = Units.radiansToDegrees(m_hoodSim.getAngleRads());
    double currentVelocityDegreesPerSec = Units.radiansToDegrees(m_hoodSim.getVelocityRadPerSec());

    m_absEncSim.setPosition(currentAngleDegrees);
    m_absEncSim.setVelocity(currentVelocityDegreesPerSec);
  }
}
