// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.implementation.SingleMotorThing;

/**
 * An example of how the subsystem might be set up under simulation, but still
 * use the same base class as the "live" hardware.
 */
public class SingleMotorThingSim extends SingleMotorThing {
  /** Arbitrary value for under simulation. */
  private double MAX_SPEED_METERS_PER_SEC = 5;

  /** Encoder ticks per revolution. */
  private static final int ENCODER_TICKS_PER_REVOLUTION = -4096;

  /** Wheel diameter in inches. */
  private static final Distance WHEEL_DIAMETER = Inches.of(6);

  /**
   * Our own, direct access to the motor controller (vs. the base class's
   * reference to it).
   */
  private final PWMSparkMax m_motor;

  /**
   * Our own, direct access to the encoder (vs. the base class's access to it via
   * a TrivialEncoder).
   */
  private final Encoder m_encoder;

  /**
   * The EncoderSim that we'll use to update the encoder.
   */
  private final EncoderSim m_encoderSim;

  /** Helper function to configure the encoder, based on an assumed wheel size. */
  protected static void configureEncoderForDistance(Encoder encoder, Distance outerDiameter) {
    encoder.setDistancePerPulse(Math.PI * WHEEL_DIAMETER.in(Meters) / ENCODER_TICKS_PER_REVOLUTION);
  }

  /** Data we use to actually create one of these things. */
  private record SimulationClassData(PWMSparkMax controller, Encoder encoder, EncoderSim encoderSim) {
  }

  /** Helper function used to allocate the underlying controllers, etc. */
  private static SimulationClassData allocateSimulationData() {
    PWMSparkMax controller = new PWMSparkMax(1);
    Encoder encoder = new Encoder(0, 1);
    configureEncoderForDistance(encoder, WHEEL_DIAMETER);
    EncoderSim encoderSim = new EncoderSim(encoder);
    return new SimulationClassData(controller, encoder, encoderSim);
  }

  /**
   * Basic contructor, externally visible.
   * 
   * Note that the real work is done by the *other* (private) constructor.
   * 
   * @see #SingleMotorThingSim(SimulationClassData)
   */
  public SingleMotorThingSim() {
    this(allocateSimulationData());
  }

  /**
   * Constructor that does the "heavy lifting" for setup.
   * 
   * We need to go through this "dance" because the base class part gets built
   * before *this* class does, which means that it needs to be given the motor
   * controller and trivial encoder (wrapped around the real encoder) before we
   * can get stuff done in this class's code. But we will need to have access to
   * the motor controller and the (raw) trivial encoder in order to set up
   * simulation support.
   * 
   * By allocating/configuring stuff in the helper function, we can then use that
   * data to set up the base class, *and* still have direct access to the "raw
   * stuff" that's needed to handle simulating things.
   * 
   * @param data the SimulationClassData (controller, encoder, encoder simulation
   *             hook) used to set up the object
   */
  private SingleMotorThingSim(SimulationClassData data) {
    // Build our base class part, using the data we're given
    super(new ConstructionData(data.controller, TrivialEncoder.forWpiLibEncoder(data.encoder, data.encoderSim)));

    // Hang onto the "raw pieces" for use in simulationPeriodic()
    m_motor = data.controller;
    m_encoder = data.encoder;
    m_encoderSim = data.encoderSim;
  }

  //
  // Methods from SubsystemBase
  //

  // We do simulation things here...
  @Override
  public void simulationPeriodic() {
    // The standard loop time is 20ms.
    final double timeIncrement = 0.020;

    // What are the current readings?
    double percentSpeed = m_motor.get();
    double initialPosition = m_encoder.getDistance();

    // How far would we move in one cycle of the loop? (Note that we're assuming
    // instantaneous acceleration, but this is a *simple* simulation.... :-)
    double currentSpeedMetersPerSec = percentSpeed * MAX_SPEED_METERS_PER_SEC;
    double distanceTravelled = currentSpeedMetersPerSec * timeIncrement;

    // OK, update the encoder (by injecting data via the encoderSim)
    m_encoderSim.setDistance(initialPosition + distanceTravelled);
    m_encoderSim.setRate(currentSpeedMetersPerSec);
  }
}
