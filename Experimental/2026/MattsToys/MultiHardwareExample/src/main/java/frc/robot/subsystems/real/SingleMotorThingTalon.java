// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.util.EncoderSupport.configureEncoderForDistance;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.implementation.SingleMotorThing;

/**
 * Provides a simple example of a "SingleMotorThing" that uses Talon controllers.
 *
 * Note that all of the "actual functionality" takes place in the base class; this class only exists
 * to set up the hardware-specific stuff.
 */
public class SingleMotorThingTalon extends SingleMotorThing {
  /** Wheel diameter on this implementation of a SingleMotorThing. */
  final static Distance WHEEL_DIAMETER = Inches.of(4);

  /** Encoder ticks/revolution for this hardware. */
  final static double ENCODER_TICKS_PER_REVOLUTION = 4096;

  /**
   * Builds the actual hardware wrappers that will be passed to the base class.
   */
  static DerivedClassData getStuffForBaseClassSetup() {
    final Encoder rawEncoder = new Encoder(3, 4);
    configureEncoderForDistance(rawEncoder, WHEEL_DIAMETER, ENCODER_TICKS_PER_REVOLUTION);

    return new DerivedClassData(
        // Motor controller
        new PWMTalonFX(6),
        // Encoder
        TrivialEncoder.forWpiLibEncoder(rawEncoder));
  }

  /** Creates a new SingleMotorThingTalon. */
  public SingleMotorThingTalon() {
    super(getStuffForBaseClassSetup());
  }
}
