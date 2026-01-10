// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.util.EncoderSupport.configureEncoderForDistance;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.implementation.SingleMotorThing;

/**
 * Provides a simple example of a "SingleMotorThing" that uses Spark controllers.
 *
 * Note that all of the "actual functionality" takes place in the base class; this class only exists
 * to set up the hardware-specific stuff.
 */
public class SingleMotorThingSpark extends SingleMotorThing {
  /** Encoder ticks per revolution. */
  public static final int ENCODER_TICKS_PER_REVOLUTION = -4096;

  /** Wheel diameter in inches. */
  public static final Distance WHEEL_DIAMETER = Inches.of(6);

  static DerivedClassData getStuffForBaseClassSetup() {
    Encoder rawEncoder = new Encoder(3, 4);
    configureEncoderForDistance(rawEncoder, WHEEL_DIAMETER, ENCODER_TICKS_PER_REVOLUTION);

    return new DerivedClassData(
        // TODO: Replace with CAN-based SparkMax stuff (when it works on Mac)
        new PWMSparkMax(1),
        // TODO: Replace with a wrapped RelativeEncoder, when we've moved to CAN controller
        TrivialEncoder.forWpiLibEncoder(rawEncoder));
  }

  /** Creates a new SingleMotorThingSpark. */
  public SingleMotorThingSpark() {
    super(getStuffForBaseClassSetup());
  }
}
