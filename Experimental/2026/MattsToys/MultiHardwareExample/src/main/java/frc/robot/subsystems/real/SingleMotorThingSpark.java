// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

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
  static DerivedClassData getStuffForBaseClassSetup() {
    return new DerivedClassData(
        // TODO: Replace with CAN-based SparkMax stuff (when it works on Mac)
        new PWMSparkMax(1),
        // TODO: Replace with a wrapped RelativeEncoder, when we've moved to CAN controller
        // TODO: Configure the encoder
        TrivialEncoder.forWpiLibEncoder(new Encoder(3, 4)));
  }

  /** Creates a new SingleMotorThingSparkMax. */
  public SingleMotorThingSpark() {
    super(getStuffForBaseClassSetup());
  }
}
