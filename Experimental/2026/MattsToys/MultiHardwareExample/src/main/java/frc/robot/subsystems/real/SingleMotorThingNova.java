// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import static edu.wpi.first.units.Units.Inches;

import com.thethriftybot.devices.ThriftyNova;
import edu.wpi.first.units.measure.Distance;
import frc.robot.sensors.ThriftyEncoderWrapper;
import frc.robot.subsystems.implementation.SingleMotorThing;

/**
 * Provides a simple example of a "SingleMotorThing" that uses Thrifty Nova
 * controllers.
 *
 * Note that all of the "actual functionality" takes place in the base class;
 * this class only exists to set up the hardware-specific stuff.
 */
public class SingleMotorThingNova extends SingleMotorThing {
  /** Wheel diameter in inches. */
  public static final Distance WHEEL_DIAMETER = Inches.of(6);

  /**
   * Builds the actual hardware wrappers that will be passed to the base class.
   */
  static DerivedClassData getStuffForBaseClassSetup() {
    ThriftyNova motorController = new ThriftyNova(1, ThriftyNova.MotorType.NEO);
    return new DerivedClassData(
        motorController, new ThriftyEncoderWrapper(motorController, WHEEL_DIAMETER));
  }

  public SingleMotorThingNova() {
    super(getStuffForBaseClassSetup());
  }
}
