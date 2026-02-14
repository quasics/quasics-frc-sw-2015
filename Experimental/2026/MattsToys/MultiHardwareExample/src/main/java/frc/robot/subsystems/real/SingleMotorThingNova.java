// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import static edu.wpi.first.units.Units.Inches;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig;

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
  static ConstructionData getStuffForBaseClassSetup(int deviceID, boolean inverted) {
    ThriftyNova motorController = new ThriftyNova(deviceID); // , ThriftyNova.MotorType.NEO
    ThriftyNovaConfig config = new ThriftyNovaConfig();

    config.encoderType = EncoderType.INTERNAL; // Built-in NEO encoder
    config.inverted = inverted;
    motorController.applyConfig(config);

    return new ConstructionData(motorController,
        new ThriftyEncoderWrapper(motorController, WHEEL_DIAMETER));
  }

  public SingleMotorThingNova(int deviceID) {
    this(deviceID, false);
  }

  public SingleMotorThingNova(int deviceID, boolean inverted) {
    super(getStuffForBaseClassSetup(deviceID, inverted));
    System.out.println("Set up SingleMotorThingNova!");
  }
}
