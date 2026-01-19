// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware.actuators;

import static edu.wpi.first.units.Units.Volts;

import com.thethriftybot.devices.ThriftyNova;
import java.io.Closeable;
import java.io.IOException;

/** IMotorControllerPlus specialization for Thrifty Nova motors. */
public class ThriftyNovaMotorControllerPlus
    extends IMotorControllerPlus.MotorControllerPlus {
  /**
   * Constructor.
   * 
   * @param controller the motor controller being wrapped
   */
  public ThriftyNovaMotorControllerPlus(ThriftyNova controller) {
    super(controller, () -> Volts.of(controller.getVoltage()), new Closeable() {
      public void close() throws IOException {
        try {
          controller.close();
        } catch (Exception e) {
          throw new IOException(e);
        }
      }
    });
  }
}
