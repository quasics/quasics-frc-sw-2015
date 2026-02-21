// Copyright (c) 2024-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;

/**
 * Helper class to wrap a Pigeon2 device in an IGyro interface.
 *
 * This used to be included in the IGyro type definition, but has been extracted
 * in order to simplify reuse of that type in projects that don't use a Pigeon2.
 */
public class Pigeon2Wrapper extends IGyro.FunctionalGyro {
  /**
   * Constructor.
   *
   * @param pigeon2 the device to be wrapped in an IGyro interface
   */
  public Pigeon2Wrapper(Pigeon2 pigeon2) {
    super(
        () -> {
          System.out.println(">>> Null-op: Pigeon2 auto-calibrates.");
        },
        // Per docs, "getAngle()" (CW+) has been replaced by "getYaw()" (CCW+).
        () -> Degrees.of(pigeon2.getYaw().getValueAsDouble()),
        // Per docs, "getRate()" (CW+) has been replaced by
        // "getAngularVelocityZWorld()" (CCW+).
        () -> DegreesPerSecond.of(
            pigeon2.getAngularVelocityZWorld().getValueAsDouble()),
        () -> pigeon2.getRotation2d().unaryMinus(),
        () -> {
          // Note that this will reset *all* axes for the Pigeon2. May want
          // to wrap further in an OffsetGyro.
          pigeon2.reset();
        },
        () -> {
          pigeon2.close();
        });
  }

  /**
   * Convenience function to produce a wrapped Pigeon2.
   *
   * @param g a Pigeon2 to be wrapped
   * @return an IGyro object providing access to the underlying Pigeon2
   */
  static IGyro wrapGyro(Pigeon2 g) {
    return new Pigeon2Wrapper(g);
  }
}
