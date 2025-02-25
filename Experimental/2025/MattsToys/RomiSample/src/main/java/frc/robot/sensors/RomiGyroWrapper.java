// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.romi.RomiGyro;

/**
 * Helper class to wrap an XRPGyro device in an IGyro interface.
 *
 * This used to be included in the IGyro type definition, but has been extracted
 * in order to simplify reuse of that type in projects that don't use XRP
 * hardware.
 */
public class RomiGyroWrapper extends IGyro.FunctionalGyro {
  public RomiGyroWrapper(RomiGyro g) {
    super(()
              -> { System.out.println(">>> Null-op: XRPGyro doesn't calibrate."); },
        ()
            -> Degrees.of(g.getAngleZ()),
        ()
            -> DegreesPerSecond.of(g.getRateZ()),
        ()
            -> { return Rotation2d.fromDegrees(g.getAngleZ()); },
        ()
            -> {
                // Note that this won't actually get invoked, because the OffsetGyro will
                // instead just reset its offset value.
            });
  }

  static IGyro wrapGyro(RomiGyro g) {
    return new RomiGyroWrapper(g);
  }
}
