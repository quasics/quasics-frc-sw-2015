// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;

/** Reusable helper functions for working with (native) WPILib encoders. */
public final class EncoderSupport {
  /**
   * Configures a WPILib encoder to report distances in meters.
   *
   * @param encoder encoder being configured
   * @param outerDiameter outer diameter of the wheel being turned
   * @param ticksPerRevolution encoder ticks for a full revolution
   */
  public static void configureEncoderForDistance(
      Encoder encoder, Distance outerDiameter, double ticksPerRevolution) {
    encoder.setDistancePerPulse(Math.PI * outerDiameter.in(Meters) / ticksPerRevolution);
  }
}
