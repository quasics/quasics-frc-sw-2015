// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

import java.util.List;

/**
 * Lighting subsystem configuration data.
 *
 * @param pwmPort     the PWM port driving the LED strip
 * @param stripLength the length (in pixels/cells) of the LED strip
 * @param subViews    list of subviews for the strip (segments following the
 *                    "main set")
 *
 * @see frc.robot.subsystems.live.Lighting
 * @see frc.robot.subsystems.live.LightingBuffer
 */
public record
    LightingConfig(int pwmPort, int stripLength, List<Integer> subViews) {
  /**
   * Constructor (with sanity checking).
   *
   * @param pwmPort     the PWM port driving the LED strip
   * @param stripLength the length (in pixels/cells) of the LED strip
   * @param subViews    list of subviews for the strip (segments following the
   *                    "main set")
   */
  public LightingConfig {
    if (subViews != null) {
      final int subViewTotalSize =
          subViews.stream().mapToInt(Integer::intValue).sum();
      if (subViewTotalSize > stripLength) {
        throw new IllegalArgumentException("Sub-view size (" + subViewTotalSize
            + ") exceeds strip length (" + stripLength + ")");
      }
    }
  }

  /**
   * Convenience constructor.
   *
   * @param pwmPort     the PWM port driving the LED strip
   * @param stripLength the length (in pixels/cells) of the LED strip
   */
  public LightingConfig(int pwmPort, int stripLength) {
    this(pwmPort, stripLength, null);
  }
}