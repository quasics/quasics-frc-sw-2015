// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * Utility class to provide a simple (fixed) linear scaling of the drive speed
 * (e.g., so as to cap maximums at x%, and keep drivers from doing something
 * crazy :-).
 */
public class SpeedScaler implements SpeedModifier {
  final double scalingFactor;

  /**
   * Constructor.
   *
   * @param scalingFactor the linear scaling factor to be applied to inputs to
   *                      getSpeed().
   */
  public SpeedScaler(double scalingFactor) {
    this.scalingFactor = scalingFactor;
  }

  @Override
  public double adjustSpeed(double inputPercentage) {
    return inputPercentage * scalingFactor;
  }
}
