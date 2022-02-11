// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * A sample class for handling "turbo, turtle, or normal mode" speed-limiting.
 */
public class TurboTurtleScaler implements SpeedModifier {

  /**
   * Interface for lambdas used to signal if a given speed mode should be enabled.
   */
  public interface ModeSignaller {
    boolean enabled();
  }

  private final SpeedModifier normal;
  private final SpeedModifier turtle;
  private final SpeedModifier turbo;
  private final ModeSignaller turtleMode;
  private final ModeSignaller turboMode;

  public TurboTurtleScaler(
      SpeedModifier normal,
      SpeedModifier turtle,
      SpeedModifier turbo,
      ModeSignaller turtleMode,
      ModeSignaller turboMode) {
    this.normal = normal;
    this.turtle = turtle;
    this.turbo = turbo;
    this.turtleMode = turtleMode;
    this.turboMode = turboMode;
  }

  @Override
  public double adjustSpeed(double inputPercentage) {
    if (turtleMode.enabled()) {
      return turtle.adjustSpeed(inputPercentage);
    } else if (turboMode.enabled()) {
      return turbo.adjustSpeed(inputPercentage);
    } else {
      return normal.adjustSpeed(inputPercentage);
    }
  }
}
