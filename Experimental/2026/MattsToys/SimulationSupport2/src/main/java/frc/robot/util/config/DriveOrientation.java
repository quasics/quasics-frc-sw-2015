// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

/** Orientation of the motors on the drive base. */
public enum DriveOrientation {
  /** Neither left/right motors are inverted. */
  NotInverted,
  /** Both Left/right motors are inverted. */
  FullyInverted,
  /** Right motor (only) is inverted. */
  RightInverted,
  /** Left motor (only) is inverted. */
  LeftInverted,
  ;

  /** @return true iff the motors on the right side are inverted */
  public boolean isRightInverted() {
    return this == FullyInverted || this == RightInverted;
  }

  /** @return true iff the motors on the left side are inverted */
  public boolean isLeftInverted() {
    return this == FullyInverted || this == LeftInverted;
  }
}