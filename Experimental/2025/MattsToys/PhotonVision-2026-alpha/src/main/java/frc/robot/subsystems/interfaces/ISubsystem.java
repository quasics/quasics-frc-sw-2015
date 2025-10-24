// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Simple interface to be used as a basis for defining subsystem-specific
 * interfaces.
 */
public interface ISubsystem {
  /**
   * Returns the subsystem for the object (usually expected to just be a cast).
   * 
   * @return the object cast to a subsystem (for listing in requirements).
   */
  default Subsystem asSubsystem() {
    return (Subsystem) this;
  }
}
