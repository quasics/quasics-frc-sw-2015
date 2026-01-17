// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.Subsystem;
import java.io.Closeable;

/**
 * Basic interface for subsystems.
 *
 * I like using a pure interface for subsystems, rather than extending
 * SubsystemBase directly, because it helps to separate the "interface" from the
 * "implementation" and allows for more flexible designs. It also allows me to
 * define wrapper types that can add functionality to existing subsystems
 * without modifying their core behavior, and without requiring a direct
 * inheritance relationship.
 */
public interface ISubsystem extends Closeable {
  /** Returns a Subsystem instance for this interface. */
  default Subsystem asSubsystem() {
    return (Subsystem) this;
  }
}
