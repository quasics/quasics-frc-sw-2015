// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Interface for controlling the climbing hardware.
 *
 * TODO: Define the climber interface (and then implement it).
 */
public interface IClimber {
  /*
   * (1 motor)
   * Extend and retract the climber
   *
   * Need to talk to folks about limit switches, etc.
   */

  /**
   * Trivial implementation of IClimber, for use when we're on a robot without
   * one.
   */
  public class NullClimber extends SubsystemBase implements IClimber {
    public NullClimber() {
    }
  }
}
