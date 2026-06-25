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

public interface IKicker {
  void stopKicker();

  /*
   * (1 motor)
   * do notes lter
   *
   * Need to talk to folks about limit switches, etc.
   */

  default void stop() {
    stopKicker();
  }

  double getKickerPosition();

  void setKickSpeed(double speed);

  /**
   * Trivial implementation of IKicker, for use when we're on a robot without
   * one.
   */
  public class NullKicker extends SubsystemBase implements IKicker {
    // public NullClimber() {
    @Override
    public void stopKicker() {
    }

    @Override
    public void setKickSpeed(double speed) {
    }

    @Override
    public double getKickerPosition() {
      return 0;
    }

  }

}
