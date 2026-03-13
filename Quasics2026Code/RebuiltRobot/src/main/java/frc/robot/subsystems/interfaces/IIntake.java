// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Interface for controlling the intake mechanism, used to pick up balls from
 * the floor.
 * 
 */
public interface IIntake {
  /*
   * (2 motors for extension, 1 for rollers)
   * Spin rollers in both directions
   * Extend and retract the intake
   */

  void setRollerSpeed(double speed);

  void stopRoller();

  void setExtensionSpeed(double speed);

  void stopExtension();

  /**
   * Trivial implementation of IIntake, for use on robots that don't have one.
   */
  public class NullIntake extends SubsystemBase implements IIntake {
    @Override
    public void setRollerSpeed(double speed) {
      // No-op.
    }

    @Override
    public void stopRoller() {
      // No-op.
    }

    @Override
    public void setExtensionSpeed(double speed) {
      // No-op.
    }

    @Override
    public void stopExtension() {
      // No-op.
    }
  }
}
