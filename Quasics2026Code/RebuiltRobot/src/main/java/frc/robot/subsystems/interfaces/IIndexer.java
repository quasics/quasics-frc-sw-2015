// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Interface for controlling the indexing of balls (between the intake and
 * shooter).
 * 
 * per sarima: 1 neo 550
 * 
 * TODO: Define the indexer interface (and then implement it).
 */
public interface IIndexer {
  /*
   * Spin rollers in both directions (1 motor)
   */

  void setIndexSpeed(double speed);

  void stopIndex();

  /**
   * Trivial implementation of IIndexer, for use on robots that don't have one.
   */
  public class NullIndexer extends SubsystemBase implements IIndexer {
    @Override
    public void setIndexSpeed(double speed) {
      // No-op.
    }

    @Override
    public void stopIndex() {
      // No-op.
    }
  }
}
