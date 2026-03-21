// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IIndexer;

/**
 * Simple timed test of the indexer, for use in system testing.
 */
public class RunIndexerTimedTest extends TimedTestBase {
  final IIndexer m_indexer;
  final double m_indexerSpeed;

  /**
   * Constructor.
   * 
   * @param indexer      subsystem under test
   * @param indexerSpeed target speed
   * @param duration     test duration
   */
  public RunIndexerTimedTest(IIndexer indexer, double indexerSpeed, Time duration) {
    super(duration);
    m_indexer = indexer;
    m_indexerSpeed = indexerSpeed;
    addRequirements((Subsystem) m_indexer);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_indexer.setIndexSpeed(m_indexerSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_indexer.stopIndex();
  }
}
