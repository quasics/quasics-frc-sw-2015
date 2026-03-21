// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IClimber;

/**
 * Simple timed test of the climber, for use in system testing.
 */
public class RunClimberTimedTest extends TimedTestBase {
  final IClimber m_climber;
  final double m_climberSpeed;

  /**
   * Constructor.
   * 
   * @param climber      subsystem under test
   * @param climberSpeed target speed
   * @param duration     test duration
   */
  public RunClimberTimedTest(IClimber climber, double climberSpeed, Time duration) {
    super(duration);
    m_climber = climber;
    m_climberSpeed = climberSpeed;
    addRequirements((Subsystem) m_climber);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_climber.setClimberSpeed(m_climberSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_climber.stop();
  }
}
