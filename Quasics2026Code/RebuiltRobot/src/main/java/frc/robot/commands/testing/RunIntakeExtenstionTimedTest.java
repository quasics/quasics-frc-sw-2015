// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IIntake;

/**
 * Simple timed test of the intake extension, for use in system testing.
 */
public class RunIntakeExtenstionTimedTest extends TimedTestBase {
  final IIntake m_intake;
  final double m_extensionSpeed;

  /**
   * Constructor.
   * 
   * @param shooter        subsystem under test
   * @param extensionSpeed target speed
   * @param duration       test duration
   */
  public RunIntakeExtenstionTimedTest(IIntake intake, double extensionSpeed, Time duration) {
    super(duration);
    m_intake = intake;
    m_extensionSpeed = extensionSpeed;
    addRequirements((Subsystem) m_intake);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_intake.setExtensionSpeed(m_extensionSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_intake.stopExtension();
  }
}
