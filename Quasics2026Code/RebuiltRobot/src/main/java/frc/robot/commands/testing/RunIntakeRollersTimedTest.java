// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IIntake;

/**
 * Simple timed test of the intake rollers, for use in system testing.
 */
public class RunIntakeRollersTimedTest extends TimedTestBase {
  final IIntake m_intake;
  final double m_rollerSpeed;

  /**
   * Constructor.
   * 
   * @param shooter     subsystem under test
   * @param rollerSpeed target speed
   * @param duration    test duration
   */
  public RunIntakeRollersTimedTest(IIntake intake, double rollerSpeed, Time duration) {
    super(duration);
    m_intake = intake;
    m_rollerSpeed = rollerSpeed;
    addRequirements((Subsystem) m_intake);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_intake.setRollerSpeed(m_rollerSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_intake.stopRoller();
  }
}
