// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IShooter;

/**
 * Simple timed test of the flywheel, for use in system testing.
 */
public class RunFlywheelTimedTest extends TimedTestBase {
  final IShooter m_shooter;
  final AngularVelocity m_shooterSpeed;

  /**
   * Constructor.
   * 
   * @param shooter      subsystem under test
   * @param shooterSpeed target speed
   * @param duration     test duration
   */
  public RunFlywheelTimedTest(IShooter shooter, AngularVelocity shooterSpeed, Time duration) {
    super(duration);
    m_shooter = shooter;
    m_shooterSpeed = shooterSpeed;
    addRequirements((Subsystem) m_shooter);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_shooter.setFlywheelRPM(m_shooterSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_shooter.stop();
  }
}
