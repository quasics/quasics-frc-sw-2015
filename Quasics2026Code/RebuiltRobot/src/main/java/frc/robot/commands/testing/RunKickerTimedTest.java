// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IShooter;

/**
 * Simple timed test of the kicker, for use in system testing.
 */
public class RunKickerTimedTest extends TimedTestBase {
  final IShooter m_shooter;
  final double m_kickerSpeed;

  /**
   * Constructor.
   * 
   * @param shooter      subsystem under test
   * @param shooterSpeed target speed
   * @param duration     test duration
   */
  public RunKickerTimedTest(IShooter shooter, double kickerSpeed, Time duration) {
    super(duration);
    m_shooter = shooter;
    m_kickerSpeed = kickerSpeed;
    addRequirements((Subsystem) m_shooter);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_shooter.setKickerSpeed(m_kickerSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_shooter.stop();
  }
}
