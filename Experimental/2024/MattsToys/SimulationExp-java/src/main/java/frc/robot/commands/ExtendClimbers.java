// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.Mode;

/**
 * Simple command to trigger climber extension. Will signal that it's finished
 * when the subsystem reports that both climbing arms are in the "stopped"
 * state.
 * 
 * TODO: Test this code.
 */
public class ExtendClimbers extends Command {
  final Climber m_climber;

  /** Creates a new ExtendClimbers. */
  public ExtendClimbers(Climber climber) {
    m_climber = climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_climber.extendLeftClimber()) {
      System.err.println("*** Warning: couldn't start extending left climber!");
    }
    if (!m_climber.extendRightClimber()) {
      System.err.println("*** Warning: couldn't start extending right climber!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.getLeftMode() == Mode.Stopped
        && m_climber.getRightMode() == Mode.Stopped;
  }
}
