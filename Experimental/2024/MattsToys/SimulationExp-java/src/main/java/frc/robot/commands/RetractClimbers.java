// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.Mode;

/**
 * Simple command to trigger climber retraction. Will signal that it's finished
 * when the subsystem reports that both climbing arms are in the "stopped"
 * state.
 * 
 * TODO: Test this code.
 */
public class RetractClimbers extends Command {
  final Climber m_climber;

  public RetractClimbers(Climber climber) {
    m_climber = climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_climber.retractLeftClimber()) {
      System.err.println("*** Warning: couldn't start retracting left climber!");
    }
    if (!m_climber.retractRightClimber()) {
      System.err.println("*** Warning: couldn't start retracting right climber!");
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  @Override
  public boolean isFinished() {
    return m_climber.getLeftMode() == Mode.Stopped
        && m_climber.getRightMode() == Mode.Stopped;
  }
}
