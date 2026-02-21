// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpScorer;

public class RunAmpScorer extends Command {
  private final AmpScorer m_ampScorer;
  private final double m_ampScorerSpeed;
  /** Creates a new RunAmpScorer. */
  public RunAmpScorer(
      AmpScorer ampScorer, double ampScorerSpeed, boolean extending) {
    m_ampScorer = ampScorer;
    if (extending) {
      m_ampScorerSpeed = -Math.abs(ampScorerSpeed);
    } else {
      m_ampScorerSpeed = Math.abs(ampScorerSpeed);
    } // check neg/pos for extending/not
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ampScorer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ampScorer.setAmpScorerSpeed(m_ampScorerSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ampScorer.setAmpScorerSpeed(m_ampScorerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ampScorer.stop();
  }
}
