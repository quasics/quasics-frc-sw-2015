package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpScorer;

public class TimedRunAmpScorer extends Command {
  private final AmpScorer m_ampScorer;
  private final double m_ampScorerSpeed;
  Timer m_timer = new Timer();
  private final Time m_time;
  /** Creates a new RunAmpScorer. */
  public TimedRunAmpScorer(AmpScorer ampScorer, double ampScorerSpeed,
      Time time, boolean extending) {
    m_ampScorer = ampScorer;
    m_time = time;
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
    m_timer.reset();
    m_timer.start();
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

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_time.in(Seconds));
  }
}