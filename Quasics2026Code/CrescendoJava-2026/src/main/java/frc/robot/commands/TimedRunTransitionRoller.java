// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitionRoller;

public class TimedRunTransitionRoller extends Command {
  /** Creates a new TimedRunTransitionRoller. */
  private final TransitionRoller m_transition;
  private final double m_transitionSpeed;
  Timer m_timer = new Timer();
  private final Time m_time;
  public TimedRunTransitionRoller(TransitionRoller transition,
      double transitionSpeed, Time time, boolean transitionTakingIn) {
    m_time = time;
    m_transition = transition;
    if (transitionTakingIn) {
      m_transitionSpeed = Math.abs(transitionSpeed);
    } else {
      m_transitionSpeed = -Math.abs(transitionSpeed);
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(transition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_transition.setTransitionRollerSpeed(m_transitionSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_transition.setTransitionRollerSpeed(m_transitionSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transition.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_time.in(Seconds));
  }
}
