// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitionRoller;

public class RunTransitionRoller extends Command {
  /** Creates a new RunTransitionRoller. */
  private final TransitionRoller m_transition;
  private final double m_transitionSpeed;

  public RunTransitionRoller(
      TransitionRoller transition, double transitionSpeed, boolean transitionTakingIn) {
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
    m_transition.setTransitionRollerSpeed(m_transitionSpeed);
    System.out.println("Running transition roller!");
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
}
