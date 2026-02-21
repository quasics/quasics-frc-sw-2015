// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransitionRoller;

public class RunTransitionUntilBeamBroken extends Command {
  private final TransitionRoller m_transition;
  private final double m_transitionSpeed;
  private final boolean m_enableSensor;

  /** Creates a new RunTransitionUntilBeamBroken. */
  public RunTransitionUntilBeamBroken(TransitionRoller transitionRoller,
      double transitionSpeed, boolean transitionTakingIn,
      boolean enableSensor) {
    m_transition = transitionRoller;
    if (transitionTakingIn) {
      m_transitionSpeed = Math.abs(transitionSpeed);
    } else {
      m_transitionSpeed = -Math.abs(transitionSpeed);
    }
    m_enableSensor = enableSensor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(transitionRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Running transition roller until beam broken!");
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
    m_transition.setTransitionRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_enableSensor) {
      return !m_transition.input.get();
    }
    return false;
  }
}
