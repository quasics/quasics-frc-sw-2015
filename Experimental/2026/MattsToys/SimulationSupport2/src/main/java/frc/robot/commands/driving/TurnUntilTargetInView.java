// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driving;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebasePlus;
import frc.robot.subsystems.interfaces.IVision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnUntilTargetInView extends Command {
  private final IVision m_vision;
  private final IDrivebasePlus m_drivebase;
  private final int m_targetId;

  /** Creates a new TurnToTarget. */
  public TurnUntilTargetInView(IVision vision, IDrivebasePlus drivebase, int targetId) {
    m_vision = vision;
    m_drivebase = drivebase;
    m_targetId = targetId;
    addRequirements(m_vision.asSubsystem(), m_drivebase.asSubsystem());
  }

  @Override
  public void initialize() {
    if (m_vision.canSeeTargetWithId(m_drivebase.getEstimatedPose(), m_targetId)) {
      // If we can already see the target, then we don't need to do anything.
      // (This is mostly to avoid a weird edge case where the command gets
      // scheduled, but then the target is immediately visible, which would cause
      // the command to end immediately and thus not run any of the "end" logic
      // that might be important for some commands that extend this one.)
      cancel();
      return;
    }

    // OK, we can't see the target, so let's start turning.
    m_drivebase.driveArcade(0, .10);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  @Override
  public boolean isFinished() {
    System.out.println("Visible targets: " + m_vision.getVisibleTargets(m_drivebase.getEstimatedPose()));
    return m_vision.canSeeTargetWithId(m_drivebase.getEstimatedPose(), m_targetId);
  }
}
