// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AbstractDrivebase;
import edu.wpi.first.wpilibj2.command.Command;

/** A simple command to test the drivebase (by driving forward). */
public class LinearSpeedCommand extends Command {
  private final AbstractDrivebase m_drivebase;

  /**
   * Creates a new o.
   *
   * @param drivebase The subsystem used by this command.
   */
  public LinearSpeedCommand(AbstractDrivebase drivebase) {
    m_drivebase = drivebase;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  // FINDME(Robert): Do you need this? If not, then it should probably be removed,
  // in order to simplify your code. (It's auto-generated when you create a new
  // command, to make things easier for you if you *do* need it, but you can get
  // rid of it if it doesn't do anything.)

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.setSpeeds(1, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.setSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
