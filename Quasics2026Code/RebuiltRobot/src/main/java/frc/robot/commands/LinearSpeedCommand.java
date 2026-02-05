// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AbstractDrivebase;
import edu.wpi.first.wpilibj2.command.Command;

/** A simple command to test the drivebase (by driving forward). */
public class LinearSpeedCommand extends Command {
  private final AbstractDrivebase m_drivebase;
  private final double m_percentSpeed;

  /**
   * Creates a new LinearSpeedCommand, with a default speed of 100% of max.
   *
   * @param drivebase The subsystem used by this command.
   */
  public LinearSpeedCommand(AbstractDrivebase drivebase) {
    this(drivebase, 0.2);
  }

  /**
   * Creates a new LinearSpeedCommand, with the specified % of max speed to be
   * used while running.
   *
   * @param drivebase The subsystem used by this command.
   */
  public LinearSpeedCommand(AbstractDrivebase drivebase, double percentSpeed) {
    m_drivebase = drivebase;
    m_percentSpeed = percentSpeed;
    addRequirements(drivebase);
  }

  @Override
  public void execute() {
    m_drivebase.setSpeeds(m_percentSpeed, m_percentSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
}
