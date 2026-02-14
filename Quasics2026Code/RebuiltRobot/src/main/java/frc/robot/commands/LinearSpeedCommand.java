// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.real.AbstractDrivebase;

/** A simple command to test the drivebase (by driving forward). */
public class LinearSpeedCommand extends Command {
  private final AbstractDrivebase m_drivebase;
  private final double m_percentSpeed;

  public final static double DEFAULT_SPEED_PERCENT = 0.2;

  /**
   * Creates a new LinearSpeedCommand, with a default speed of
   * DEFAULT_SPEED_PERCENT of the supported maximum.
   *
   * @param drivebase The subsystem used by this command.
   */
  public LinearSpeedCommand(AbstractDrivebase drivebase) {
    this(drivebase, DEFAULT_SPEED_PERCENT);
  }

  /**
   * Creates a new LinearSpeedCommand, with the specified % of max speed to be
   * used while running.
   *
   * @param drivebase    The subsystem used by this command.
   * @param percentSpeed How fast this command should move.
   *                     TODO: Add a STOP condition and take a DISTANCE parameter
   */
  public LinearSpeedCommand(AbstractDrivebase drivebase, double percentSpeed) {
    // TODO(Robert): Decide if we want this to take a speed (LinearVelocity) or a
    // percent.
    // We initially wrote it to take a speed.
    // This is a good lesson for both of us on why it's important not to use magic
    // numbers - It's ambiguous!
    m_drivebase = drivebase;
    m_percentSpeed = percentSpeed;
    addRequirements(drivebase);
  }

  @Override
  public void execute() {
    // TODO(ROBERT): This is not a speed! It is a percent.
    m_drivebase.setSpeeds(m_percentSpeed, m_percentSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
}
