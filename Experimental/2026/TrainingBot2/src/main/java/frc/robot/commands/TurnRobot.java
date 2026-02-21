// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;

/**
 * Makes the robot turn by a certain angle (degrees, radians, whatever), at a
 * given speed.
 */
public class TurnRobot extends Command {
  final AbstractDrivebase m_drivebase;
  final double m_percentSpeed;
  final Angle m_turnAngle;

  /**
   * Creates a new TurnRobot.
   * 
   * Note that we need to make sure that we handle cases where (for instance) the
   * speed is negative, but the angle is positive (or vice versa). This is left as
   * an exercise for the student....
   */
  public TurnRobot(AbstractDrivebase drivebase, double percentSpeed, Angle turnAngle) {
    m_drivebase = drivebase;
    m_percentSpeed = percentSpeed;
    m_turnAngle = turnAngle;

    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
