// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;

public class DriveForDistance extends Command {
  final private AbstractDrivebase m_drivebase;

  /** Creates a new DriveForDistance. */
  public DriveForDistance(AbstractDrivebase drivebase, Measure<Distance> distanceToTravel,
      Measure<Velocity<Distance>> speed) {
    m_drivebase = drivebase;
    m_distanceToTravel = distanceToTravel;
    m_speed = speed;
    addRequirements(m_drivebase);
  }

  private Measure<Distance> m_goal;
  final private Measure<Distance> m_distanceToTravel;
  final private Measure<Velocity<Distance>> m_speed;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_goal = m_drivebase.getLeftDistance().plus(m_distanceToTravel);
    m_drivebase.arcadeDrive(m_speed, RadiansPerSecond.of(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.arcadeDrive(m_speed, RadiansPerSecond.of(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivebase.getLeftDistance().gte(m_goal);
  }
}
