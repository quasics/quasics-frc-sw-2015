// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.AbstractDrivebase;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class DriveForTime extends Command {
  /** Creates a new DriveForTime. */

  final AbstractDrivebase m_drivebase;
  final Time m_seconds;
  final ChassisSpeeds m_chassis;

  public DriveForTime(AbstractDrivebase drivebase, Time seconds, ChassisSpeeds chassis) {
    m_drivebase = drivebase;
    m_seconds = seconds;
    m_chassis = chassis;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateSpeeds();
    Timer.delay(m_seconds.in(Seconds));
    end(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateSpeeds();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  final void updateSpeeds() {
    m_drivebase.setSpeeds(m_chassis);
  }
}
