// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.AbstractDrivebase;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToTarget extends Command {
  private final AbstractDrivebase m_drivebase;
  private final Vision m_vision;
  private boolean m_finished = false;
  private int m_targetID;
  private double m_howFarAway;

  /** Creates a new DriveToTarget. */
  public DriveToTarget(AbstractDrivebase drivebase, Vision vision, int targetID, double howFarAway) {
    m_drivebase = drivebase;
    m_vision = vision;
    m_targetID = targetID;
    m_howFarAway = howFarAway;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LinearVelocity speed = MetersPerSecond.of(10);

    m_drivebase.arcadeDrive(speed, null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
