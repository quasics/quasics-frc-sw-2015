// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
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
  private double m_howFarAwayInMeters; // how far away do we want to stop
  final double m_percentSpeed;
  Distance m_stopPosition;
  private double m_rangeInMeters;

  /** Creates a new DriveToTarget. */
  public DriveToTarget(AbstractDrivebase drivebase, Vision vision, int targetID, double howFarAwayInMeters,
      double percentSpeed) {
    m_drivebase = drivebase;
    m_vision = vision;
    m_targetID = targetID;
    m_howFarAwayInMeters = howFarAwayInMeters;
    m_percentSpeed = percentSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_finished = false;
    m_rangeInMeters = m_vision.getTargetRange(m_targetID);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_rangeInMeters);
    m_stopPosition = m_drivebase.getLeftPosition().plus(Meters.of(m_rangeInMeters))
        .minus(Meters.of(m_howFarAwayInMeters));
    if (m_rangeInMeters == 0.0) {
      System.out
          .println("quite literally on top of target or cannot see target \n try aim at target before running again");
      m_finished = true;
    }
    if (m_rangeInMeters != 0.0) {
      m_drivebase.setSpeeds(m_percentSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished || m_drivebase.getLeftPosition().gte(m_stopPosition);
  }
}
