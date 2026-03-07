// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ratios;
import frc.robot.Constants.Tolerances;
import frc.robot.subsystems.interfaces.IShooterHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotHoodToPosition extends Command {

  IShooterHood m_hood;
  private double m_speed;
  private Angle m_endAngle;

  /** Creates a new PivotHoodToPosition. */
  public PivotHoodToPosition(IShooterHood hood, double speed, Angle endAngle) {

    m_hood = hood;

    m_speed = speed;

    m_endAngle = endAngle.times(Ratios.ENCODERTOHOODRATIO);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((SubsystemBase) hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_hood.getCurrentAngle().gt(m_endAngle.minus(Tolerances.ANGLETOLERANCE))) {

      m_hood.moveDown(m_speed);

    } else if (m_hood.getCurrentAngle().lt(m_endAngle.plus(Tolerances.ANGLETOLERANCE))) {

      m_hood.moveUp(m_speed);

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_hood.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (m_hood.getCurrentAngle().lt(m_endAngle.plus(Tolerances.ANGLETOLERANCE)) &&
        m_hood.getCurrentAngle().gt(m_endAngle.minus(Tolerances.ANGLETOLERANCE)));

  }
}
