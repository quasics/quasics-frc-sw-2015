// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ratios;
import frc.robot.Constants.Tolerances;
import frc.robot.subsystems.interfaces.IShooterHood;

/**
 * Moves the hood to a specified position.
 * 
 * FINDME(Daniel): You're actually implementing a precursor to PID control in
 * this command. You could easily move this into the RealHood class, and then
 * have the interface support a "setAngle()" method, which would establish the
 * desired position (setpoint) and start the motor turning, and then use
 * "periodic()" to decide (if it's trying to move to a setoiint) if that's been
 * reached, and then stop it.
 */
public class PivotHoodToPosition extends Command {
  IShooterHood m_hood;
  private double m_speed;
  private Angle m_endAngle;

  /** Creates a new PivotHoodToPosition. */
  public PivotHoodToPosition(IShooterHood hood, double speed, Angle endAngle) {
    m_hood = hood;
    m_speed = speed;
    m_endAngle = endAngle.times(Ratios.ENCODERTOHOODRATIO);

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

  @Override
  public void end(boolean interrupted) {
    m_hood.stop();
  }

  @Override
  public boolean isFinished() {
    return (m_hood.getCurrentAngle().lt(m_endAngle.plus(Tolerances.ANGLETOLERANCE)) &&
        m_hood.getCurrentAngle().gt(m_endAngle.minus(Tolerances.ANGLETOLERANCE)));
  }
}
