// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;

/**
 * Simple command to turn the robot by a certain amount (expressed in angular units). 
 */
public class TurnRobot extends Command {
  /** Creates a new TurnRobot. */
  final AbstractDrivebase m_drivebase;

  final double m_percent;

  final Angle m_angle;

  Angle m_stopAngle;

  public TurnRobot(AbstractDrivebase drivebase, double percent, Angle angle) {
    
    m_drivebase = drivebase;
    
    m_angle = angle;
    
    m_percent = Math.signum(angle.baseUnitMagnitude()) * percent;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stopAngle = m_drivebase.getHeading().plus(m_angle);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.tankDrive(m_percent, m_percent*-1);
    System.out.println("Angle: " + m_drivebase.getHeading());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_angle.baseUnitMagnitude() > 0){
      return m_drivebase.getHeading().gte(m_stopAngle);
    } else{
    return m_drivebase.getHeading().lte(m_stopAngle);
    }
  }
}
