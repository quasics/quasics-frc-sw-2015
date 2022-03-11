// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

/**
 * Command to run the conveyor that moves balls from the floor pickup to the
 * shooter.
 */
public class RunConveyor extends CommandBase {
  private final Conveyor m_conveyor;
  private final double m_speed;

  /**
   * Creates a new RunConveyor.
   * 
   * @param conveyor     the conveyor subsystem
   * @param percentSpeed the % motor speed (-1.0 to +1.0) at which the conveyor
   *                     should be run
   */
  public RunConveyor(Conveyor conveyor, double percentSpeed) {
    m_conveyor = conveyor;
    m_speed = Math.max(-1, Math.min(+1, percentSpeed));

    addRequirements(m_conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_conveyor.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.stop();
  }
}
