// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractElevator;
import frc.robot.subsystems.AbstractElevator.Mode;

/*
 * TODO: consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class RaiseElevator extends Command {
  final AbstractElevator m_elevator;

  /** Creates a new RaiseElevator. */
  public RaiseElevator(AbstractElevator elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    if (!m_elevator.extend()) {
      System.err.println("*** Warning: couldn't start extending elevator!");
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return m_elevator.getMode() == Mode.Stopped;
  }
}
