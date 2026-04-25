// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IClimber;

public class ManualClimberControlCommand extends Command {
  final IClimber m_climber;
  final boolean m_extending;

  /** Creates a new ManualClimberControlCommand. */
  public ManualClimberControlCommand(IClimber climber, boolean extending) {
    m_climber = climber;
    m_extending = extending;

    addRequirements((Subsystem) climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_extending) {
      m_climber.extend();
    } else {
      m_climber.retract();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }
}
