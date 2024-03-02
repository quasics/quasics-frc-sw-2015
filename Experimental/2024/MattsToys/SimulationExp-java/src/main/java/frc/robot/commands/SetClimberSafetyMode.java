// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

/**
 * Command to enable/disable safety mode on the climber subsystem.
 * 
 * TODO: Test this code.
 */
public class SetClimberSafetyMode extends Command {
  final Climber m_climber;
  final boolean m_enableSafeMode;

  /**
   * @param enableSafeMode specifies if safety mode is to be enabled (true) or
   *                       disabled (false) on the climber
   */
  public SetClimberSafetyMode(Climber climber, boolean enableSafeMode) {
    m_climber = climber;
    m_enableSafeMode = enableSafeMode;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climber.enableSafeMode(m_enableSafeMode);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
