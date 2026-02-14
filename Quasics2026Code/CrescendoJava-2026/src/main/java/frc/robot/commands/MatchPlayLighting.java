// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import java.util.Optional;

public class MatchPlayLighting extends Command {
  final Lights m_lights;
  /** Creates a new MatchPlayLighting. */
  public MatchPlayLighting(Lights lights) {
    m_lights = lights;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lights.setStripColor(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        m_lights.setStripColor(255, 0, 0);
      }
      if (alliance.get() == DriverStation.Alliance.Blue) {
        m_lights.setStripColor(0, 0, 255);
      }
    } else {
      m_lights.setStripColor(0, 255, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lights.setStripColor(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
