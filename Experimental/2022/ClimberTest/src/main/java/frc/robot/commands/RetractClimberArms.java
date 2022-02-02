// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RetractClimberArms extends CommandBase {
  private final Climber m_climber;

  /** Creates a new RetractClimberArms. */
  public RetractClimberArms(Climber climber) {
    addRequirements(climber);

    m_climber = climber;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.retractArms();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.getPosition() <= 0;
  }
}
